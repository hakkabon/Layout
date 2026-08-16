//! UniFFI bindings: a single batched entry point for Swift (or any other
//! UniFFI-supported language) to run the full layout pipeline in one call.
//!
//! Design notes:
//! - **One call, not five.** `layout()` runs cycle breaking through edge
//!   routing internally. Crossing the FFI boundary five times per layout
//!   (once per phase) would multiply marshaling overhead for no benefit —
//!   Swift never needs to inspect intermediate pipeline state.
//! - **Opaque `u64` node ids.** Internally this crate requires dense,
//!   zero-based `NodeId`s (see `LayoutGraph::add_node`). Callers' own node
//!   identities (SPPF/GSS/syntax-tree nodes) won't naturally be dense, so
//!   the FFI layer accepts an arbitrary `u64` per node and maps it to an
//!   internal id itself; positions and edge routes are reported back
//!   keyed on the caller's original ids.
//! - **Panics never cross the boundary.** A Rust panic unwinding into
//!   Swift is undefined behavior. Every path through `layout()` is wrapped
//!   in `catch_unwind` and converted into an `FfiLayoutError`.
//! - **Dummy nodes are not reported.** They're an internal routing detail;
//!   their positions are already folded into each route's waypoints.

use crate::{
    assign_coordinates, assign_ranks, break_cycles, insert_dummy_nodes, order_layers,
    route_edges, CoordAlgorithm, CoordConfig, EdgeChain, LayoutDirection, LayoutEdge, LayoutError,
    LayoutGraph, LayoutNode, NodeId, NodeType, RoutingStyle,
};
use std::collections::HashMap;

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiNode {
    pub id: u64,
    pub width: f32,
    pub height: f32,
}

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiEdge {
    pub from: u64,
    pub to: u64,
    /// Optional label width for obstacle-free label placement.
    pub label_width: Option<f32>,
    /// Optional label height for obstacle-free label placement.
    pub label_height: Option<f32>,
}

#[derive(Debug, Clone, Copy, uniffi::Enum)]
pub enum FfiAlgorithm {
    MedianRelax,
    BrandesKopf,
}

impl From<FfiAlgorithm> for CoordAlgorithm {
    fn from(a: FfiAlgorithm) -> Self {
        match a {
            FfiAlgorithm::MedianRelax => CoordAlgorithm::MedianRelax,
            FfiAlgorithm::BrandesKopf => CoordAlgorithm::BrandesKopf,
        }
    }
}

#[derive(Debug, Clone, Copy, uniffi::Enum)]
pub enum FfiRoutingStyle {
    Straight,
    Orthogonal,
    Bezier,
}

impl From<FfiRoutingStyle> for RoutingStyle {
    fn from(s: FfiRoutingStyle) -> Self {
        match s {
            FfiRoutingStyle::Straight => RoutingStyle::Straight,
            FfiRoutingStyle::Orthogonal => RoutingStyle::Orthogonal,
            FfiRoutingStyle::Bezier => RoutingStyle::Bezier,
        }
    }
}

#[derive(Debug, Clone, Copy, uniffi::Enum)]
pub enum FfiDirection {
    TopToBottom,
    LeftToRight,
}

impl From<FfiDirection> for LayoutDirection {
    fn from(d: FfiDirection) -> Self {
        match d {
            FfiDirection::TopToBottom => LayoutDirection::TopToBottom,
            FfiDirection::LeftToRight => LayoutDirection::LeftToRight,
        }
    }
}

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiConfig {
    pub h_gap: f32,
    pub v_gap: f32,
    pub relax_passes: u32,
    /// Barycenter sweep count for crossing reduction. 4 is a reasonable
    /// default; raise it for a "final" layout, lower it while the user is
    /// interactively editing the graph.
    pub sweeps: u32,
    pub algorithm: FfiAlgorithm,
    pub routing: FfiRoutingStyle,
    pub direction: FfiDirection,
}

#[derive(Debug, Clone, Copy, uniffi::Record)]
pub struct FfiPoint {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, Copy, uniffi::Record)]
pub struct FfiRect {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub width: f32,
    pub height: f32,
}

#[derive(Debug, Clone, Copy, uniffi::Record)]
pub struct FfiArrowhead {
    pub tip: FfiPoint,
    /// Tangent angle in radians (pointing in the direction of the edge flow).
    pub angle: f32,
    pub left: FfiPoint,
    pub right: FfiPoint,
}

#[derive(Debug, Clone, uniffi::Enum)]
pub enum FfiPathSegment {
    Line {
        start: FfiPoint,
        end: FfiPoint,
    },
    CubicCurve {
        start: FfiPoint,
        control1: FfiPoint,
        control2: FfiPoint,
        end: FfiPoint,
    },
}

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiPosition {
    pub id: u64,
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiEdgeRoute {
    pub from: u64,
    pub to: u64,
    /// True if this edge was reversed by cycle breaking; draw the
    /// arrowhead at the `from` end rather than the `to` end.
    pub reversed: bool,
    pub is_self_loop: bool,
    pub waypoints: Vec<FfiPoint>,
    pub segments: Vec<FfiPathSegment>,
    /// Precomputed arrowhead geometry (tip + wing vertices) at the target end.
    pub arrowhead: Option<FfiArrowhead>,
    /// Obstacle-free label center position, if the edge had label dimensions.
    pub label_position: Option<FfiPoint>,
}

#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiLayoutResult {
    pub positions: Vec<FfiPosition>,
    pub routes: Vec<FfiEdgeRoute>,
    /// Self-loop edges extracted before layout (see module docs).
    pub self_loops: Vec<FfiEdge>,
    pub bounds: FfiRect,
}

#[derive(Debug, uniffi::Error)]
#[uniffi(flat_error)]
pub enum FfiLayoutError {
    /// The input graph itself was invalid: a duplicate/unknown node id, or
    /// (from `LayoutError`) a genuine cycle that survived cycle breaking.
    InvalidGraph(String),
    /// The layout engine panicked internally. This is always a bug in the
    /// engine, not bad input — please report it with the graph that
    /// triggered it.
    Internal(String),
}

impl std::fmt::Display for FfiLayoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FfiLayoutError::InvalidGraph(msg) => write!(f, "invalid graph: {msg}"),
            FfiLayoutError::Internal(msg) => write!(f, "internal layout engine error: {msg}"),
        }
    }
}

impl std::error::Error for FfiLayoutError {}

impl From<LayoutError> for FfiLayoutError {
    fn from(e: LayoutError) -> Self {
        FfiLayoutError::InvalidGraph(e.to_string())
    }
}

/// Runs the full layout pipeline (cycle breaking through edge routing) in
/// one call and returns final node positions and edge routes, keyed on the
/// caller's own `u64` node ids.
#[uniffi::export]
pub fn layout(
    nodes: Vec<FfiNode>,
    edges: Vec<FfiEdge>,
    config: FfiConfig,
) -> Result<FfiLayoutResult, FfiLayoutError> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        run_pipeline(nodes, edges, &config)
    }))
    .unwrap_or_else(|_| {
        Err(FfiLayoutError::Internal(
            "layout engine panicked; this is a bug — please report it".to_string(),
        ))
    })
}

fn run_pipeline(
    nodes: Vec<FfiNode>,
    edges: Vec<FfiEdge>,
    config: &FfiConfig,
) -> Result<FfiLayoutResult, FfiLayoutError> {
    let mut id_map: HashMap<u64, NodeId> = HashMap::with_capacity(nodes.len());
    let mut external_ids: Vec<u64> = Vec::with_capacity(nodes.len());
    let mut graph = LayoutGraph::default();

    for n in &nodes {
        if id_map.contains_key(&n.id) {
            return Err(FfiLayoutError::InvalidGraph(format!(
                "duplicate node id {}",
                n.id
            )));
        }
        let internal_id = graph.add_node(LayoutNode {
            width: n.width,
            height: n.height,
            ..Default::default()
        });
        id_map.insert(n.id, internal_id);
        external_ids.push(n.id);
    }

    for e in &edges {
        let from = *id_map.get(&e.from).ok_or_else(|| {
            FfiLayoutError::InvalidGraph(format!("edge references unknown node id {}", e.from))
        })?;
        let to = *id_map.get(&e.to).ok_or_else(|| {
            FfiLayoutError::InvalidGraph(format!("edge references unknown node id {}", e.to))
        })?;
        let label_size = match (e.label_width, e.label_height) {
            (Some(w), Some(h)) => Some((w, h)),
            _ => None,
        };
        graph.edges.push(LayoutEdge {
            from,
            to,
            reversed: false,
            label_size,
        });
    }

    let raw_self_loops = graph.extract_self_loops();
    let self_loops: Vec<FfiEdge> = raw_self_loops
        .iter()
        .map(|e| FfiEdge {
            from: external_ids[e.from],
            to: external_ids[e.to],
            label_width: e.label_size.map(|(w, _)| w),
            label_height: e.label_size.map(|(_, h)| h),
        })
        .collect();

    let _reversed = break_cycles(&mut graph);
    let mut ranks = assign_ranks(&mut graph)?;
    let mut chains = insert_dummy_nodes(&mut graph, &mut ranks)?;

    // Add self-loops as explicit edge chains so route_edges generates routes for them
    for loop_edge in raw_self_loops {
        chains.push(EdgeChain {
            source: loop_edge.from,
            target: loop_edge.to,
            reversed: false,
            is_self_loop: true,
            label_size: loop_edge.label_size,
            chain: vec![loop_edge.from],
        });
    }

    order_layers(&mut graph, &mut ranks, config.sweeps as usize)?;

    let coord_config = CoordConfig {
        h_gap: config.h_gap,
        v_gap: config.v_gap,
        relax_passes: config.relax_passes as usize,
        algorithm: config.algorithm.into(),
        direction: config.direction.into(),
    };
    assign_coordinates(&mut graph, &ranks, &coord_config)?;
    let routes = route_edges(&graph, &chains, config.routing.into())?;

    let positions: Vec<FfiPosition> = graph
        .nodes
        .iter()
        .filter(|n| n.node_type == NodeType::Normal)
        .map(|n| FfiPosition {
            id: external_ids[n.id],
            x: n.x,
            y: n.y,
        })
        .collect();

    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for n in graph.nodes.iter().filter(|n| n.node_type == NodeType::Normal) {
        let half_w = n.width / 2.0;
        let half_h = n.height / 2.0;
        if n.x - half_w < min_x { min_x = n.x - half_w; }
        if n.x + half_w > max_x { max_x = n.x + half_w; }
        if n.y - half_h < min_y { min_y = n.y - half_h; }
        if n.y + half_h > max_y { max_y = n.y + half_h; }
    }

    let bounds = if min_x.is_finite() && max_x.is_finite() {
        FfiRect {
            min_x,
            min_y,
            max_x,
            max_y,
            width: (max_x - min_x).max(0.0),
            height: (max_y - min_y).max(0.0),
        }
    } else {
        FfiRect {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 0.0,
            max_y: 0.0,
            width: 0.0,
            height: 0.0,
        }
    };

    let ffi_routes: Vec<FfiEdgeRoute> = routes
        .into_iter()
        .map(|r| {
            let pts: Vec<FfiPoint> = r
                .waypoints
                .iter()
                .map(|&(x, y)| FfiPoint { x, y })
                .collect();

            let mut segments = Vec::new();
            match config.routing {
                FfiRoutingStyle::Bezier => {
                    if pts.len() >= 4 && (pts.len() - 1) % 3 == 0 {
                        let num_segments = (pts.len() - 1) / 3;
                        for s in 0..num_segments {
                            segments.push(FfiPathSegment::CubicCurve {
                                start: pts[3 * s],
                                control1: pts[3 * s + 1],
                                control2: pts[3 * s + 2],
                                end: pts[3 * s + 3],
                            });
                        }
                    } else if pts.len() == 2 {
                        segments.push(FfiPathSegment::Line {
                            start: pts[0],
                            end: pts[1],
                        });
                    } else {
                        for i in 0..pts.len().saturating_sub(1) {
                            segments.push(FfiPathSegment::Line {
                                start: pts[i],
                                end: pts[i + 1],
                            });
                        }
                    }
                }
                _ => {
                    for i in 0..pts.len().saturating_sub(1) {
                        segments.push(FfiPathSegment::Line {
                            start: pts[i],
                            end: pts[i + 1],
                        });
                    }
                }
            }

            let ffi_arrowhead = r.arrowhead.map(|ah| FfiArrowhead {
                tip: FfiPoint { x: ah.tip_x, y: ah.tip_y },
                angle: ah.angle,
                left: FfiPoint { x: ah.left_x, y: ah.left_y },
                right: FfiPoint { x: ah.right_x, y: ah.right_y },
            });
            let ffi_label_pos = r.label_pos.map(|(x, y)| FfiPoint { x, y });

            FfiEdgeRoute {
                from: external_ids[r.source],
                to: external_ids[r.target],
                reversed: r.reversed,
                is_self_loop: r.is_self_loop,
                waypoints: pts,
                segments,
                arrowhead: ffi_arrowhead,
                label_position: ffi_label_pos,
            }
        })
        .collect();

    Ok(FfiLayoutResult {
        positions,
        routes: ffi_routes,
        self_loops,
        bounds,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ffi_layout_pipeline_produces_valid_result() {
        let nodes = vec![
            FfiNode { id: 100, width: 40.0, height: 20.0 },
            FfiNode { id: 200, width: 40.0, height: 20.0 },
            FfiNode { id: 300, width: 40.0, height: 20.0 },
        ];
        let edges = vec![
            FfiEdge { from: 100, to: 200 },
            FfiEdge { from: 200, to: 300 },
            FfiEdge { from: 100, to: 100 }, // self loop
        ];
        let config = FfiConfig {
            h_gap: 20.0,
            v_gap: 40.0,
            relax_passes: 4,
            sweeps: 4,
            algorithm: FfiAlgorithm::BrandesKopf,
            routing: FfiRoutingStyle::Bezier,
            direction: FfiDirection::TopToBottom,
        };

        let result = layout(nodes, edges, config).unwrap();
        assert_eq!(result.positions.len(), 3);
        assert_eq!(result.self_loops.len(), 1);
        assert_eq!(result.self_loops[0].from, 100);

        // Self-loop should be included in routes with is_self_loop = true
        let self_loop_route = result.routes.iter().find(|r| r.is_self_loop);
        assert!(self_loop_route.is_some());
        assert_eq!(self_loop_route.unwrap().from, 100);
        assert_eq!(self_loop_route.unwrap().to, 100);

        // Bounds should be non-zero
        assert!(result.bounds.width > 0.0);
        assert!(result.bounds.height > 0.0);

        // Segments should be generated for each route
        for r in &result.routes {
            assert!(!r.segments.is_empty());
        }
    }

    #[test]
    fn ffi_layout_left_to_right_direction() {
        let nodes = vec![
            FfiNode { id: 1, width: 50.0, height: 30.0 },
            FfiNode { id: 2, width: 50.0, height: 30.0 },
        ];
        let edges = vec![
            FfiEdge { from: 1, to: 2 },
        ];
        let config = FfiConfig {
            h_gap: 20.0,
            v_gap: 40.0,
            relax_passes: 4,
            sweeps: 4,
            algorithm: FfiAlgorithm::MedianRelax,
            routing: FfiRoutingStyle::Bezier,
            direction: FfiDirection::LeftToRight,
        };

        let result = layout(nodes, edges, config).unwrap();
        let pos1 = result.positions.iter().find(|p| p.id == 1).unwrap();
        let pos2 = result.positions.iter().find(|p| p.id == 2).unwrap();

        // In LeftToRight, node 1 is to the left of node 2 (x1 < x2)
        assert!(pos1.x < pos2.x);
    }
}


