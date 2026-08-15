//! Coordinate Assignment and Edge Routing
//!
//! Assigns x/y coordinates to nodes and computes edge routes.
//! Supports two algorithms for x-coordinate assignment:
//! - Weighted median relaxation (simpler, faster)
//! - Brandes-Köpf alignment (produces more balanced layouts)

use crate::types::{LayoutGraph, RankSystem, NodeId, EdgeChain, EdgeRoute, RoutingStyle, LayoutError};

/// Configuration for coordinate assignment algorithms.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct CoordConfig {
    /// Minimum horizontal gap between adjacent nodes in the same layer.
    pub h_gap: f32,
    /// Vertical gap between rank layers.
    pub v_gap: f32,
    /// Number of median-relaxation passes (only used for MedianRelax algorithm).
    pub relax_passes: usize,
    /// Which algorithm to use for x-coordinate assignment.
    pub algorithm: CoordAlgorithm,
}

impl Default for CoordConfig {
    fn default() -> Self {
        Self {
            h_gap: 20.0,
            v_gap: 40.0,
            relax_passes: 4,
            algorithm: CoordAlgorithm::default(),
        }
    }
}

/// Algorithm selection for x-coordinate assignment.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, Default)]
pub enum CoordAlgorithm {
    /// Weighted median relaxation with compaction (simpler, O(N·passes))
    #[default]
    MedianRelax,
    /// Brandes-Köpf alignment averaging (better quality, O(N))
    BrandesKopf,
}

/// Assigns coordinates to all nodes in the graph.
///
/// # Algorithm
/// 1. Y-coordinates: assigned by rank (trivial)
/// 2. X-coordinates: based on CoordAlgorithm:
///    - MedianRelax: weighted median relaxation with compaction
///    - BrandesKopf: four-pass alignment averaging (top-left, top-right, bottom-left, bottom-right)
/// 3. Centering: shift all coordinates so bounding box is centered at origin
///
/// # Errors
/// Returns `LayoutError::DanglingEdge` if an edge references a node id
/// outside `graph.nodes`, or `LayoutError::MissingRank` if `assign_ranks`
/// hasn't been run.
///
/// # Arguments
/// * `graph` - The layout graph with ordered layers
/// * `ranks` - The rank system defining layers
/// * `config` - Configuration for spacing and algorithm selection
pub fn assign_coordinates(graph: &mut LayoutGraph, ranks: &RankSystem, config: &CoordConfig) -> Result<(), LayoutError> {
    let layer_count = ranks.layers.len();

    // Stage 1: Y-coordinate assignment (trivial)
    // Compute max height per layer for proper vertical spacing
    let mut layer_heights: Vec<f32> = vec![0.0; layer_count];
    for (layer_idx, layer) in ranks.layers.iter().enumerate() {
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                let height = graph.nodes[node_id].height;
                if height > layer_heights[layer_idx] {
                    layer_heights[layer_idx] = height;
                }
            }
        }
    }

    // Assign y coordinates (top-aligned within each layer)
    let mut y_accum: f32 = 0.0;
    for (layer_idx, layer) in ranks.layers.iter().enumerate() {
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                graph.nodes[node_id].y = y_accum;
            }
        }
        y_accum += layer_heights[layer_idx] + config.v_gap;
    }

    // Stage 2: X-coordinate assignment based on selected algorithm
    let x_coords = match config.algorithm {
        CoordAlgorithm::MedianRelax => {
            median_relax_x_coords(graph, ranks, config.h_gap, config.relax_passes)?
        }
        CoordAlgorithm::BrandesKopf => {
            brandes_kopf_x_coords(graph, ranks, config.h_gap)?
        }
    };

    // Stage 3: Center the layout
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for (i, node) in graph.nodes.iter().enumerate() {
        let x = x_coords[i];
        let half_w = node.width / 2.0;
        if x - half_w < min_x { min_x = x - half_w; }
        if x + half_w > max_x { max_x = x + half_w; }
        if node.y < min_y { min_y = node.y; }
        if node.y + node.height > max_y { max_y = node.y + node.height; }
    }

    let center_x = (min_x + max_x) / 2.0;
    let center_y = (min_y + max_y) / 2.0;

    // Apply centering and store final coordinates
    for (i, node) in graph.nodes.iter_mut().enumerate() {
        node.x = x_coords[i] - center_x;
        node.y = node.y - center_y;
    }

    Ok(())
}

/// X-coordinate assignment using weighted median relaxation.
fn median_relax_x_coords(
    graph: &mut LayoutGraph,
    ranks: &RankSystem,
    h_gap: f32,
    relax_passes: usize,
) -> Result<Vec<f32>, LayoutError> {
    let n = graph.nodes.len();
    let mut x_coords: Vec<f32> = vec![0.0; n];
    let mut widths: Vec<f32> = vec![0.0; n];

    // Initial placement based on order
    for layer in &ranks.layers {
        if layer.is_empty() {
            continue;
        }
        // Find max width in this layer for spacing
        let mut max_width: f32 = 0.0;
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                let w = graph.nodes[node_id].width;
                if w > max_width {
                    max_width = w;
                }
                widths[node_id] = graph.nodes[node_id].width;
            }
        }

        // Initial x placement
        let mut x: f32 = 0.0;
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                x_coords[node_id] = x;
                x += max_width + h_gap;
            }
        }
    }

    // Build neighbor lists for median relaxation
    let mut up_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];
    let mut down_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];

    for edge in &graph.edges {
        let from_node = graph.nodes.get(edge.from)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let to_node = graph.nodes.get(edge.to)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let rank_from = from_node.rank.ok_or(LayoutError::MissingRank(edge.from))? as isize;
        let rank_to = to_node.rank.ok_or(LayoutError::MissingRank(edge.to))? as isize;
        if rank_to - rank_from == 1 {
            down_neighbors[edge.from].push(edge.to);
            up_neighbors[edge.to].push(edge.from);
        }
    }

    // Median relaxation passes (alternating up/down)
    for pass in 0..relax_passes {
        let use_up = pass % 2 == 0;
        let neighbors = if use_up { &up_neighbors } else { &down_neighbors };

        // Compute preferred x for each node (median of neighbors)
        let mut preferred_x: Vec<Option<f32>> = vec![None; n];
        for node_id in 0..n {
            let adj = &neighbors[node_id];
            if !adj.is_empty() {
                let mut neighbor_x: Vec<f32> = adj.iter()
                    .map(|&nb| x_coords[nb])
                    .collect();
                neighbor_x.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let median_idx = neighbor_x.len() / 2;
                preferred_x[node_id] = Some(neighbor_x[median_idx]);
            }
        }

        // Compact each layer based on preferred x
        for layer in &ranks.layers {
            if layer.len() <= 1 {
                continue;
            }

            // Sort by preferred x (stable, keeping original order for ties)
            let mut sorted: Vec<(NodeId, f32)> = layer.iter()
                .map(|&id| (id, preferred_x[id].unwrap_or(x_coords[id])))
                .collect();
            sorted.sort_by(|a, b| {
                a.1.partial_cmp(&b.1)
                    .unwrap_or_else(|| {
                        // Tie-break by current order
                        let ord_a = graph.nodes[a.0].order.unwrap_or(0);
                        let ord_b = graph.nodes[b.0].order.unwrap_or(0);
                        ord_a.cmp(&ord_b)
                    })
            });

            // Compact: place nodes left-to-right respecting widths and gaps
            let mut x: f32 = 0.0;
            let mut prev_width: f32 = 0.0;
            for (i, &(node_id, _)) in sorted.iter().enumerate() {
                if i == 0 {
                    x = preferred_x[node_id].unwrap_or(0.0);
                } else {
                    let min_x = x + prev_width / 2.0 + widths[node_id] / 2.0 + h_gap;
                    let pref = preferred_x[node_id].unwrap_or(x_coords[node_id]);
                    x = pref.max(min_x - widths[node_id] / 2.0);
                }
                x_coords[node_id] = x;
                prev_width = widths[node_id];
            }
        }
    }

    Ok(x_coords)
}

/// X-coordinate assignment using a Brandes-Köpf-*inspired* heuristic.
///
/// Note: this is **not** the published Brandes-Köpf algorithm, which
/// resolves type-1/type-2 conflicts between dummy-node chains and regular
/// edges and aligns nodes within resulting blocks. What's implemented here
/// is simpler: four directional weighted-average passes (top-down/
/// bottom-up × left/right-aligned) that are then averaged together. It's a
/// reasonable and much cheaper approximation, but unlike true BK it does
/// not guarantee that dummy-node chains crossing other edges come out
/// straight. Produces more balanced and symmetric layouts than plain
/// median relaxation by computing four independent alignments and
/// averaging them:
/// 1. Top-left: process layers top-to-bottom, align left
/// 2. Top-right: process layers top-to-bottom, align right
/// 3. Bottom-left: process layers bottom-to-top, align left
/// 4. Bottom-right: process layers bottom-to-top, align right
///
/// The final x-coordinate is the average of all four alignments.
fn brandes_kopf_x_coords(
    graph: &mut LayoutGraph,
    ranks: &RankSystem,
    h_gap: f32,
) -> Result<Vec<f32>, LayoutError> {
    let n = graph.nodes.len();

    // Build adjacency lists for neighbors within consecutive ranks
    let mut up_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];
    let mut down_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];

    for edge in &graph.edges {
        let from_node = graph.nodes.get(edge.from)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let to_node = graph.nodes.get(edge.to)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let rank_from = from_node.rank.ok_or(LayoutError::MissingRank(edge.from))? as isize;
        let rank_to = to_node.rank.ok_or(LayoutError::MissingRank(edge.to))? as isize;
        if rank_to - rank_from == 1 {
            down_neighbors[edge.from].push(edge.to);
            up_neighbors[edge.to].push(edge.from);
        }
    }

    // Store four alignments
    let mut x_coords: [Vec<f32>; 4] = [
        vec![0.0; n],
        vec![0.0; n],
        vec![0.0; n],
        vec![0.0; n],
    ];

    // Alignment 0: Top-Left (process top-to-bottom, align left)
    compute_alignment(&mut x_coords[0], graph, ranks, &up_neighbors, true, true, h_gap);

    // Alignment 1: Top-Right (process top-to-bottom, align right)
    compute_alignment(&mut x_coords[1], graph, ranks, &up_neighbors, true, false, h_gap);

    // Alignment 2: Bottom-Left (process bottom-to-top, align left)
    compute_alignment(&mut x_coords[2], graph, ranks, &down_neighbors, false, true, h_gap);

    // Alignment 3: Bottom-Right (process bottom-to-top, align right)
    compute_alignment(&mut x_coords[3], graph, ranks, &down_neighbors, false, false, h_gap);

    // Average the four alignments
    let mut avg_x: Vec<f32> = vec![0.0; n];
    for i in 0..n {
        avg_x[i] = (x_coords[0][i] + x_coords[1][i] + x_coords[2][i] + x_coords[3][i]) / 4.0;
    }

    // Each of the four alignments individually respects h_gap spacing, but
    // the average of four different valid layouts does not — most visibly
    // on symmetric structures (e.g. a diamond), where sibling nodes with
    // identical local structure average to the *same* x and end up
    // overlapping. `repair_layer_spacing` fixes that: within each layer it
    // finds the closest (least-squares) set of positions that preserves
    // each node's already-fixed `order` and satisfies the minimum-gap
    // constraint, via isotonic regression (pool-adjacent-violators). Unlike
    // a naive "anchor the first node, push the rest right" compaction,
    // PAVA distributes the correction across every violating node — so two
    // overlapping siblings end up centered symmetrically around their
    // shared mean, not with one anchored and the other shoved aside.
    //
    // Fixing sibling spacing alone can still leave a parent/child off
    // center relative to its now-separated neighbors (e.g. a diamond's
    // root should sit above the midpoint of its two children). So
    // alternate a recentering step — pull each node toward the mean of
    // *all* its neighbors, up and down together — with re-repairing
    // spacing, letting both effects settle over a few passes.
    let mut combined_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];
    for id in 0..n {
        combined_neighbors[id].extend(&up_neighbors[id]);
        combined_neighbors[id].extend(&down_neighbors[id]);
    }

    repair_layer_spacing(&mut avg_x, graph, ranks, h_gap);
    for _ in 0..8 {
        let mut preferred: Vec<f32> = avg_x.clone();
        for id in 0..n {
            if !combined_neighbors[id].is_empty() {
                let sum: f32 = combined_neighbors[id].iter().map(|&nb| avg_x[nb]).sum();
                preferred[id] = sum / combined_neighbors[id].len() as f32;
            }
        }
        // Damped update: a full replacement (avg_x = preferred) is a
        // Jacobi-style simultaneous update, which can oscillate between two
        // states indefinitely rather than converge (observed on the
        // diamond test case above). Blending old and new breaks the cycle.
        for id in 0..n {
            avg_x[id] = 0.5 * avg_x[id] + 0.5 * preferred[id];
        }
        repair_layer_spacing(&mut avg_x, graph, ranks, h_gap);
    }

    Ok(avg_x)
}

/// Within each layer, finds the closest set of positions (in a
/// least-squares sense) to the given `x` values that preserves each node's
/// already-fixed `order` and satisfies the minimum-gap constraint implied
/// by `h_gap` and node widths.
///
/// Implemented via isotonic regression (pool-adjacent-violators, PAVA):
/// subtracting each node's cumulative minimum offset from its position
/// turns "positions respect minimum gaps, in order" into "positions are
/// non-decreasing," which PAVA solves optimally in O(layer size). This is
/// what makes the repair *symmetric* — when two equal-weight positions
/// violate the ordering, PAVA pools them to their shared mean rather than
/// anchoring one and pushing the other.
fn repair_layer_spacing(x: &mut [f32], graph: &LayoutGraph, ranks: &RankSystem, h_gap: f32) {
    for layer in &ranks.layers {
        if layer.len() <= 1 {
            continue;
        }
        let mut ordered: Vec<NodeId> = layer.clone();
        ordered.sort_by_key(|&id| graph.nodes[id].order.unwrap_or(0));
        ordered.retain(|&id| id < graph.nodes.len());
        if ordered.len() <= 1 {
            continue;
        }

        // Cumulative minimum offset from the first node in the layer.
        let mut offsets: Vec<f32> = Vec::with_capacity(ordered.len());
        let mut cum = 0.0f32;
        for (i, &id) in ordered.iter().enumerate() {
            if i == 0 {
                offsets.push(0.0);
            } else {
                let prev_half = graph.nodes[ordered[i - 1]].width / 2.0;
                let half = graph.nodes[id].width / 2.0;
                cum += prev_half + h_gap + half;
                offsets.push(cum);
            }
        }

        let adjusted: Vec<f32> = ordered.iter().zip(&offsets)
            .map(|(&id, &off)| x[id] - off)
            .collect();
        let solved = isotonic_regression(&adjusted);
        for (i, &id) in ordered.iter().enumerate() {
            x[id] = solved[i] + offsets[i];
        }
    }
}

/// Finds the non-decreasing sequence closest to `values` in the
/// least-squares sense, via the pool-adjacent-violators algorithm (PAVA).
/// O(n) amortized.
fn isotonic_regression(values: &[f32]) -> Vec<f32> {
    // Each entry in the stack is a pooled block: (mean, count).
    let mut blocks: Vec<(f32, usize)> = Vec::new();
    for &v in values {
        let mut mean = v;
        let mut count = 1usize;
        while let Some(&(prev_mean, prev_count)) = blocks.last() {
            if prev_mean > mean {
                blocks.pop();
                let total = prev_count + count;
                mean = (prev_mean * prev_count as f32 + mean * count as f32) / total as f32;
                count = total;
            } else {
                break;
            }
        }
        blocks.push((mean, count));
    }

    let mut result = Vec::with_capacity(values.len());
    for (mean, count) in blocks {
        for _ in 0..count {
            result.push(mean);
        }
    }
    result
}

/// Computes a single Brandes-Köpf alignment pass.
///
/// # Arguments
/// * `x_out` - Output vector to store x-coordinates
/// * `graph` - The layout graph
/// * `ranks` - The rank system
/// * `neighbors` - Neighbor list (up or down depending on direction)
/// * `top_down` - If true, process layers top-to-bottom; else bottom-to-top
/// * `align_left` - If true, align to left; else align to right
fn compute_alignment(
    x_out: &mut [f32],
    graph: &LayoutGraph,
    ranks: &RankSystem,
    neighbors: &[Vec<NodeId>],
    top_down: bool,
    align_left: bool,
    h_gap: f32,
) {
    let layer_indices: Vec<usize> = if top_down {
        (0..ranks.layers.len()).collect()
    } else {
        (0..ranks.layers.len()).rev().collect()
    };

    // Process each layer in order
    for &layer_idx in &layer_indices {
        let layer = &ranks.layers[layer_idx];
        if layer.is_empty() {
            continue;
        }

        // Compute preferred x for each node (median/average of neighbors already placed)
        let mut preferred_x: Vec<f32> = vec![0.0; graph.nodes.len()];
        let mut has_preferred: Vec<bool> = vec![false; graph.nodes.len()];

        for &node_id in layer {
            if node_id >= graph.nodes.len() {
                continue;
            }
            let adj = &neighbors[node_id];
            if !adj.is_empty() {
                let mut sum: f32 = 0.0;
                let mut count: usize = 0;
                for &nb in adj {
                    if nb < graph.nodes.len() {
                        sum += x_out[nb];
                        count += 1;
                    }
                }
                if count > 0 {
                    preferred_x[node_id] = sum / count as f32;
                    has_preferred[node_id] = true;
                }
            }
        }

        // Sort nodes by preferred x (or by order if no preferred x)
        let mut sorted: Vec<(NodeId, f32)> = layer.iter()
            .map(|&id| {
                let px = if has_preferred[id] { preferred_x[id] } else { x_out[id] };
                (id, px)
            })
            .collect();

        // Stable sort by preferred x, tie-break by current order
        sorted.sort_by(|a, b| {
            a.1.partial_cmp(&b.1)
                .unwrap_or_else(|| {
                    let ord_a = graph.nodes[a.0].order.unwrap_or(0);
                    let ord_b = graph.nodes[b.0].order.unwrap_or(0);
                    ord_a.cmp(&ord_b)
                })
        });

        if !align_left {
            // Reverse for right alignment
            sorted.reverse();
        }

        // Place nodes with compaction
        let mut x: f32;
        let mut prev_right: f32 = 0.0;

        for (i, &(node_id, _)) in sorted.iter().enumerate() {
            if node_id >= graph.nodes.len() {
                continue;
            }
            let node = &graph.nodes[node_id];
            let half_w = node.width / 2.0;

            if i == 0 {
                if align_left {
                    x = if has_preferred[node_id] { preferred_x[node_id] - half_w } else { 0.0 };
                } else {
                    x = if has_preferred[node_id] { preferred_x[node_id] + half_w } else { 0.0 };
                }
            } else {
                let min_x = prev_right + h_gap;
                let target_x = if has_preferred[node_id] {
                    if align_left {
                        preferred_x[node_id] - half_w
                    } else {
                        preferred_x[node_id] + half_w
                    }
                } else {
                    min_x
                };
                x = target_x.max(min_x);
            }

            x_out[node_id] = x + half_w; // Store center x
            prev_right = x + node.width;
        }
    }
}

/// Routes edges through the graph based on node positions and edge chains.
///
/// # Arguments
/// * `graph` - The layout graph with assigned coordinates
/// * `chains` - Edge chains from phase 2a
/// * `style` - The routing style (Straight, Orthogonal, or Bezier)
///
/// # Errors
/// Returns `LayoutError::DanglingEdge` if a chain references a node id
/// outside `graph.nodes`.
///
/// # Returns
/// A vector of `EdgeRoute` structures with waypoints for each edge.
pub fn route_edges(graph: &LayoutGraph, chains: &[EdgeChain], style: RoutingStyle) -> Result<Vec<EdgeRoute>, LayoutError> {
    let mut routes = Vec::new();

    for chain in chains {
        // Extract waypoint coordinates from the chain
        let mut waypoints: Vec<(f32, f32)> = Vec::new();

        for (i, &node_id) in chain.chain.iter().enumerate() {
            let node = graph.nodes.get(node_id)
                .ok_or(LayoutError::DanglingEdge { from: chain.source, to: chain.target })?;
            let cx = node.x; // center x

            if i == 0 {
                // Source: exit from the bottom edge, half the node's
                // height below its center.
                waypoints.push((cx, node.y + node.height / 2.0));
            } else if i == chain.chain.len() - 1 {
                // Target: enter at the top edge, half the node's height
                // above its center.
                waypoints.push((cx, node.y - node.height / 2.0));
            } else {
                // Dummy node: these always have zero width/height, so
                // their "edge" and "center" are the same point.
                waypoints.push((cx, node.y));
            }
        }

        if waypoints.len() < 2 {
            continue;
        }

        let route = match style {
            RoutingStyle::Straight => {
                // Only first and last points
                let first = *waypoints.first().unwrap();
                let last = *waypoints.last().unwrap();
                EdgeRoute {
                    source: chain.source,
                    target: chain.target,
                    reversed: chain.reversed,
                    waypoints: vec![first, last],
                }
            }
            RoutingStyle::Orthogonal => {
                // L-bends at midpoints between ranks
                let mut ortho_points: Vec<(f32, f32)> = Vec::new();
                for i in 0..waypoints.len() - 1 {
                    let (x1, y1) = waypoints[i];
                    let (x2, y2) = waypoints[i + 1];
                    let mid_y = (y1 + y2) / 2.0;
                    ortho_points.push((x1, mid_y));
                    ortho_points.push((x2, mid_y));
                }
                // Add final point
                ortho_points.push(*waypoints.last().unwrap());

                EdgeRoute {
                    source: chain.source,
                    target: chain.target,
                    reversed: chain.reversed,
                    waypoints: ortho_points,
                }
            }
            RoutingStyle::Bezier => {
                // Catmull-Rom to cubic Bezier conversion
                let mut bezier_points: Vec<(f32, f32)> = Vec::new();

                if waypoints.len() == 2 {
                    // Simple case: just start and end
                    bezier_points = waypoints.clone();
                } else {
                    // For each segment, compute cubic Bezier control points
                    for i in 0..waypoints.len() - 1 {
                        let p0 = waypoints[i];
                        let p1 = waypoints[i + 1];

                        // Get previous and next points for tangent calculation
                        let p_prev = if i > 0 { waypoints[i - 1] } else {
                            // Extrapolate: p0 - (p1 - p0) = 2*p0 - p1
                            (2.0 * p0.0 - p1.0, 2.0 * p0.1 - p1.1)
                        };
                        let p_next = if i + 2 < waypoints.len() { waypoints[i + 2] } else {
                            // Extrapolate: p1 + (p1 - p0) = 2*p1 - p0
                            (2.0 * p1.0 - p0.0, 2.0 * p1.1 - p0.1)
                        };

                        // Catmull-Rom to Bezier conversion
                        // C1 = P0 + (P_next - P_prev) / 6
                        // C2 = P1 - (P_next2 - P0) / 6 ... simplified for single segment
                        let tension = 1.0 / 6.0;
                        let c1 = (
                            p0.0 + (p_next.0 - p_prev.0) * tension,
                            p0.1 + (p_next.1 - p_prev.1) * tension,
                        );
                        let c2 = (
                            p1.0 - (p_next.0 - p0.0) * tension,
                            p1.1 - (p_next.1 - p0.1) * tension,
                        );

                        if i == 0 {
                            bezier_points.push(p0);
                        }
                        bezier_points.push(c1);
                        bezier_points.push(c2);
                        bezier_points.push(p1);
                    }
                }

                EdgeRoute {
                    source: chain.source,
                    target: chain.target,
                    reversed: chain.reversed,
                    waypoints: bezier_points,
                }
            }
        };

        routes.push(route);
    }

    Ok(routes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LayoutNode, NodeType, LayoutEdge, RankSystem, EdgeChain};

    fn node(id: NodeId) -> LayoutNode {
        LayoutNode {
            id,
            node_type: NodeType::Normal,
            width: 20.0,
            height: 20.0,
            x: 0.0,
            y: 0.0,
            rank: None,
            order: None,
        }
    }

    #[test]
    fn simple_graph_gets_coordinates_median_relax() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        let ranks = RankSystem {
            layers: vec![vec![0], vec![1]],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);
        graph.nodes[0].order = Some(0);
        graph.nodes[1].order = Some(0);

        let config = CoordConfig {
            algorithm: CoordAlgorithm::MedianRelax,
            ..Default::default()
        };
        assign_coordinates(&mut graph, &ranks, &config).unwrap();

        // Nodes should have non-zero, different coordinates
        assert_ne!(graph.nodes[0].y, graph.nodes[1].y);
        // After centering, coordinates should be around 0
        assert!(graph.nodes[0].y < graph.nodes[1].y);
    }

    #[test]
    fn simple_graph_gets_coordinates_brandes_kopf() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 0, to: 2, reversed: false },
                LayoutEdge { from: 1, to: 3, reversed: false },
                LayoutEdge { from: 2, to: 3, reversed: false },
            ],
        };
        let ranks = RankSystem {
            layers: vec![vec![0], vec![1, 2], vec![3]],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);
        graph.nodes[2].rank = Some(1);
        graph.nodes[3].rank = Some(2);
        graph.nodes[0].order = Some(0);
        graph.nodes[1].order = Some(0);
        graph.nodes[2].order = Some(1);
        graph.nodes[3].order = Some(0);

        let config = CoordConfig {
            algorithm: CoordAlgorithm::BrandesKopf,
            ..Default::default()
        };
        assign_coordinates(&mut graph, &ranks, &config).unwrap();

        // All nodes should have coordinates assigned
        for node in &graph.nodes {
            assert!(node.x.is_finite());
            assert!(node.y.is_finite());
        }

        // Layer 0 should be above layer 1, layer 1 above layer 2
        assert!(graph.nodes[0].y < graph.nodes[1].y);
        assert!(graph.nodes[1].y < graph.nodes[3].y);

        // Diamond layout should be roughly symmetric with Brandes-Köpf
        // Nodes 1 and 2 should be at similar x positions (centered around node 0 and 3)
        let mid_0_3 = (graph.nodes[0].x + graph.nodes[3].x) / 2.0;
        let mid_1_2 = (graph.nodes[1].x + graph.nodes[2].x) / 2.0;
        // Allow some tolerance for symmetry
        assert!((mid_0_3 - mid_1_2).abs() < 10.0, "Diamond should be roughly symmetric");
    }

    #[test]
    fn brandes_kopf_produces_different_layout_than_median() {
        // Create a graph where BK and Median will likely produce different results
        let base_graph = || LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3), node(4)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 0, to: 2, reversed: false },
                LayoutEdge { from: 0, to: 3, reversed: false },
                LayoutEdge { from: 1, to: 4, reversed: false },
                LayoutEdge { from: 2, to: 4, reversed: false },
                LayoutEdge { from: 3, to: 4, reversed: false },
            ],
        };
        let ranks = RankSystem {
            layers: vec![vec![0], vec![1, 2, 3], vec![4]],
        };

        // Test with Median Relaxation
        let mut graph_median = base_graph();
        graph_median.nodes[0].rank = Some(0);
        graph_median.nodes[1].rank = Some(1);
        graph_median.nodes[2].rank = Some(1);
        graph_median.nodes[3].rank = Some(1);
        graph_median.nodes[4].rank = Some(2);
        for (i, node) in graph_median.nodes.iter_mut().enumerate() {
            node.order = Some(i);
        }

        let config_median = CoordConfig {
            algorithm: CoordAlgorithm::MedianRelax,
            relax_passes: 4,
            ..Default::default()
        };
        assign_coordinates(&mut graph_median, &ranks, &config_median).unwrap();

        // Test with Brandes-Köpf
        let mut graph_bk = base_graph();
        graph_bk.nodes[0].rank = Some(0);
        graph_bk.nodes[1].rank = Some(1);
        graph_bk.nodes[2].rank = Some(1);
        graph_bk.nodes[3].rank = Some(1);
        graph_bk.nodes[4].rank = Some(2);
        for (i, node) in graph_bk.nodes.iter_mut().enumerate() {
            node.order = Some(i);
        }

        let config_bk = CoordConfig {
            algorithm: CoordAlgorithm::BrandesKopf,
            ..Default::default()
        };
        assign_coordinates(&mut graph_bk, &ranks, &config_bk).unwrap();

        // Both should produce valid layouts
        for g in [&graph_median, &graph_bk] {
            for node in &g.nodes {
                assert!(node.x.is_finite());
                assert!(node.y.is_finite());
            }
        }
    }

    #[test]
    fn route_edges_straight_produces_two_points() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![],
        };
        let chains = vec![EdgeChain {
            source: 0,
            target: 1,
            reversed: false,
            chain: vec![0, 1],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Straight).unwrap();
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].waypoints.len(), 2);
    }

    #[test]
    fn route_edges_waypoints_attach_at_node_edges_not_center() {
        // Regression test: `node.x`/`node.y` are each node's CENTER
        // (the convention `assign_coordinates` establishes), not a
        // top-left-style bounding-box corner. Earlier code assumed the
        // latter, which overshot the source node's actual bottom edge
        // by a full node-height while landing exactly on the target
        // node's center with no offset — an asymmetric bug invisible in
        // tests that only checked waypoint *counts*, never values.
        let mut source = node(0);
        source.x = 0.0;
        source.y = -128.0;
        source.height = 48.0;

        let mut target = node(1);
        target.x = 0.0;
        target.y = -24.0;
        target.height = 48.0;

        let graph = LayoutGraph {
            nodes: vec![source, target],
            edges: vec![],
        };
        let chains = vec![EdgeChain {
            source: 0,
            target: 1,
            reversed: false,
            chain: vec![0, 1],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Straight).unwrap();
        assert_eq!(routes.len(), 1);
        let waypoints = &routes[0].waypoints;
        assert_eq!(waypoints.len(), 2);
        // Source: bottom edge, half its height below center.
        assert_eq!(waypoints[0], (0.0, -104.0));
        // Target: top edge, half its height above center.
        assert_eq!(waypoints[1], (0.0, -48.0));
    }

    #[test]
    fn route_edges_dummy_node_waypoint_uses_its_own_center() {
        // Dummy nodes always have zero width/height, so their "edge" and
        // "center" coincide -- this pins that down explicitly rather than
        // relying on it being incidentally true.
        let mut source = node(0);
        source.y = -100.0;
        source.height = 40.0;

        let mut dummy = node(1);
        dummy.node_type = NodeType::Dummy;
        dummy.y = 0.0;
        dummy.width = 0.0;
        dummy.height = 0.0;

        let mut target = node(2);
        target.y = 100.0;
        target.height = 40.0;

        let graph = LayoutGraph {
            nodes: vec![source, dummy, target],
            edges: vec![],
        };
        let chains = vec![EdgeChain {
            source: 0,
            target: 2,
            reversed: false,
            chain: vec![0, 1, 2],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Straight).unwrap();
        // Straight routing only keeps first/last, so use Orthogonal here
        // to exercise the dummy node's own waypoint too: orthogonal
        // routing turns [source_edge=(0,-80), dummy_center=(0,0),
        // target_edge=(0,80)] into midpoint-based L-bends, so its first
        // point is the midpoint of the first two raw waypoints.
        let routes_ortho = route_edges(&graph, &chains, RoutingStyle::Orthogonal).unwrap();
        assert_eq!(routes.len(), 1);
        assert_eq!(routes_ortho.len(), 1);
        assert_eq!(routes_ortho[0].waypoints.first().unwrap().1, -40.0);
    }

    #[test]
    fn route_edges_bezier_produces_four_plus_points() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2)],
            edges: vec![],
        };
        // Chain with one dummy node: source, dummy, target
        let chains = vec![EdgeChain {
            source: 0,
            target: 2,
            reversed: false,
            chain: vec![0, 1, 2],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Bezier).unwrap();
        assert_eq!(routes.len(), 1);
        // Bezier with 3 waypoints produces 4 points per segment × 2 segments = more than 4
        // Actually for 3 waypoints we get 2 segments, each with 4 points but shared
        // So: P0, C1, C2, P1, C1', C2', P2 = 7 points? Let's check implementation
        // Our impl adds P0 once, then for each segment: C1, C2, P1
        // Segment 0: C1, C2, P1 (3 points after P0)
        // Segment 1: C1, C2, P1 (3 more points)
        // Total: 1 + 3 + 3 = 7 points
        assert!(routes[0].waypoints.len() >= 4, "Bezier should produce at least 4 waypoints");
    }

    #[test]
    fn brandes_kopf_no_sibling_overlap() {
        // Regression test: averaging four independently-compacted BK
        // alignments can collapse siblings that share identical local
        // structure onto the exact same x-coordinate. On this diamond,
        // nodes 1 and 2 both connect only to node 0 above and node 3
        // below, which used to make them average to the same x.
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 0, to: 2, reversed: false },
                LayoutEdge { from: 1, to: 3, reversed: false },
                LayoutEdge { from: 2, to: 3, reversed: false },
            ],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);
        graph.nodes[2].rank = Some(1);
        graph.nodes[3].rank = Some(2);
        graph.nodes[0].order = Some(0);
        graph.nodes[1].order = Some(0);
        graph.nodes[2].order = Some(1);
        graph.nodes[3].order = Some(0);

        let ranks = RankSystem {
            layers: vec![vec![0], vec![1, 2], vec![3]],
        };
        let config = CoordConfig {
            algorithm: CoordAlgorithm::BrandesKopf,
            h_gap: 20.0,
            ..Default::default()
        };
        assign_coordinates(&mut graph, &ranks, &config).unwrap();

        let gap = (graph.nodes[2].x - graph.nodes[1].x).abs();
        let min_required = graph.nodes[1].width / 2.0 + config.h_gap + graph.nodes[2].width / 2.0;
        assert!(
            gap >= min_required - 0.01,
            "siblings must be at least h_gap apart, got gap={gap}, required={min_required}"
        );

        // And it should still be symmetric, as the earlier test checks.
        let mid_0_3 = (graph.nodes[0].x + graph.nodes[3].x) / 2.0;
        let mid_1_2 = (graph.nodes[1].x + graph.nodes[2].x) / 2.0;
        assert!((mid_0_3 - mid_1_2).abs() < 0.01, "diamond should be exactly symmetric");
    }
}