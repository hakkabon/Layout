//! Coordinate Assignment and Edge Routing
//!
//! Assigns x/y coordinates to nodes and computes edge routes.
//! Supports two algorithms for x-coordinate assignment:
//! - Weighted median relaxation (simpler, faster)
//! - Brandes-Köpf alignment (produces more balanced layouts)

use crate::types::{LayoutGraph, RankSystem, NodeId, EdgeChain, EdgeRoute, RoutingStyle, LayoutError, LayoutDirection};

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
    /// Layout direction (TopToBottom or LeftToRight).
    pub direction: LayoutDirection,
}

impl Default for CoordConfig {
    fn default() -> Self {
        Self {
            h_gap: 20.0,
            v_gap: 40.0,
            relax_passes: 4,
            algorithm: CoordAlgorithm::default(),
            direction: LayoutDirection::default(),
        }
    }
}

/// Algorithm selection for x-coordinate assignment.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
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
/// 1. Primary axis (Y for TopToBottom, X for LeftToRight): assigned by rank
/// 2. Secondary axis (X for TopToBottom, Y for LeftToRight): based on CoordAlgorithm
/// 3. Centering: shift all coordinates so bounding box is centered at origin
///
/// # Errors
/// Returns `LayoutError::DanglingEdge` if an edge references a node id
/// outside `graph.nodes`, or `LayoutError::MissingRank` if `assign_ranks`
/// hasn't been run.
pub fn assign_coordinates(graph: &mut LayoutGraph, ranks: &RankSystem, config: &CoordConfig) -> Result<(), LayoutError> {
    let layer_count = ranks.layers.len();

    let is_lr = config.direction == LayoutDirection::LeftToRight;

    // Stage 1: Inter-layer coordinate assignment
    // Compute max extent per layer along the rank direction
    let mut layer_extents: Vec<f32> = vec![0.0; layer_count];
    for (layer_idx, layer) in ranks.layers.iter().enumerate() {
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                let extent = if is_lr { graph.nodes[node_id].width } else { graph.nodes[node_id].height };
                if extent > layer_extents[layer_idx] {
                    layer_extents[layer_idx] = extent;
                }
            }
        }
    }

    // Assign layer coordinates (accumulated along primary axis)
    let mut rank_coords: Vec<f32> = vec![0.0; graph.nodes.len()];
    let mut rank_accum: f32 = 0.0;
    for (layer_idx, layer) in ranks.layers.iter().enumerate() {
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                rank_coords[node_id] = rank_accum;
            }
        }
        rank_accum += layer_extents[layer_idx] + config.v_gap;
    }

    // Stage 2: Intra-layer coordinate assignment based on selected algorithm
    let intra_coords = match config.algorithm {
        CoordAlgorithm::MedianRelax => {
            median_relax_x_coords(graph, ranks, config.h_gap, config.relax_passes, is_lr)?
        }
        CoordAlgorithm::BrandesKopf => {
            brandes_kopf_x_coords(graph, ranks, config.h_gap, is_lr)?
        }
    };

    // Stage 3: Map canonical coordinates to X/Y and center the layout
    for (i, node) in graph.nodes.iter_mut().enumerate() {
        if is_lr {
            node.x = rank_coords[i];
            node.y = intra_coords[i];
        } else {
            node.x = intra_coords[i];
            node.y = rank_coords[i];
        }
    }

    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for node in graph.nodes.iter() {
        let half_w = node.width / 2.0;
        let half_h = node.height / 2.0;
        if node.x - half_w < min_x { min_x = node.x - half_w; }
        if node.x + half_w > max_x { max_x = node.x + half_w; }
        if node.y - half_h < min_y { min_y = node.y - half_h; }
        if node.y + half_h > max_y { max_y = node.y + half_h; }
    }

    if min_x.is_finite() && max_x.is_finite() {
        let center_x = (min_x + max_x) / 2.0;
        let center_y = (min_y + max_y) / 2.0;

        for node in graph.nodes.iter_mut() {
            node.x -= center_x;
            node.y -= center_y;
        }
    }

    Ok(())
}

/// X-coordinate assignment using weighted median relaxation.
fn median_relax_x_coords(
    graph: &mut LayoutGraph,
    ranks: &RankSystem,
    h_gap: f32,
    relax_passes: usize,
    is_lr: bool,
) -> Result<Vec<f32>, LayoutError> {
    let n = graph.nodes.len();
    let mut x_coords: Vec<f32> = vec![0.0; n];
    let mut extents: Vec<f32> = vec![0.0; n];

    // Initial placement based on order
    for layer in &ranks.layers {
        if layer.is_empty() {
            continue;
        }
        let mut max_extent: f32 = 0.0;
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                let extent = if is_lr { graph.nodes[node_id].height } else { graph.nodes[node_id].width };
                if extent > max_extent {
                    max_extent = extent;
                }
                extents[node_id] = extent;
            }
        }

        // Initial intra-layer placement
        let mut x: f32 = 0.0;
        for &node_id in layer {
            if node_id < graph.nodes.len() {
                x_coords[node_id] = x;
                x += max_extent + h_gap;
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
                        let ord_a = graph.nodes[a.0].order.unwrap_or(0);
                        let ord_b = graph.nodes[b.0].order.unwrap_or(0);
                        ord_a.cmp(&ord_b)
                    })
            });

            // Compact: place nodes respecting widths/extents and gaps
            let mut x: f32 = 0.0;
            let mut prev_extent: f32 = 0.0;
            for (i, &(node_id, _)) in sorted.iter().enumerate() {
                if i == 0 {
                    x = preferred_x[node_id].unwrap_or(0.0);
                } else {
                    let min_x = x + prev_extent / 2.0 + extents[node_id] / 2.0 + h_gap;
                    let pref = preferred_x[node_id].unwrap_or(x_coords[node_id]);
                    x = pref.max(min_x - extents[node_id] / 2.0);
                }
                x_coords[node_id] = x;
                prev_extent = extents[node_id];
            }
        }
    }

    Ok(x_coords)
}

/// X-coordinate assignment using a Brandes-Köpf-inspired heuristic.
fn brandes_kopf_x_coords(
    graph: &mut LayoutGraph,
    ranks: &RankSystem,
    h_gap: f32,
    is_lr: bool,
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
    compute_alignment(&mut x_coords[0], graph, ranks, &up_neighbors, true, true, h_gap, is_lr);

    // Alignment 1: Top-Right (process top-to-bottom, align right)
    compute_alignment(&mut x_coords[1], graph, ranks, &up_neighbors, true, false, h_gap, is_lr);

    // Alignment 2: Bottom-Left (process bottom-to-top, align left)
    compute_alignment(&mut x_coords[2], graph, ranks, &down_neighbors, false, true, h_gap, is_lr);

    // Alignment 3: Bottom-Right (process bottom-to-top, align right)
    compute_alignment(&mut x_coords[3], graph, ranks, &down_neighbors, false, false, h_gap, is_lr);

    // Average the four alignments
    let mut avg_x: Vec<f32> = vec![0.0; n];
    for i in 0..n {
        avg_x[i] = (x_coords[0][i] + x_coords[1][i] + x_coords[2][i] + x_coords[3][i]) / 4.0;
    }

    let mut combined_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];
    for id in 0..n {
        combined_neighbors[id].extend(&up_neighbors[id]);
        combined_neighbors[id].extend(&down_neighbors[id]);
    }

    repair_layer_spacing(&mut avg_x, graph, ranks, h_gap, is_lr);
    for _ in 0..8 {
        let mut preferred: Vec<f32> = avg_x.clone();
        for id in 0..n {
            if !combined_neighbors[id].is_empty() {
                let sum: f32 = combined_neighbors[id].iter().map(|&nb| avg_x[nb]).sum();
                preferred[id] = sum / combined_neighbors[id].len() as f32;
            }
        }
        for id in 0..n {
            avg_x[id] = 0.5 * avg_x[id] + 0.5 * preferred[id];
        }
        repair_layer_spacing(&mut avg_x, graph, ranks, h_gap, is_lr);
    }

    Ok(avg_x)
}

fn repair_layer_spacing(x: &mut [f32], graph: &LayoutGraph, ranks: &RankSystem, h_gap: f32, is_lr: bool) {
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

        let mut offsets: Vec<f32> = Vec::with_capacity(ordered.len());
        let mut cum = 0.0f32;
        for (i, &id) in ordered.iter().enumerate() {
            let half = (if is_lr { graph.nodes[id].height } else { graph.nodes[id].width }) / 2.0;
            if i == 0 {
                offsets.push(0.0);
            } else {
                let prev_half = (if is_lr { graph.nodes[ordered[i - 1]].height } else { graph.nodes[ordered[i - 1]].width }) / 2.0;
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

fn isotonic_regression(values: &[f32]) -> Vec<f32> {
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

fn compute_alignment(
    x_out: &mut [f32],
    graph: &LayoutGraph,
    ranks: &RankSystem,
    neighbors: &[Vec<NodeId>],
    top_down: bool,
    align_left: bool,
    h_gap: f32,
    is_lr: bool,
) {
    let layer_indices: Vec<usize> = if top_down {
        (0..ranks.layers.len()).collect()
    } else {
        (0..ranks.layers.len()).rev().collect()
    };

    for &layer_idx in &layer_indices {
        let layer = &ranks.layers[layer_idx];
        if layer.is_empty() {
            continue;
        }

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

        let mut ordered_nodes: Vec<NodeId> = layer.clone();
        ordered_nodes.sort_by_key(|&id| graph.nodes[id].order.unwrap_or(0));

        if align_left {
            let mut prev_right: f32 = 0.0;
            for (i, &node_id) in ordered_nodes.iter().enumerate() {
                if node_id >= graph.nodes.len() {
                    continue;
                }
                let node = &graph.nodes[node_id];
                let extent = if is_lr { node.height } else { node.width };
                let half_w = extent / 2.0;

                let x = if i == 0 {
                    if has_preferred[node_id] {
                        preferred_x[node_id] - half_w
                    } else {
                        0.0
                    }
                } else {
                    let min_x = prev_right + h_gap;
                    if has_preferred[node_id] {
                        (preferred_x[node_id] - half_w).max(min_x)
                    } else {
                        min_x
                    }
                };

                x_out[node_id] = x + half_w;
                prev_right = x + extent;
            }
        } else {
            let mut prev_left: f32 = 0.0;
            for (i, &node_id) in ordered_nodes.iter().rev().enumerate() {
                if node_id >= graph.nodes.len() {
                    continue;
                }
                let node = &graph.nodes[node_id];
                let extent = if is_lr { node.height } else { node.width };
                let half_w = extent / 2.0;

                let x = if i == 0 {
                    if has_preferred[node_id] {
                        preferred_x[node_id] - half_w
                    } else {
                        0.0
                    }
                } else {
                    let max_x = prev_left - h_gap - extent;
                    if has_preferred[node_id] {
                        (preferred_x[node_id] - half_w).min(max_x)
                    } else {
                        max_x
                    }
                };

                x_out[node_id] = x + half_w;
                prev_left = x;
            }
        }
    }
}

/// Routes edges through the graph based on node positions and edge chains.
pub fn route_edges(graph: &LayoutGraph, chains: &[EdgeChain], style: RoutingStyle) -> Result<Vec<EdgeRoute>, LayoutError> {
    use std::collections::HashMap;

    let mut routes = Vec::new();

    // Track edge multiplicity between node pairs to apply curvature offsets
    let mut pair_counts: HashMap<(NodeId, NodeId), usize> = HashMap::new();
    let mut pair_seen: HashMap<(NodeId, NodeId), usize> = HashMap::new();

    for chain in chains {
        let p = (chain.source.min(chain.target), chain.source.max(chain.target));
        *pair_counts.entry(p).or_insert(0) += 1;
    }

    for chain in chains {
        let p = (chain.source.min(chain.target), chain.source.max(chain.target));
        let count = *pair_counts.get(&p).unwrap_or(&1);
        let idx = *pair_seen.entry(p).or_insert(0);
        pair_seen.insert(p, idx + 1);

        let offset = if count > 1 {
            (idx as f32 - (count - 1) as f32 / 2.0) * 20.0
        } else {
            0.0
        };

        if chain.is_self_loop || chain.source == chain.target {
            let node = graph.nodes.get(chain.source)
                .ok_or(LayoutError::DanglingEdge { from: chain.source, to: chain.target })?;
            let cx = node.x;
            let cy = node.y;
            let w = node.width.max(16.0);
            let h = node.height.max(16.0);
            let r_offset = offset.abs() * 0.6;

            let p0 = (cx + w * 0.2, cy - h / 2.0);
            let c1 = (cx + w * 0.6 + 15.0 + r_offset, cy - h / 2.0 - 25.0 - r_offset);
            let c2 = (cx - w * 0.6 - 15.0 - r_offset, cy - h / 2.0 - 25.0 - r_offset);
            let p1 = (cx - w * 0.2, cy - h / 2.0);

            let loop_waypoints = match style {
                RoutingStyle::Straight => vec![p0, ((p0.0 + p1.0) / 2.0, cy - h / 2.0 - 20.0 - r_offset), p1],
                RoutingStyle::Orthogonal => vec![
                    p0,
                    (p0.0, cy - h / 2.0 - 15.0 - r_offset),
                    (p1.0, cy - h / 2.0 - 15.0 - r_offset),
                    p1,
                ],
                RoutingStyle::Bezier => vec![p0, c1, c2, p1],
            };

            routes.push(EdgeRoute {
                source: chain.source,
                target: chain.target,
                reversed: chain.reversed,
                is_self_loop: true,
                waypoints: loop_waypoints,
            });
            continue;
        }

        // Extract waypoint coordinates from the chain
        let mut waypoints: Vec<(f32, f32)> = Vec::new();

        let source_node = graph.nodes.get(chain.source)
            .ok_or(LayoutError::DanglingEdge { from: chain.source, to: chain.target })?;
        let target_node = graph.nodes.get(chain.target)
            .ok_or(LayoutError::DanglingEdge { from: chain.source, to: chain.target })?;

        let dx_total = target_node.x - source_node.x;
        let dy_total = target_node.y - source_node.y;
        let is_primarily_vertical = dy_total.abs() >= dx_total.abs();

        for (i, &node_id) in chain.chain.iter().enumerate() {
            let node = graph.nodes.get(node_id)
                .ok_or(LayoutError::DanglingEdge { from: chain.source, to: chain.target })?;
            let cx = node.x;
            let cy = node.y;

            if i == 0 {
                // Source attachment
                if is_primarily_vertical {
                    let y_att = if dy_total >= 0.0 { cy + node.height / 2.0 } else { cy - node.height / 2.0 };
                    waypoints.push((cx, y_att));
                } else {
                    let x_att = if dx_total >= 0.0 { cx + node.width / 2.0 } else { cx - node.width / 2.0 };
                    waypoints.push((x_att, cy));
                }
            } else if i == chain.chain.len() - 1 {
                // Target attachment
                if is_primarily_vertical {
                    let y_att = if dy_total >= 0.0 { cy - node.height / 2.0 } else { cy + node.height / 2.0 };
                    waypoints.push((cx, y_att));
                } else {
                    let x_att = if dx_total >= 0.0 { cx - node.width / 2.0 } else { cx + node.width / 2.0 };
                    waypoints.push((x_att, cy));
                }
            } else {
                // Dummy node
                waypoints.push((cx, cy));
            }
        }

        if waypoints.len() < 2 {
            continue;
        }

        let route = match style {
            RoutingStyle::Straight => {
                let first = *waypoints.first().unwrap();
                let last = *waypoints.last().unwrap();
                let straight_points = if offset.abs() > 0.001 {
                    let mid_x = (first.0 + last.0) / 2.0;
                    let mid_y = (first.1 + last.1) / 2.0;
                    let dx = last.0 - first.0;
                    let dy = last.1 - first.1;
                    let len = (dx * dx + dy * dy).sqrt().max(1.0);
                    let nx = -dy / len;
                    let ny = dx / len;
                    vec![first, (mid_x + nx * offset, mid_y + ny * offset), last]
                } else {
                    vec![first, last]
                };

                EdgeRoute {
                    source: chain.source,
                    target: chain.target,
                    reversed: chain.reversed,
                    is_self_loop: false,
                    waypoints: straight_points,
                }
            }
            RoutingStyle::Orthogonal => {
                let mut ortho_points: Vec<(f32, f32)> = Vec::new();
                ortho_points.push(waypoints[0]);
                for i in 0..waypoints.len() - 1 {
                    let (x1, y1) = waypoints[i];
                    let (x2, y2) = waypoints[i + 1];
                    let mid_y = (y1 + y2) / 2.0 + offset * 0.3;
                    ortho_points.push((x1, mid_y));
                    ortho_points.push((x2, mid_y));
                }
                ortho_points.push(*waypoints.last().unwrap());

                EdgeRoute {
                    source: chain.source,
                    target: chain.target,
                    reversed: chain.reversed,
                    is_self_loop: false,
                    waypoints: ortho_points,
                }
            }
            RoutingStyle::Bezier => {
                let mut bezier_points: Vec<(f32, f32)> = Vec::new();

                if waypoints.len() == 2 {
                    let p0 = waypoints[0];
                    let p1 = waypoints[1];
                    let dx = p1.0 - p0.0;
                    let dy = p1.1 - p0.1;
                    let len = (dx * dx + dy * dy).sqrt().max(1.0);
                    let nx = -dy / len;
                    let ny = dx / len;

                    let (c1, c2) = if is_primarily_vertical {
                        (
                            (p0.0 + nx * offset, p0.1 + dy * 0.5 + ny * offset),
                            (p1.0 + nx * offset, p1.1 - dy * 0.5 + ny * offset),
                        )
                    } else {
                        (
                            (p0.0 + dx * 0.5 + nx * offset, p0.1 + ny * offset),
                            (p1.0 - dx * 0.5 + nx * offset, p1.1 + ny * offset),
                        )
                    };

                    bezier_points = vec![p0, c1, c2, p1];
                } else {
                    for i in 0..waypoints.len() - 1 {
                        let p0 = waypoints[i];
                        let p1 = waypoints[i + 1];

                        let p_prev = if i > 0 { waypoints[i - 1] } else {
                            (2.0 * p0.0 - p1.0, 2.0 * p0.1 - p1.1)
                        };
                        let p_next = if i + 2 < waypoints.len() { waypoints[i + 2] } else {
                            (2.0 * p1.0 - p0.0, 2.0 * p1.1 - p0.1)
                        };

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
                    is_self_loop: false,
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
            is_self_loop: false,
            chain: vec![0, 1],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Straight).unwrap();
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].waypoints.len(), 2);
    }

    #[test]
    fn route_edges_waypoints_attach_at_node_edges_not_center() {
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
            is_self_loop: false,
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
            is_self_loop: false,
            chain: vec![0, 1, 2],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Straight).unwrap();
        let routes_ortho = route_edges(&graph, &chains, RoutingStyle::Orthogonal).unwrap();
        assert_eq!(routes.len(), 1);
        assert_eq!(routes_ortho.len(), 1);
        // Orthogonal routing starts at source node attachment edge (-80.0)
        assert_eq!(routes_ortho[0].waypoints.first().unwrap().1, -80.0);
        // Second point is the L-bend midpoint
        assert_eq!(routes_ortho[0].waypoints[1].1, -40.0);
    }

    #[test]
    fn route_edges_bezier_produces_four_plus_points() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2)],
            edges: vec![],
        };
        let chains = vec![EdgeChain {
            source: 0,
            target: 2,
            reversed: false,
            is_self_loop: false,
            chain: vec![0, 1, 2],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Bezier).unwrap();
        assert_eq!(routes.len(), 1);
        assert!(routes[0].waypoints.len() >= 4, "Bezier should produce at least 4 waypoints");
    }

    #[test]
    fn route_edges_self_loops_generate_valid_loop_routes() {
        let graph = LayoutGraph {
            nodes: vec![node(0)],
            edges: vec![],
        };
        let chains = vec![EdgeChain {
            source: 0,
            target: 0,
            reversed: false,
            is_self_loop: true,
            chain: vec![0],
        }];

        let routes = route_edges(&graph, &chains, RoutingStyle::Bezier).unwrap();
        assert_eq!(routes.len(), 1);
        assert!(routes[0].is_self_loop);
        assert_eq!(routes[0].waypoints.len(), 4, "Self loop Bezier should have 4 points (P0, C1, C2, P1)");
    }

    #[test]
    fn route_edges_multi_edges_have_distinct_waypoints() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![],
        };
        let chains = vec![
            EdgeChain {
                source: 0,
                target: 1,
                reversed: false,
                is_self_loop: false,
                chain: vec![0, 1],
            },
            EdgeChain {
                source: 0,
                target: 1,
                reversed: false,
                is_self_loop: false,
                chain: vec![0, 1],
            },
        ];

        let routes = route_edges(&graph, &chains, RoutingStyle::Bezier).unwrap();
        assert_eq!(routes.len(), 2);
        // Multi-edges should not have identical control points
        assert_ne!(routes[0].waypoints[1], routes[1].waypoints[1]);
    }

    #[test]
    fn left_to_right_layout_coordinates() {
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
            direction: LayoutDirection::LeftToRight,
            ..Default::default()
        };
        assign_coordinates(&mut graph, &ranks, &config).unwrap();

        // In LeftToRight, rank 0 is to the left of rank 1 (x0 < x1)
        assert!(graph.nodes[0].x < graph.nodes[1].x);
    }

    #[test]
    fn brandes_kopf_no_sibling_overlap() {
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

        let mid_0_3 = (graph.nodes[0].x + graph.nodes[3].x) / 2.0;
        let mid_1_2 = (graph.nodes[1].x + graph.nodes[2].x) / 2.0;
        assert!((mid_0_3 - mid_1_2).abs() < 0.01, "diamond should be exactly symmetric");
    }
}