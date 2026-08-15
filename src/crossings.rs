//! Layer Ordering (Crossing Reduction)
//!
//! Implements the barycenter heuristic with alternating sweeps and
//! transpose cleanup to minimize edge crossings between adjacent layers.

use crate::types::{LayoutGraph, RankSystem, NodeId, RankId, LayoutError};

/// Upper bound on transpose sweeps per call to `order_layers`. Each full
/// sweep strictly decreases the crossing count when it makes any swap, so
/// the loop is guaranteed to terminate on its own — but on a large or
/// pathological graph that could still mean an unbounded number of O(N)
/// sweeps. This cap trades a small amount of layout quality on worst-case
/// inputs for a predictable upper bound on work done.
const MAX_TRANSPOSE_PASSES: usize = 100;

/// Orders nodes within each layer to minimize edge crossings.
///
/// Uses the barycenter heuristic with alternating downward and upward sweeps,
/// followed by a transpose pass to clean up remaining adjacent swaps.
///
/// # Preconditions
/// - Every edge must span exactly one rank (run `insert_dummy_nodes` first).
///
/// # Errors
/// Returns `LayoutError::DanglingEdge` if an edge references a node id
/// outside `graph.nodes`, or `LayoutError::MissingRank` if `assign_ranks`
/// hasn't been run.
///
/// # Arguments
/// * `graph` - The layout graph with single-rank edges
/// * `ranks` - The rank system defining layers
/// * `sweeps` - Number of barycenter sweep iterations
pub fn order_layers(graph: &mut LayoutGraph, ranks: &mut RankSystem, sweeps: usize) -> Result<(), LayoutError> {
    let n = graph.nodes.len();
    let mut up_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];
    let mut down_neighbors: Vec<Vec<NodeId>> = vec![Vec::new(); n];

    // Build adjacency lists for single-rank edges
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
        } else if rank_from - rank_to == 1 {
            down_neighbors[edge.to].push(edge.from);
            up_neighbors[edge.from].push(edge.to);
        }
    }

    let layer_count = ranks.layers.len();

    for sweep in 0..sweeps {
        if sweep % 2 == 0 {
            // Downward sweep: order layer i by its already-fixed neighbors in layer i-1
            for layer_idx in 1..layer_count {
                reorder_layer_by_barycenter(graph, ranks, layer_idx, &up_neighbors);
            }
        } else {
            // Upward sweep: mirror image, using layer i+1 as the anchor
            for layer_idx in (0..layer_count.saturating_sub(1)).rev() {
                reorder_layer_by_barycenter(graph, ranks, layer_idx, &down_neighbors);
            }
        }

        // Transpose pass to clean up adjacent swaps
        transpose_pass(graph, ranks, &up_neighbors, &down_neighbors);
    }

    Ok(())
}

fn reorder_layer_by_barycenter(
    graph: &mut LayoutGraph,
    ranks: &mut RankSystem,
    layer_idx: RankId,
    neighbors: &[Vec<NodeId>],
) {
    let layer = &ranks.layers[layer_idx];

    let mut scored: Vec<(NodeId, f32)> = layer
        .iter()
        .map(|&node_id| {
            let adj = &neighbors[node_id];
            let score = if adj.is_empty() {
                // No anchor in the adjacent layer: keep current position
                graph.nodes[node_id].order.unwrap_or(0) as f32
            } else {
                let sum: usize = adj
                    .iter()
                    .map(|&nb| graph.nodes[nb].order.unwrap_or(0))
                    .sum();
                sum as f32 / adj.len() as f32
            };
            (node_id, score)
        })
        .collect();

    // Stable sort so ties don't get shuffled non-deterministically
    scored.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    let new_layer: Vec<NodeId> = scored.into_iter().map(|(id, _)| id).collect();
    for (pos, &node_id) in new_layer.iter().enumerate() {
        graph.nodes[node_id].order = Some(pos);
    }
    ranks.layers[layer_idx] = new_layer;
}

/// Repeatedly swaps adjacent node pairs within a layer whenever doing so
/// strictly reduces crossings toward both neighboring layers.
fn transpose_pass(
    graph: &mut LayoutGraph,
    ranks: &mut RankSystem,
    up_neighbors: &[Vec<NodeId>],
    down_neighbors: &[Vec<NodeId>],
) {
    let mut improved = true;
    let mut pass = 0;
    while improved && pass < MAX_TRANSPOSE_PASSES {
        improved = false;
        pass += 1;
        for layer_idx in 0..ranks.layers.len() {
            let len = ranks.layers[layer_idx].len();
            for i in 0..len.saturating_sub(1) {
                let a = ranks.layers[layer_idx][i];
                let b = ranks.layers[layer_idx][i + 1];

                let before = local_crossings(graph, a, b, up_neighbors)
                    + local_crossings(graph, a, b, down_neighbors);
                let after = local_crossings(graph, b, a, up_neighbors)
                    + local_crossings(graph, b, a, down_neighbors);

                if after < before {
                    ranks.layers[layer_idx].swap(i, i + 1);
                    graph.nodes[a].order = Some(i + 1);
                    graph.nodes[b].order = Some(i);
                    improved = true;
                }
            }
        }
    }
}

/// Counts crossings between two adjacent nodes' edges toward a shared layer.
fn local_crossings(
    graph: &LayoutGraph,
    left: NodeId,
    right: NodeId,
    neighbors: &[Vec<NodeId>],
) -> usize {
    let mut crossings = 0;
    for &l_nb in &neighbors[left] {
        let l_order = graph.nodes[l_nb].order.unwrap_or(0);
        for &r_nb in &neighbors[right] {
            let r_order = graph.nodes[r_nb].order.unwrap_or(0);
            if l_order > r_order {
                crossings += 1;
            }
        }
    }
    crossings
}

/// Counts total crossings across all adjacent layer pairs.
/// Useful as a test oracle or debugging aid.
pub fn count_total_crossings(graph: &LayoutGraph, ranks: &RankSystem) -> usize {
    let mut total = 0;
    for layer_idx in 0..ranks.layers.len().saturating_sub(1) {
        let upper = &ranks.layers[layer_idx];
        let lower = &ranks.layers[layer_idx + 1];

        let mut segments: Vec<(usize, usize)> = Vec::new();
        for edge in &graph.edges {
            let from_rank = graph.nodes[edge.from].rank.unwrap();
            if from_rank == layer_idx {
                let u_pos = upper.iter().position(|&n| n == edge.from).unwrap();
                let l_pos = lower.iter().position(|&n| n == edge.to).unwrap();
                segments.push((u_pos, l_pos));
            }
        }

        for i in 0..segments.len() {
            for j in (i + 1)..segments.len() {
                let (u1, l1) = segments[i];
                let (u2, l2) = segments[j];
                if (u1 < u2 && l1 > l2) || (u1 > u2 && l1 < l2) {
                    total += 1;
                }
            }
        }
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LayoutNode, NodeType, LayoutEdge};

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
    fn barycenter_sweeps_reduce_crossings() {
        // Two ranks, deliberately crossed: 0-3, 1-2 wired as an X
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 3, reversed: false },
                LayoutEdge { from: 1, to: 2, reversed: false },
            ],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(0);
        graph.nodes[2].rank = Some(1);
        graph.nodes[3].rank = Some(1);

        let mut ranks = RankSystem {
            layers: vec![vec![0, 1], vec![2, 3]],
        };
        for layer in &ranks.layers {
            for (pos, &id) in layer.iter().enumerate() {
                graph.nodes[id].order = Some(pos);
            }
        }

        let before = count_total_crossings(&graph, &ranks);
        order_layers(&mut graph, &mut ranks, 4).unwrap();
        let after = count_total_crossings(&graph, &ranks);

        assert!(before >= 1, "test setup should start with a crossing");
        assert_eq!(after, 0, "transpose pass should fully untangle a simple X");
    }

    #[test]
    fn no_crossings_unchanged() {
        // Parallel edges with no crossings
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 2, reversed: false },
                LayoutEdge { from: 1, to: 3, reversed: false },
            ],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(0);
        graph.nodes[2].rank = Some(1);
        graph.nodes[3].rank = Some(1);

        let mut ranks = RankSystem {
            layers: vec![vec![0, 1], vec![2, 3]],
        };
        for layer in &ranks.layers {
            for (pos, &id) in layer.iter().enumerate() {
                graph.nodes[id].order = Some(pos);
            }
        }

        let before = count_total_crossings(&graph, &ranks);
        order_layers(&mut graph, &mut ranks, 4).unwrap();
        let after = count_total_crossings(&graph, &ranks);

        assert_eq!(before, 0);
        assert_eq!(after, 0);
    }

    #[test]
    fn single_node_per_layer_unchanged() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);

        let mut ranks = RankSystem {
            layers: vec![vec![0], vec![1]],
        };
        for layer in &ranks.layers {
            for (pos, &id) in layer.iter().enumerate() {
                graph.nodes[id].order = Some(pos);
            }
        }

        order_layers(&mut graph, &mut ranks, 4).unwrap();
        assert_eq!(count_total_crossings(&graph, &ranks), 0);
    }
}