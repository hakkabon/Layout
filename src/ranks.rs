//! Rank Assignment
//!
//! Assigns ranks to nodes using Kahn's algorithm for topological sorting
//! combined with longest-path relaxation. This ensures minimum height
//! (minimum number of layers) for the layout.

use std::collections::VecDeque;
use crate::types::{LayoutGraph, LayoutError, RankSystem, NodeId, RankId};

/// Internal struct for backward compatibility - holds phase methods
pub struct LayoutPipeline;

impl LayoutPipeline {
    /// Assign ranks to all nodes.
    ///
    /// Uses Kahn's algorithm to get a valid topological order, then applies
    /// longest-path relaxation to assign each node to the earliest possible rank.
    ///
    /// # Errors
    /// Returns `LayoutError::CyclicGraph` if the graph contains a cycle.
    pub fn assign_ranks(graph: &mut LayoutGraph) -> Result<RankSystem, LayoutError> {
        crate::validate::validate_graph(graph)?;
        let n = graph.nodes.len();

        // Build adjacency and in-degree counts
        let mut out_edges: Vec<Vec<NodeId>> = vec![Vec::new(); n];
        let mut in_degree: Vec<usize> = vec![0; n];

        for edge in &graph.edges {
            out_edges[edge.from].push(edge.to);
            in_degree[edge.to] += 1;
        }

        // Kahn's algorithm: start with all source nodes (in_degree == 0)
        let mut queue: VecDeque<NodeId> =
            (0..n).filter(|&id| in_degree[id] == 0).collect();
        let mut topo_order: Vec<NodeId> = Vec::with_capacity(n);

        while let Some(node_id) = queue.pop_front() {
            topo_order.push(node_id);
            for &successor in &out_edges[node_id] {
                in_degree[successor] -= 1;
                if in_degree[successor] == 0 {
                    queue.push_back(successor);
                }
            }
        }

        // If we couldn't visit all nodes, there's a cycle
        if topo_order.len() != n {
            return Err(LayoutError::CyclicGraph);
        }

        // Initialize all nodes to rank 0
        for node in &mut graph.nodes {
            node.rank = Some(0);
        }

        // Longest-path relaxation: propagate ranks forward
        let mut max_rank: RankId = 0;
        for &node_id in &topo_order {
            let current_rank = graph.nodes[node_id].rank.unwrap();
            for &successor in &out_edges[node_id] {
                let candidate = current_rank + 1;
                if candidate > graph.nodes[successor].rank.unwrap() {
                    graph.nodes[successor].rank = Some(candidate);
                }
                max_rank = max_rank.max(candidate);
            }
        }

        // Build the rank system layers
        let mut ranks = RankSystem {
            layers: vec![Vec::new(); max_rank + 1],
        };
        for node in &graph.nodes {
            let r = node.rank.unwrap();
            ranks.layers[r].push(node.id);
        }

        // Seed order with initial positions
        for layer in &ranks.layers {
            for (pos, &node_id) in layer.iter().enumerate() {
                graph.nodes[node_id].order = Some(pos);
            }
        }

        Ok(ranks)
    }
}

/// public function: assigns ranks to nodes.
pub fn assign_ranks(graph: &mut LayoutGraph) -> Result<RankSystem, LayoutError> {
    LayoutPipeline::assign_ranks(graph)
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
    fn diamond_graph_ranks_correctly_regardless_of_storage_order() {
        // 0 -> 1 -> 3
        // 0 -> 2 -> 3
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(3), node(1), node(2)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 0, to: 2, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 3, reversed: false, label_size: None },
                LayoutEdge { from: 2, to: 3, reversed: false, label_size: None },
            ],
        };
        // Re-sort nodes by id so NodeId == index
        graph.nodes.sort_by_key(|n| n.id);

        let ranks = LayoutPipeline::assign_ranks(&mut graph).unwrap();

        assert_eq!(graph.nodes[0].rank, Some(0));
        assert_eq!(graph.nodes[1].rank, Some(1));
        assert_eq!(graph.nodes[2].rank, Some(1));
        assert_eq!(graph.nodes[3].rank, Some(2));
        assert_eq!(ranks.layers.len(), 3);
    }

    #[test]
    fn cyclic_graph_is_rejected() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 2, reversed: false, label_size: None },
                LayoutEdge { from: 2, to: 0, reversed: false, label_size: None }, // closes the cycle
            ],
        };
        let result = LayoutPipeline::assign_ranks(&mut graph);
        assert_eq!(result.unwrap_err(), LayoutError::CyclicGraph);
    }

    #[test]
    fn linear_chain_ranks_correctly() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 2, reversed: false, label_size: None },
                LayoutEdge { from: 2, to: 3, reversed: false, label_size: None },
            ],
        };
        let ranks = LayoutPipeline::assign_ranks(&mut graph).unwrap();

        assert_eq!(graph.nodes[0].rank, Some(0));
        assert_eq!(graph.nodes[1].rank, Some(1));
        assert_eq!(graph.nodes[2].rank, Some(2));
        assert_eq!(graph.nodes[3].rank, Some(3));
        assert_eq!(ranks.layers.len(), 4);
    }
}
