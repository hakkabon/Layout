//! Cycle Breaking
//!
//! This phase converts a potentially cyclic graph into a DAG by reversing
//! back-edges detected via DFS. The reversed edges are marked so they can
//! be drawn with their arrowheads at the visually higher end.

use crate::types::LayoutGraph;

/// Breaks cycles in the graph by reversing back-edges detected via DFS.
/// Returns the indices of edges that were reversed.
///
/// Uses an iterative 3-color DFS to avoid stack overflow and correctly
/// identify back-edges without duplicate expansions.
///
/// # Complexity
/// O(N + E) where N is the number of nodes and E is the number of edges.
pub fn break_cycles(graph: &mut LayoutGraph) -> Vec<usize> {
    let n = graph.nodes.len();
    if n == 0 {
        return Vec::new();
    }

    // 0 = White (unvisited), 1 = Gray (on stack / visiting), 2 = Black (visited)
    let mut color = vec![0u8; n];
    let mut reversed_indices = Vec::new();

    // Build adjacency list: node -> Vec<(neighbor, edge_index)>
    let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
    for (idx, edge) in graph.edges.iter().enumerate() {
        if edge.from < n && edge.to < n {
            adj[edge.from].push((edge.to, idx));
        }
    }

    for start in 0..n {
        if color[start] != 0 {
            continue;
        }

        // Stack contains (node, next_edge_index)
        let mut stack: Vec<(usize, usize)> = Vec::new();
        color[start] = 1;
        stack.push((start, 0));

        while let Some((node, edge_idx)) = stack.last_mut() {
            let u = *node;
            if *edge_idx < adj[u].len() {
                let (v, e_idx) = adj[u][*edge_idx];
                *edge_idx += 1;

                if color[v] == 0 {
                    color[v] = 1;
                    stack.push((v, 0));
                } else if color[v] == 1 {
                    // Back-edge detected - reverse it
                    let edge = &mut graph.edges[e_idx];
                    std::mem::swap(&mut edge.from, &mut edge.to);
                    edge.reversed = true;
                    reversed_indices.push(e_idx);
                }
            } else {
                color[u] = 2;
                stack.pop();
            }
        }
    }

    reversed_indices
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LayoutNode, NodeType, LayoutEdge};

    fn node(id: usize) -> LayoutNode {
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
    fn acyclic_graph_unchanged() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 2, reversed: false, label_size: None },
            ],
        };
        let reversed = break_cycles(&mut graph);
        assert!(reversed.is_empty());
        assert_eq!(graph.edges[0].from, 0);
        assert_eq!(graph.edges[0].to, 1);
        assert!(!graph.edges[0].reversed);
    }

    #[test]
    fn simple_cycle_is_broken() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 2, reversed: false, label_size: None },
                LayoutEdge { from: 2, to: 0, reversed: false, label_size: None }, // closes cycle
            ],
        };
        let reversed = break_cycles(&mut graph);
        assert_eq!(reversed.len(), 1);
        // One edge should be reversed
        let reversed_count = graph.edges.iter().filter(|e| e.reversed).count();
        assert_eq!(reversed_count, 1);
    }

    #[test]
    fn diamond_graph_unchanged() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false, label_size: None },
                LayoutEdge { from: 0, to: 2, reversed: false, label_size: None },
                LayoutEdge { from: 1, to: 3, reversed: false, label_size: None },
                LayoutEdge { from: 2, to: 3, reversed: false, label_size: None },
            ],
        };
        let reversed = break_cycles(&mut graph);
        assert!(reversed.is_empty());
    }
}
