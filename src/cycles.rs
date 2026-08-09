//! Cycle Breaking
//!
//! This phase converts a potentially cyclic graph into a DAG by reversing
//! back-edges detected via DFS. The reversed edges are marked so they can
//! be drawn with their arrowheads at the visually higher end.

use crate::types::LayoutGraph;

/// Breaks cycles in the graph by reversing back-edges detected via DFS.
/// Returns the indices of edges that were reversed.
///
/// Uses an iterative (stack-based) DFS to avoid stack overflow on deep graphs.
///
/// # Complexity
/// O(N + E) where N is the number of nodes and E is the number of edges.
pub fn break_cycles(graph: &mut LayoutGraph) -> Vec<usize> {
    let n = graph.nodes.len();
    let mut visited = vec![false; n];
    let mut on_stack = vec![false; n];
    let mut reversed_indices = Vec::new();

    // Build adjacency list for efficient traversal
    let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n]; // (neighbor, edge_index)
    for (idx, edge) in graph.edges.iter().enumerate() {
        adj[edge.from].push((edge.to, idx));
    }

    for start in 0..n {
        if visited[start] {
            continue;
        }

        // Iterative DFS using explicit stack
        // Stack entries: (node_id, edge_iterator_index, entering)
        let mut stack: Vec<(usize, usize, bool)> = Vec::new();
        stack.push((start, 0, true));

        while let Some((node, _edge_idx, entering)) = stack.pop() {
            if entering {
                if visited[node] && !on_stack[node] {
                    continue;
                }
                visited[node] = true;
                on_stack[node] = true;
                // Push exit marker
                stack.push((node, 0, false));
                // Push all children
                for &(neighbor, e_idx) in &adj[node] {
                    if !visited[neighbor] {
                        stack.push((neighbor, 0, true));
                    } else if on_stack[neighbor] {
                        // Back-edge detected - reverse it
                        let edge = &mut graph.edges[e_idx];
                        std::mem::swap(&mut edge.from, &mut edge.to);
                        edge.reversed = true;
                        reversed_indices.push(e_idx);
                    }
                }
            } else {
                on_stack[node] = false;
            }
        }
    }

    reversed_indices
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LayoutNode, NodeType};

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
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 1, to: 2, reversed: false },
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
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 1, to: 2, reversed: false },
                LayoutEdge { from: 2, to: 0, reversed: false }, // closes cycle
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
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 0, to: 2, reversed: false },
                LayoutEdge { from: 1, to: 3, reversed: false },
                LayoutEdge { from: 2, to: 3, reversed: false },
            ],
        };
        let reversed = break_cycles(&mut graph);
        assert!(reversed.is_empty());
    }
}
