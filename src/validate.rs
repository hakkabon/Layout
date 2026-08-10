//! Graph validation utilities.

use crate::types::{LayoutGraph, LayoutError};

/// Validates that the graph is in a consistent state.
///
/// Checks, in order:
/// 1. Every node's `id` matches its position in `graph.nodes` (the dense
///    index invariant every phase in this crate relies on).
/// 2. Every edge's `from`/`to` reference an existing node.
/// 3. No edge is a self-loop (`from == to`) — see
///    [`LayoutGraph::extract_self_loops`](crate::LayoutGraph::extract_self_loops).
///
/// This does *not* check for cycles; that's `break_cycles`'s job, and
/// `assign_ranks` reports `LayoutError::CyclicGraph` if one survives.
pub fn validate_graph(graph: &LayoutGraph) -> Result<(), LayoutError> {
    let n = graph.nodes.len();

    // Check all node IDs are valid, dense indices.
    for (i, node) in graph.nodes.iter().enumerate() {
        if node.id != i {
            return Err(LayoutError::InvalidNodeId { index: i, id: node.id });
        }
    }

    // Check all edge endpoints exist.
    for edge in &graph.edges {
        if edge.from >= n || edge.to >= n {
            return Err(LayoutError::DanglingEdge { from: edge.from, to: edge.to });
        }
    }

    // Check for self-loops.
    for edge in &graph.edges {
        if edge.from == edge.to {
            return Err(LayoutError::SelfLoop(edge.from));
        }
    }

    Ok(())
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
    fn valid_graph_passes_validation() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        assert!(validate_graph(&graph).is_ok());
    }

    #[test]
    fn invalid_edge_endpoint_fails_validation() {
        let graph = LayoutGraph {
            nodes: vec![node(0)],
            edges: vec![LayoutEdge { from: 0, to: 5, reversed: false }],
        };
        assert!(validate_graph(&graph).is_err());
    }

    #[test]
    fn self_loop_fails_validation_with_specific_error() {
        let graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 1, to: 1, reversed: false },
            ],
        };
        match validate_graph(&graph) {
            Err(LayoutError::SelfLoop(id)) => assert_eq!(id, 1),
            other => panic!("expected LayoutError::SelfLoop(1), got {other:?}"),
        }
    }

    #[test]
    fn extract_self_loops_removes_them_and_leaves_graph_valid() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false },
                LayoutEdge { from: 1, to: 1, reversed: false },
            ],
        };
        let loops = graph.extract_self_loops();
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].from, 1);
        assert!(validate_graph(&graph).is_ok());
    }

    #[test]
    fn add_node_assigns_dense_ids() {
        let mut graph = LayoutGraph::default();
        let a = graph.add_node(node(999)); // id in the literal should be overwritten
        let b = graph.add_node(node(0));
        assert_eq!(a, 0);
        assert_eq!(b, 1);
        assert!(validate_graph(&graph).is_ok());
    }
}
