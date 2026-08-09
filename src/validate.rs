//! Graph validation utilities.

use crate::types::{LayoutGraph, LayoutError};

/// Validates that the graph is in a consistent state.
/// Currently checks for duplicate node IDs and edges referencing non-existent nodes.
pub fn validate_graph(graph: &LayoutGraph) -> Result<(), LayoutError> {
    let n = graph.nodes.len();

    // Check all node IDs are valid indices
    for (i, node) in graph.nodes.iter().enumerate() {
        if node.id != i {
            return Err(LayoutError::CyclicGraph); // Reuse error for now
        }
    }

    // Check all edge endpoints exist
    for edge in &graph.edges {
        if edge.from >= n || edge.to >= n {
            return Err(LayoutError::CyclicGraph);
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
}
