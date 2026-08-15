//! Dummy Node Insertion
//!
//! Splits long edges (edges spanning multiple ranks) into chains of
//! single-rank edges by inserting dummy nodes. This is required before
//! the crossing-reduction, which only handles adjacent-layer edges.
 
use crate::types::{LayoutGraph, LayoutNode, NodeType, LayoutEdge, EdgeChain, LayoutError};
 
/// Inserts dummy nodes for edges that span more than one rank.
///
/// Returns a vector of `EdgeChain` structures tracking how each original
/// edge was decomposed into single-rank segments.
///
/// # Errors
/// Returns `LayoutError::DanglingEdge` if an edge references a node id
/// outside `graph.nodes`, or `LayoutError::MissingRank` if `assign_ranks`
/// hasn't been run (or a node was added afterward without a rank).
///
/// # Complexity
/// O(E · S) where E is the number of edges and S is the average edge span.
pub fn insert_dummy_nodes(
    graph: &mut LayoutGraph,
    ranks: &mut crate::types::RankSystem,
) -> Result<Vec<EdgeChain>, LayoutError> {
    let mut chains = Vec::new();
    let mut new_edges = Vec::new();

    // Process each edge
    for edge in &graph.edges {
        let from_node = graph.nodes.get(edge.from)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let to_node = graph.nodes.get(edge.to)
            .ok_or(LayoutError::DanglingEdge { from: edge.from, to: edge.to })?;
        let rank_from = from_node.rank.ok_or(LayoutError::MissingRank(edge.from))?;
        let rank_to = to_node.rank.ok_or(LayoutError::MissingRank(edge.to))?;
        let span = rank_to as isize - rank_from as isize;

        // Original endpoints before cycle breaking
        let orig_source = if edge.reversed { edge.to } else { edge.from };
        let orig_target = if edge.reversed { edge.from } else { edge.to };
        let is_self_loop = edge.from == edge.to;

        if is_self_loop {
            chains.push(EdgeChain {
                source: orig_source,
                target: orig_target,
                reversed: false,
                is_self_loop: true,
                chain: vec![edge.from],
            });
            new_edges.push(LayoutEdge {
                from: edge.from,
                to: edge.to,
                reversed: false,
            });
        } else if span == 1 {
            // Short edge - no dummies needed
            chains.push(EdgeChain {
                source: orig_source,
                target: orig_target,
                reversed: edge.reversed,
                is_self_loop: false,
                chain: vec![edge.from, edge.to],
            });
            new_edges.push(LayoutEdge {
                from: edge.from,
                to: edge.to,
                reversed: edge.reversed,
            });
        } else if span > 1 {
            // Long edge - insert dummies
            let mut chain = vec![edge.from];
            let mut prev = edge.from;

            for r in (rank_from + 1)..rank_to {
                let dummy_id = graph.nodes.len();
                let dummy_node = LayoutNode {
                    id: dummy_id,
                    node_type: NodeType::Dummy,
                    width: 0.0,
                    height: 0.0,
                    x: 0.0,
                    y: 0.0,
                    rank: Some(r),
                    order: None,
                };
                graph.nodes.push(dummy_node);

                // Add dummy to the layer
                if r < ranks.layers.len() {
                    ranks.layers[r].push(dummy_id);
                } else {
                    // Extend layers if needed
                    while ranks.layers.len() <= r {
                        ranks.layers.push(Vec::new());
                    }
                    ranks.layers[r].push(dummy_id);
                }

                new_edges.push(LayoutEdge {
                    from: prev,
                    to: dummy_id,
                    reversed: false,
                });
                chain.push(dummy_id);
                prev = dummy_id;
            }

            // Final segment to target
            new_edges.push(LayoutEdge {
                from: prev,
                to: edge.to,
                reversed: false,
            });
            chain.push(edge.to);

            chains.push(EdgeChain {
                source: orig_source,
                target: orig_target,
                reversed: edge.reversed,
                is_self_loop: false,
                chain,
            });
        } else {
            // Edge going backwards or same rank
            chains.push(EdgeChain {
                source: orig_source,
                target: orig_target,
                reversed: edge.reversed,
                is_self_loop: false,
                chain: vec![edge.from, edge.to],
            });
            new_edges.push(LayoutEdge {
                from: edge.from,
                to: edge.to,
                reversed: edge.reversed,
            });
        }
    }

    // Replace edges with new single-rank edges
    graph.edges = new_edges;

    // Re-seed order for any layer that received new dummy nodes
    for layer in &ranks.layers {
        for (pos, &id) in layer.iter().enumerate() {
            if id < graph.nodes.len() {
                graph.nodes[id].order = Some(pos);
            }
        }
    }

    Ok(chains)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{LayoutNode, NodeType, LayoutEdge, RankSystem, NodeId};

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
    fn short_edge_unchanged() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        let mut ranks = RankSystem {
            layers: vec![vec![0], vec![1]],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);

        let chains = insert_dummy_nodes(&mut graph, &mut ranks).unwrap();

        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].chain, vec![0, 1]);
        assert_eq!(graph.edges.len(), 1);
        assert_eq!(graph.nodes.len(), 2); // No dummies added
    }

    #[test]
    fn long_edge_gets_dummies() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        let mut ranks = RankSystem {
            layers: vec![vec![0], vec![], vec![], vec![1]],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(3);

        let chains = insert_dummy_nodes(&mut graph, &mut ranks).unwrap();

        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].chain.len(), 4); // source + 2 dummies + target
        assert_eq!(chains[0].chain[0], 0);
        assert_eq!(chains[0].chain[3], 1);
        assert_eq!(graph.edges.len(), 3); // 3 single-rank edges
        assert_eq!(graph.nodes.len(), 4); // 2 original  2 dummies
    }

    #[test]
    fn mixed_short_and_long_edges() {
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1), node(2), node(3)],
            edges: vec![
                LayoutEdge { from: 0, to: 1, reversed: false }, // short: rank 0->1
                LayoutEdge { from: 0, to: 3, reversed: false }, // long: rank 0->3
            ],
        };
        let mut ranks = RankSystem {
            layers: vec![vec![0], vec![1], vec![], vec![3]],
        };
        graph.nodes[0].rank = Some(0);
        graph.nodes[1].rank = Some(1);
        graph.nodes[3].rank = Some(3);

        let chains = insert_dummy_nodes(&mut graph, &mut ranks).unwrap();

        assert_eq!(chains.len(), 2);
        // First chain should be short
        assert_eq!(chains[0].chain, vec![0, 1]);
        // Second chain should have dummies
        assert_eq!(chains[1].chain.len(), 4);
    }

    #[test]
    fn missing_rank_returns_error_instead_of_panicking() {
        // Regression test: calling insert_dummy_nodes (or any later phase)
        // before assign_ranks used to panic via unwrap(). It should now
        // report LayoutError::MissingRank.
        let mut graph = LayoutGraph {
            nodes: vec![node(0), node(1)],
            edges: vec![LayoutEdge { from: 0, to: 1, reversed: false }],
        };
        let mut ranks = RankSystem { layers: vec![] };
        // Note: node ranks are never set here.
        match insert_dummy_nodes(&mut graph, &mut ranks) {
            Err(LayoutError::MissingRank(_)) => {}
            other => panic!("expected LayoutError::MissingRank, got {other:?}"),
        }
    }
}