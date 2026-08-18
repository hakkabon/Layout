//! Property-based tests for the full layout pipeline.
//!
//! `cargo test --lib` covers specific, hand-picked graphs — a diamond, a
//! linear chain, a known cycle. Those are good regression tests once a bug
//! is understood, but they can't tell you about the graph shape nobody
//! thought to write by hand. These tests instead generate a wide variety of
//! random graphs (arbitrary node counts, edge counts, self-loops, multi-edges,
//! disconnected components, cycles) and assert invariants that should hold
//! for *any* well-formed input, regardless of its specific shape.
//!
//! `run_full_pipeline` below mirrors `ffi::run_pipeline` phase-for-phase —
//! same self-loop extraction, same manual self-loop chain construction — but
//! skips the FFI id translation, so this compiles and runs under a plain
//! `cargo test` without the `ffi` feature (which needs a newer toolchain than
//! the core crate does; see the comment on `bindgen-cli` in `Cargo.toml`).

use layout::{
    assign_coordinates, assign_ranks, break_cycles, insert_dummy_nodes, order_layers,
    route_edges, CoordAlgorithm, CoordConfig, EdgeChain, EdgeRoute, LayoutDirection, LayoutEdge,
    LayoutError, LayoutGraph, LayoutNode, NodeType, RoutingStyle,
};
use proptest::prelude::*;

/// Runs cycle-breaking through edge-routing in the same order
/// `ffi::run_pipeline` does, including pushing self-loops back in as
/// explicit `EdgeChain`s after `insert_dummy_nodes` (they're extracted
/// before ranking, since a self-loop can never resolve in Kahn's-algorithm
/// rank assignment — see `LayoutGraph::extract_self_loops`'s doc comment).
fn run_full_pipeline(
    mut graph: LayoutGraph,
    coord_config: &CoordConfig,
    sweeps: usize,
    routing: RoutingStyle,
) -> Result<(LayoutGraph, Vec<EdgeChain>, Vec<EdgeRoute>), LayoutError> {
    let self_loops = graph.extract_self_loops();
    let _reversed = break_cycles(&mut graph);
    let mut ranks = assign_ranks(&mut graph)?;
    let mut chains = insert_dummy_nodes(&mut graph, &mut ranks)?;

    for loop_edge in self_loops {
        chains.push(EdgeChain {
            source: loop_edge.from,
            target: loop_edge.to,
            reversed: false,
            is_self_loop: true,
            label_size: loop_edge.label_size,
            chain: vec![loop_edge.from],
        });
    }

    order_layers(&mut graph, &mut ranks, sweeps)?;
    assign_coordinates(&mut graph, &ranks, coord_config)?;
    let routes = route_edges(&graph, &chains, routing)?;
    Ok((graph, chains, routes))
}

/// Cap on generated graph size. Kept small so the full property suite (six
/// properties x hundreds of cases each) runs in well under a second — large
/// enough to hit multi-layer, multi-sibling, multi-edge, and self-loop
/// cases, not so large that a shrunk failure is painful to read.
const MAX_NODES: usize = 24;

/// A `LayoutNode` needs a real width/height for coordinate assignment's
/// separation math to mean anything; `4.0..120.0` keeps sizes realistic
/// without letting proptest waste shrinking time near zero.
fn arb_graph() -> impl Strategy<Value = LayoutGraph> {
    (1..=MAX_NODES).prop_flat_map(|n| {
        let nodes = prop::collection::vec((4.0f32..120.0, 4.0f32..120.0), n..=n).prop_map(
            move |sizes| {
                sizes
                    .into_iter()
                    .enumerate()
                    .map(|(id, (width, height))| LayoutNode {
                        id,
                        node_type: NodeType::Normal,
                        width,
                        height,
                        ..Default::default()
                    })
                    .collect::<Vec<_>>()
            },
        );

        // Endpoints range over the full 0..n, so `from == to` (self-loops)
        // and repeated pairs (multi-edges) both show up naturally, rather
        // than being generated as a separate special case.
        let max_edges = (n * 2).max(1);
        let edges = prop::collection::vec(
            (
                0..n,
                0..n,
                prop::option::weighted(0.3, (4.0f32..40.0, 4.0f32..20.0)),
            ),
            0..=max_edges,
        )
        .prop_map(|raw| {
            raw.into_iter()
                .map(|(from, to, label_size)| LayoutEdge {
                    from,
                    to,
                    reversed: false,
                    label_size,
                })
                .collect::<Vec<_>>()
        });

        (nodes, edges).prop_map(|(nodes, edges)| LayoutGraph { nodes, edges })
    })
}

fn arb_config() -> impl Strategy<Value = (CoordConfig, usize, RoutingStyle)> {
    (
        0.0f32..30.0,
        0.0f32..60.0,
        0usize..6,
        prop_oneof![
            Just(CoordAlgorithm::MedianRelax),
            Just(CoordAlgorithm::BrandesKopf)
        ],
        prop_oneof![
            Just(LayoutDirection::TopToBottom),
            Just(LayoutDirection::LeftToRight)
        ],
        0usize..6,
        prop_oneof![
            Just(RoutingStyle::Straight),
            Just(RoutingStyle::Orthogonal),
            Just(RoutingStyle::Bezier),
        ],
    )
        .prop_map(
            |(h_gap, v_gap, relax_passes, algorithm, direction, sweeps, routing)| {
                (
                    CoordConfig {
                        h_gap,
                        v_gap,
                        relax_passes,
                        algorithm,
                        direction,
                    },
                    sweeps,
                    routing,
                )
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    /// Every coordinate the engine hands back — node positions and every
    /// route waypoint — is finite. A NaN or Inf here is exactly the kind of
    /// thing that vanishes silently in a SwiftUI renderer (as the earlier
    /// negative-coordinate clipping issue showed for out-of-frame values)
    /// rather than failing loudly, so it's worth asserting directly instead
    /// of relying on a human noticing a missing node or edge.
    #[test]
    fn coordinates_are_always_finite((graph, (config, sweeps, routing)) in (arb_graph(), arb_config())) {
        if let Ok((laid_out, _chains, routes)) = run_full_pipeline(graph, &config, sweeps, routing) {
            for node in &laid_out.nodes {
                prop_assert!(node.x.is_finite(), "node {} has non-finite x: {}", node.id, node.x);
                prop_assert!(node.y.is_finite(), "node {} has non-finite y: {}", node.id, node.y);
            }
            for route in &routes {
                for &(x, y) in &route.waypoints {
                    prop_assert!(
                        x.is_finite() && y.is_finite(),
                        "route {}->{} has a non-finite waypoint ({}, {})",
                        route.source, route.target, x, y
                    );
                }
            }
        }
    }

    /// `route_edges` produces exactly one route per chain — no edge
    /// silently dropped or duplicated between dummy-node insertion and
    /// final routing, across arbitrary graph shapes (this is the general
    /// form of what `route_edges_multi_edges_have_distinct_waypoints` in
    /// `coordinates.rs` checks for one hand-picked case).
    #[test]
    fn every_chain_gets_exactly_one_route((graph, (config, sweeps, routing)) in (arb_graph(), arb_config())) {
        if let Ok((_laid_out, chains, routes)) = run_full_pipeline(graph, &config, sweeps, routing) {
            prop_assert_eq!(routes.len(), chains.len());
        }
    }

    /// Every route has at least two waypoints (a start and an end).
    /// Nothing downstream — arrowhead placement, label placement, or a
    /// SwiftUI `Canvas` stroke — should ever have to handle a degenerate
    /// single-point or empty route.
    #[test]
    fn every_route_has_at_least_two_waypoints((graph, (config, sweeps, routing)) in (arb_graph(), arb_config())) {
        if let Ok((_laid_out, _chains, routes)) = run_full_pipeline(graph, &config, sweeps, routing) {
            for route in &routes {
                prop_assert!(
                    route.waypoints.len() >= 2,
                    "route {}->{} has only {} waypoint(s)",
                    route.source, route.target, route.waypoints.len()
                );
            }
        }
    }

    /// No two nodes in the same layer visually overlap. This generalizes
    /// the hand-written `brandes_kopf_no_sibling_overlap` regression test
    /// in `coordinates.rs` to arbitrary graphs and both coordinate
    /// algorithms — sibling overlap is exactly the kind of bug that's easy
    /// to reintroduce in a future coordinate-assignment change and easy to
    /// miss with hand-picked test graphs alone.
    #[test]
    fn siblings_in_same_layer_never_overlap((graph, (config, sweeps, routing)) in (arb_graph(), arb_config())) {
        let is_lr = config.direction == LayoutDirection::LeftToRight;
        if let Ok((laid_out, _chains, _routes)) = run_full_pipeline(graph, &config, sweeps, routing) {
            let mut by_rank: std::collections::HashMap<usize, Vec<&LayoutNode>> = std::collections::HashMap::new();
            for node in laid_out.nodes.iter().filter(|n| n.node_type == NodeType::Normal) {
                if let Some(rank) = node.rank {
                    by_rank.entry(rank).or_default().push(node);
                }
            }
            for nodes in by_rank.values() {
                for i in 0..nodes.len() {
                    for j in (i + 1)..nodes.len() {
                        let (a, b) = (nodes[i], nodes[j]);
                        let (a_pos, a_extent) = if is_lr { (a.y, a.height) } else { (a.x, a.width) };
                        let (b_pos, b_extent) = if is_lr { (b.y, b.height) } else { (b.x, b.width) };
                        let min_sep = a_extent / 2.0 + b_extent / 2.0;
                        prop_assert!(
                            (a_pos - b_pos).abs() >= min_sep - 0.01,
                            "nodes {} and {} in the same layer overlap: positions {} and {}, need >= {} apart",
                            a.id, b.id, a_pos, b_pos, min_sep
                        );
                    }
                }
            }
        }
    }

    /// The pipeline is a pure function of its input: running it twice on
    /// an identical graph must produce bit-identical output. Any
    /// nondeterminism here (say, from iterating a `HashMap` where order
    /// happens to matter) wouldn't fail a single test run — it would show
    /// up downstream as layout "jitter" between two otherwise-identical
    /// relayouts in Swift-Layout, which is a much harder thing to notice
    /// and track back to its source than a failing assertion here.
    #[test]
    fn pipeline_is_deterministic((graph, (config, sweeps, routing)) in (arb_graph(), arb_config())) {
        let first = run_full_pipeline(graph.clone(), &config, sweeps, routing);
        let second = run_full_pipeline(graph, &config, sweeps, routing);
        match (first, second) {
            (Ok((g1, _, r1)), Ok((g2, _, r2))) => {
                prop_assert_eq!(g1.nodes.len(), g2.nodes.len());
                for (n1, n2) in g1.nodes.iter().zip(g2.nodes.iter()) {
                    prop_assert_eq!(n1.x.to_bits(), n2.x.to_bits());
                    prop_assert_eq!(n1.y.to_bits(), n2.y.to_bits());
                }
                prop_assert_eq!(r1.len(), r2.len());
                for (route1, route2) in r1.iter().zip(r2.iter()) {
                    prop_assert_eq!(route1.waypoints.len(), route2.waypoints.len());
                    for (&(x1, y1), &(x2, y2)) in route1.waypoints.iter().zip(route2.waypoints.iter()) {
                        prop_assert_eq!(x1.to_bits(), x2.to_bits());
                        prop_assert_eq!(y1.to_bits(), y2.to_bits());
                    }
                }
            }
            (Err(e1), Err(e2)) => prop_assert_eq!(e1, e2),
            (r1, r2) => prop_assert!(
                false,
                "pipeline result differs between two runs of the same input: ok={} vs ok={}",
                r1.is_ok(), r2.is_ok()
            ),
        }
    }
}
