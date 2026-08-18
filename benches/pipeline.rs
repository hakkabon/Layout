//! Benchmarks for the layout pipeline.
//!
//! Two groups:
//! - `layout_pipeline_end_to_end`: full pipeline (cycle-breaking through edge
//!   routing) across a range of graph sizes and three representative shapes
//!   (a deep narrow chain, a single wide fan-out layer, and a multi-layer DAG
//!   with real cross-layer sharing, closer to what an SPPF or GSS actually
//!   looks like) and both coordinate algorithms. This is the number to watch
//!   for "does this still feel instant at the graph sizes Sample-App/
//!   Swift-Layout consumers actually produce."
//! - `layout_pipeline_phases`: the same large graph, broken down phase by
//!   phase, so a future regression (or a future optimization) shows up
//!   against a specific phase rather than "the whole thing got slower."
//!   This is also the empirical check on the `# Complexity` doc comments
//!   already on `insert_dummy_nodes`/`order_layers`/etc. — if a phase's
//!   measured scaling stops matching its documented Big-O as sizes grow,
//!   that's worth a closer look.
//!
//! Run with `cargo bench --bench pipeline`.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use layout::{
    assign_coordinates, assign_ranks, break_cycles, insert_dummy_nodes, order_layers,
    route_edges, CoordAlgorithm, CoordConfig, EdgeChain, LayoutEdge, LayoutGraph, LayoutNode,
    NodeType, RoutingStyle,
};

/// Same phase order as `ffi::run_pipeline` / `tests/pipeline_properties.rs`'s
/// `run_full_pipeline` — see the comment there for why self-loops are
/// extracted up front and pushed back in as chains after dummy-node
/// insertion rather than flowing through ranking.
fn run_full_pipeline(mut graph: LayoutGraph, config: &CoordConfig, sweeps: usize, routing: RoutingStyle) {
    let self_loops = graph.extract_self_loops();
    let _ = break_cycles(&mut graph);
    let mut ranks = assign_ranks(&mut graph).expect("benchmark graphs are always acyclic-after-break-cycles");
    let mut chains = insert_dummy_nodes(&mut graph, &mut ranks).expect("ranks were just assigned");
    for e in self_loops {
        chains.push(EdgeChain {
            source: e.from,
            target: e.to,
            reversed: false,
            is_self_loop: true,
            label_size: e.label_size,
            chain: vec![e.from],
        });
    }
    order_layers(&mut graph, &mut ranks, sweeps).expect("ranks were just assigned");
    assign_coordinates(&mut graph, &ranks, config).expect("ranks were just assigned");
    let _ = route_edges(&graph, &chains, routing).expect("chains came from insert_dummy_nodes");
}

fn node(id: usize) -> LayoutNode {
    LayoutNode {
        id,
        node_type: NodeType::Normal,
        width: 80.0,
        height: 32.0,
        x: 0.0,
        y: 0.0,
        rank: None,
        order: None,
    }
}

/// A deep, narrow chain: n nodes, n-1 edges, one node per rank. Stresses
/// `assign_ranks`' longest-path relaxation and little else — every layer
/// has exactly one node, so crossing reduction and intra-layer compaction
/// are trivial.
fn linear_chain(n: usize) -> LayoutGraph {
    let nodes = (0..n).map(node).collect();
    let edges = (0..n.saturating_sub(1))
        .map(|i| LayoutEdge { from: i, to: i + 1, reversed: false, label_size: None })
        .collect();
    LayoutGraph { nodes, edges }
}

/// One root fanning out to `width` children, all `width` children then
/// converging on a single sink: three ranks, but the middle one has
/// `width` siblings. Stresses crossing reduction's per-layer barycenter
/// sort and coordinate assignment's intra-layer compaction — exactly the
/// code path the `median_relax_no_sibling_overlap` regression test and the
/// `siblings_in_same_layer_never_overlap` property test exercise, just at
/// a much larger scale.
fn wide_fanout(width: usize) -> LayoutGraph {
    let n = width + 2;
    let root = 0;
    let sink = n - 1;
    let nodes = (0..n).map(node).collect();
    let mut edges = Vec::with_capacity(width * 2);
    for child in 1..=width {
        edges.push(LayoutEdge { from: root, to: child, reversed: false, label_size: None });
        edges.push(LayoutEdge { from: child, to: sink, reversed: false, label_size: None });
    }
    LayoutGraph { nodes, edges }
}

/// A multi-layer DAG with real cross-layer sharing — closer to an SPPF or
/// GSS than either shape above, both of which are trees underneath. Each
/// node in layer i connects to a handful of nodes in layer i+1, so lower
/// layers get genuine fan-in (multiple parents sharing a child), which is
/// exactly the case a tree-shaped benchmark can't exercise. Uses a tiny
/// fixed-seed xorshift rather than pulling in `rand` as another
/// dependency — deterministic, so a benchmark run is reproducible, and the
/// exact distribution doesn't matter for a throughput measurement.
fn layered_dag(layers: usize, per_layer: usize, fanout: usize) -> LayoutGraph {
    let n = layers * per_layer;
    let nodes = (0..n).map(node).collect();
    let mut edges = Vec::new();
    let mut state: u64 = 0x9E3779B97F4A7C15;
    let mut next = move |bound: usize| -> usize {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state as usize) % bound.max(1)
    };
    for layer in 0..layers.saturating_sub(1) {
        for i in 0..per_layer {
            let from = layer * per_layer + i;
            for _ in 0..fanout {
                let to = (layer + 1) * per_layer + next(per_layer);
                edges.push(LayoutEdge { from, to, reversed: false, label_size: None });
            }
        }
    }
    LayoutGraph { nodes, edges }
}

fn end_to_end(c: &mut Criterion) {
    let sizes = [50usize, 200, 800, 3200];
    let mut group = c.benchmark_group("layout_pipeline_end_to_end");

    for &size in &sizes {
        for algorithm in [CoordAlgorithm::MedianRelax, CoordAlgorithm::BrandesKopf] {
            let config = CoordConfig { algorithm, ..Default::default() };

            group.bench_with_input(
                BenchmarkId::new(format!("chain/{algorithm:?}"), size),
                &size,
                |b, &size| {
                    b.iter(|| run_full_pipeline(linear_chain(size), &config, 4, RoutingStyle::Bezier));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("wide_fanout/{algorithm:?}"), size),
                &size,
                |b, &size| {
                    b.iter(|| run_full_pipeline(wide_fanout(size), &config, 4, RoutingStyle::Bezier));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("layered_dag/{algorithm:?}"), size),
                &size,
                |b, &size| {
                    // ~size nodes total, spread over layers of 8 with 3-way fanout.
                    let per_layer = 8;
                    let layers = (size / per_layer).max(2);
                    b.iter(|| run_full_pipeline(layered_dag(layers, per_layer, 3), &config, 4, RoutingStyle::Bezier));
                },
            );
        }
    }
    group.finish();
}

fn phases(c: &mut Criterion) {
    // One large, representative graph — a layered DAG with sharing, since
    // that's the shape real SPPF/GSS consumers actually produce — broken
    // down phase by phase.
    let graph = layered_dag(100, 8, 3); // 800 nodes
    let mut group = c.benchmark_group("layout_pipeline_phases");

    group.bench_function("assign_ranks", |b| {
        b.iter(|| {
            let mut g = graph.clone();
            let _ = break_cycles(&mut g);
            assign_ranks(&mut g).unwrap();
        });
    });

    group.bench_function("insert_dummy_nodes", |b| {
        b.iter(|| {
            let mut g = graph.clone();
            let _ = break_cycles(&mut g);
            let mut ranks = assign_ranks(&mut g).unwrap();
            insert_dummy_nodes(&mut g, &mut ranks).unwrap();
        });
    });

    group.bench_function("order_layers", |b| {
        b.iter(|| {
            let mut g = graph.clone();
            let _ = break_cycles(&mut g);
            let mut ranks = assign_ranks(&mut g).unwrap();
            insert_dummy_nodes(&mut g, &mut ranks).unwrap();
            order_layers(&mut g, &mut ranks, 4).unwrap();
        });
    });

    for algorithm in [CoordAlgorithm::MedianRelax, CoordAlgorithm::BrandesKopf] {
        let config = CoordConfig { algorithm, ..Default::default() };
        group.bench_function(format!("assign_coordinates/{algorithm:?}"), |b| {
            b.iter(|| {
                let mut g = graph.clone();
                let _ = break_cycles(&mut g);
                let mut ranks = assign_ranks(&mut g).unwrap();
                insert_dummy_nodes(&mut g, &mut ranks).unwrap();
                order_layers(&mut g, &mut ranks, 4).unwrap();
                assign_coordinates(&mut g, &ranks, &config).unwrap();
            });
        });
    }

    group.bench_function("route_edges", |b| {
        b.iter(|| {
            let mut g = graph.clone();
            let _ = break_cycles(&mut g);
            let mut ranks = assign_ranks(&mut g).unwrap();
            let mut chains = insert_dummy_nodes(&mut g, &mut ranks).unwrap();
            order_layers(&mut g, &mut ranks, 4).unwrap();
            assign_coordinates(&mut g, &ranks, &CoordConfig::default()).unwrap();
            chains.retain(|c| !c.is_self_loop);
            route_edges(&g, &chains, RoutingStyle::Bezier).unwrap();
        });
    });

    group.finish();
}

criterion_group!(benches, end_to_end, phases);
criterion_main!(benches);
