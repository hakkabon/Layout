# Layout Engine

A hierarchical graph layout engine implementing the Sugiyama algorithm for directed acyclic graphs (DAGs).

## Overview

This library produces clean, readable layouts for directed graphs by arranging nodes in hierarchical layers and computing edge routes. It's suitable for visualizing:

- Flowcharts and process diagrams
- Dependency graphs
- Class hierarchies
- Data flow diagrams
- Organizational charts

## Pipeline Phases

The layout process consists of four main phases:

### Cycle Breaking

**Function:** `break_cycles(graph)`

Uses iterative DFS to detect and break cycles by reversing back-edges. Returns indices of reversed edges for potential restoration during rendering. Note: this index list becomes stale after `insert_dummy_nodes` rewrites `graph.edges`, so treat it as informational/debug-only rather than something to hold onto across phases.

Self-loops (`from == to`) are a separate concern: reversing one is a no-op, so it survives `break_cycles` unchanged, and rank assignment can never resolve it (its own in-degree contribution can't be cleared until the node itself is processed). Call `graph.extract_self_loops()` before `break_cycles` to pull these out and render them separately — see the Usage Example below.

- **Algorithm:** DFS back-edge reversal (iterative, stack-based)
- **Complexity:** O(N + E)
- **Output:** Modified graph with `reversed` flags on affected edges

### Rank Assignment

**Function:** `assign_ranks(graph) -> Result<RankSystem, LayoutError>`

Assigns nodes to ranks (layers) using longest-path ranking, which minimizes the number of layers (minimum height). Internally calls `validate_graph` first, so a malformed graph (dangling edge, non-dense node ids, a self-loop) reports a specific `LayoutError` variant instead of panicking.

- **Algorithm:** Kahn's algorithm with longest-path tracking
- **Complexity:** O(N + E)
- **Output:** `RankSystem` with nodes grouped by layer

### Dummy Node Insertion

**Function:** `insert_dummy_nodes(graph, ranks) -> Result<Vec<EdgeChain>, LayoutError>`

Splits long edges (spanning multiple ranks) into chains of single-rank segments by inserting dummy nodes. Takes `ranks` by `&mut` because new dummy nodes are appended to their layer as they're created.

- **Algorithm:** Edge decomposition
- **Complexity:** O(E · S) where S = average edge span
- **Output:** `Vec<EdgeChain>` tracking original edge decomposition

### Layer Ordering

**Function:** `order_layers(graph, ranks, sweeps) -> Result<(), LayoutError>`

Orders nodes within each layer to minimize edge crossings using the barycentric heuristic with median-based refinement, followed by a bounded transpose cleanup pass (capped at 100 sweeps, so a pathological graph can't force unbounded work).

- **Algorithm:** Barycentric ordering with median relaxation
- **Complexity:** O(N + E) per barycenter sweep; the transpose pass adds up to `MAX_TRANSPOSE_PASSES` additional O(N) sweeps
- **Output:** Graph with updated `order` field on nodes

### Coordinate Assignment

**Function:** `assign_coordinates(graph, ranks, config) -> Result<(), LayoutError>`

Assigns x/y coordinates to nodes. Supports two algorithms:

#### Median Relaxation (Default)

Simpler and faster, uses weighted median relaxation with compaction.

- **Complexity:** O((N + E) · passes)
- **Best for:** Quick layouts, smaller graphs

#### Brandes-Köpf-Inspired Alignment

**Not the published Brandes-Köpf algorithm** — that algorithm resolves type-1/type-2 conflicts between dummy-node chains and regular edges and aligns nodes within the resulting blocks. What's implemented here is a cheaper approximation: four directional weighted-average passes (top-down/bottom-up × left/right-aligned), averaged together, followed by a repair pass (isotonic regression / pool-adjacent-violators) that restores minimum spacing and a damped recentering loop that pulls each node toward the mean of all its neighbors. The repair step matters: naively averaging four independently-compacted layouts can otherwise collapse siblings with identical local structure — e.g. two children of the same single parent — onto the exact same x-coordinate. See `coordinates::tests::brandes_kopf_no_sibling_overlap` for the regression case.

- **Complexity:** O(N + E) per pass, small constant number of passes
- **Best for:** Publication-quality layouts, graphs with many dummy chains — but without true BK's guarantee that dummy chains crossing other edges come out straight

### Edge Routing

**Function:** `route_edges(graph, chains, style) -> Result<Vec<EdgeRoute>, LayoutError>`

Computes edge routes based on node positions and edge chains. Three routing styles available:

- **Straight:** Direct line from source to target (2 waypoints)
- **Orthogonal:** Right-angle polylines through dummy node midpoints
- **Bezier:** Smooth cubic Bézier curves using Catmull-Rom conversion

## Usage Example

```rust
use layout_engine::*;

// Build your graph. `add_node` assigns a dense NodeId automatically —
// every phase below indexes nodes directly by id, so ids must match their
// position in `graph.nodes`. Prefer add_node over pushing LayoutNodes by hand.
let mut graph = LayoutGraph::default();
let a = graph.add_node(LayoutNode { width: 50.0, height: 30.0, ..Default::default() });
let b = graph.add_node(LayoutNode { width: 50.0, height: 30.0, ..Default::default() });
graph.edges.push(LayoutEdge { from: a, to: b, reversed: false });

// Optional: pull out self-loops before ranking. Sugiyama ranking has no
// way to place a self-loop (from == to); leaving one in place causes
// assign_ranks to report LayoutError::CyclicGraph even after break_cycles.
// Render extracted self-loops separately, e.g. as a small loop decoration.
let _self_loops = graph.extract_self_loops();

// Phase 0: Break cycles (if any)
let _reversed = break_cycles(&mut graph);

// Phase 1: Assign ranks
let mut ranks = assign_ranks(&mut graph)?;

// Phase 2a: Insert dummy nodes for long edges (mutates `ranks` in place —
// new dummy nodes get added to their layer)
let chains = insert_dummy_nodes(&mut graph, &mut ranks)?;

// Phase 2b: Order layers to minimize crossings (`sweeps` controls how many
// barycenter passes to run; each is followed by a bounded transpose cleanup)
order_layers(&mut graph, &mut ranks, 4)?;

// Phase 3: Assign coordinates
let config = CoordConfig {
    h_gap: 20.0,
    v_gap: 40.0,
    relax_passes: 4,
    algorithm: CoordAlgorithm::BrandesKopf, // or CoordAlgorithm::MedianRelax
};
assign_coordinates(&mut graph, &ranks, &config)?;

// Route edges
let routes = route_edges(&graph, &chains, RoutingStyle::Bezier)?;

// Use graph.nodes[i].x and graph.nodes[i].y for rendering
// Use routes for edge drawing
```

Every phase after `break_cycles` returns `Result<_, LayoutError>`: `CyclicGraph`
if a cycle survives cycle breaking, `InvalidNodeId`/`DanglingEdge` if the
graph violates the dense-id invariant, `SelfLoop` if a self-loop edge wasn't
extracted first, or `MissingRank` if a phase runs before `assign_ranks`. You
can also call `validate_graph(&graph)` explicitly up front — `assign_ranks`
already does this internally, so it's mainly useful for surfacing a bad
graph earlier, before you've done any other work.

## Configuration

### CoordConfig

| Field          | Type             | Default       | Description                                           |
| -------------- | ---------------- | ------------- | ----------------------------------------------------- |
| `h_gap`        | `f32`            | 20.0          | Minimum horizontal gap between adjacent nodes         |
| `v_gap`        | `f32`            | 40.0          | Vertical gap between rank layers                      |
| `relax_passes` | `usize`          | 4             | Number of median-relaxation passes (MedianRelax only) |
| `algorithm`    | `CoordAlgorithm` | `MedianRelax` | X-coordinate assignment algorithm                     |

### CoordAlgorithm

- `MedianRelax` - Weighted median relaxation (faster, simpler)
- `BrandesKopf` - Four-pass alignment averaging (better quality)

### RoutingStyle

- `Straight` - Direct lines
- `Orthogonal` - Right-angle bends
- `Bezier` - Smooth curves

## Module Structure

```
src/
├── coordinates.rs  — Coordinate assignment and edge routing
├── crossings.rs    — Layer ordering (crossing minimization)
├── cycles.rs       — Cycle breaking (DFS back-edge reversal)       
├── dummy_nodes.rs  — Dummy node insertion (edge splitting)
├── lib.rs          — Module declarations and re-exports
├── ranks.rs.       — Rank assignment (longest-path)
├── types.rs        — Shared types (LayoutNode, LayoutEdge, etc.)  
└── validate.rs     — Graph validation utilities
```

## Algorithm Comparison

### X-Coordinate Assignment

| Algorithm              | Quality | Speed    | Best Use Case                                    |
| ---------------------- | ------- | -------- | ------------------------------------------------- |
| Median Relaxation      | Good    | Fast     | Interactive applications, large graphs             |
| Brandes-Köpf (approx.) | Better  | Moderate | Static diagrams, graphs with many dummy chains     |

The Brandes-Köpf-inspired algorithm produces more balanced layouts than median relaxation, especially for graphs with many long edges and dummy chains, by averaging four independent alignments computed from different corners and repairing the result for minimum spacing and symmetry. It's an approximation of the published algorithm, not a full implementation — see the note in [Coordinate Assignment](#coordinate-assignment).

## Performance Notes

- All phases are linear or near-linear in graph size
- The layout engine handles graphs with thousands of nodes efficiently
- For very large graphs (>10k nodes), consider reducing `relax_passes` or using fewer iterations in `order_layers`

## Optional Features

### `serde`

Enables `Serialize`/`Deserialize` on all public types (`LayoutGraph`, `LayoutNode`, `LayoutEdge`, `RankSystem`, `EdgeChain`, `EdgeRoute`, `CoordConfig`, `LayoutError`, etc.):

```toml
layout_engine = { path = "...", features = ["serde"] }
```

Useful as a quick JSON bridge to another language (e.g. serialize a `LayoutGraph` + `Vec<EdgeRoute>` to JSON and decode it into `Codable` structs on the Swift side) before committing to a binding generator, or as a format for golden-file tests.

## Breaking Changes (v0.2)

- `insert_dummy_nodes`, `order_layers`, `assign_coordinates`, and `route_edges` now return `Result<_, LayoutError>` instead of panicking on missing ranks or out-of-bounds node ids.
- `insert_dummy_nodes` and `order_layers` now take `ranks: &mut RankSystem` (previously `&RankSystem` in some call sites, inconsistently).
- `LayoutError` gained `InvalidNodeId`, `DanglingEdge`, `SelfLoop`, and `MissingRank` variants; `validate_graph` now returns the specific variant instead of overloading `CyclicGraph` for everything.
- Added `LayoutGraph::add_node` and `LayoutGraph::extract_self_loops`.
- `LayoutNode` and `NodeType` now implement `Default`.
- Fixed a real layout bug in Brandes-Köpf coordinate assignment where siblings with identical local structure (e.g. two children of the same parent) could average onto the same x-coordinate and overlap; see `coordinates::tests::brandes_kopf_no_sibling_overlap`.
- `order_layers`'s internal transpose pass is now capped at `MAX_TRANSPOSE_PASSES` (100) rather than looping to a fixed point unconditionally.

## Breaking Changes (v2.0)

- `CoordinateConfig` replaced with `CoordConfig` supporting multiple algorithms
- Added `CoordAlgorithm` enum for algorithm selection
- `LayoutEdge` now includes a `reversed` field for cycle tracking

## License

MIT License - see LICENSE file for details.
