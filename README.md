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

Uses iterative DFS to detect and break cycles by reversing back-edges. Returns indices of reversed edges for potential restoration during rendering.

- **Algorithm:** DFS back-edge reversal (iterative, stack-based)
- **Complexity:** O(N + E)
- **Output:** Modified graph with `reversed` flags on affected edges

### Rank Assignment

**Function:** `assign_ranks(graph)`

Assigns nodes to ranks (layers) using longest-path ranking, which minimizes the number of layers (minimum height).

- **Algorithm:** Kahn's algorithm with longest-path tracking
- **Complexity:** O(N + E)
- **Output:** `RankSystem` with nodes grouped by layer

### Dummy Node Insertion

**Function:** `insert_dummy_nodes(graph, ranks)`

Splits long edges (spanning multiple ranks) into chains of single-rank segments by inserting dummy nodes.

- **Algorithm:** Edge decomposition
- **Complexity:** O(E · S) where S = average edge span
- **Output:** `Vec<EdgeChain>` tracking original edge decomposition

### Layer Ordering

**Function:** `order_layers(graph, ranks)`

Orders nodes within each layer to minimize edge crossings using the barycentric heuristic with median-based refinement.

- **Algorithm:** Barycentric ordering with median relaxation
- **Complexity:** O(N + E) per iteration
- **Output:** Graph with updated `order` field on nodes

### Coordinate Assignment

**Function:** `assign_coordinates(graph, ranks, config)`

Assigns x/y coordinates to nodes. Supports two algorithms:

#### Median Relaxation (Default)

Simpler and faster, uses weighted median relaxation with compaction.

- **Complexity:** O((N + E) · passes)
- **Best for:** Quick layouts, smaller graphs

#### Brandes-Köpf Alignment

Produces more balanced and symmetric layouts by computing four independent alignments (top-left, top-right, bottom-left, bottom-right) and averaging them.

- **Complexity:** O(N + E)
- **Best for:** Publication-quality layouts, graphs with many dummy chains

### Edge Routing

**Function:** `route_edges(graph, chains, style)`

Computes edge routes based on node positions and edge chains. Three routing styles available:

- **Straight:** Direct line from source to target (2 waypoints)
- **Orthogonal:** Right-angle polylines through dummy node midpoints
- **Bezier:** Smooth cubic Bézier curves using Catmull-Rom conversion

## Usage Example

```rust
use layout_engine::*;

// Build your graph
let mut graph = LayoutGraph {
    nodes: vec![
        LayoutNode { id: 0, node_type: NodeType::Normal, width: 50.0, height: 30.0, ..Default::default() },
        LayoutNode { id: 1, node_type: NodeType::Normal, width: 50.0, height: 30.0, ..Default::default() },
    ],
    edges: vec![
        LayoutEdge { from: 0, to: 1, reversed: false },
    ],
};

// Phase 0: Break cycles (if any)
let _reversed = break_cycles(&mut graph);

// Phase 1: Assign ranks
let ranks = assign_ranks(&graph)?;

// Phase 2a: Insert dummy nodes for long edges
let chains = insert_dummy_nodes(&mut graph, &ranks);

// Phase 2b: Order layers to minimize crossings
order_layers(&mut graph, &ranks);

// Phase 3: Assign coordinates
let config = CoordConfig {
    h_gap: 20.0,
    v_gap: 40.0,
    relax_passes: 4,
    algorithm: CoordAlgorithm::BrandesKopf, // or CoordAlgorithm::MedianRelax
};
assign_coordinates(&mut graph, &ranks, &config);

// Route edges
let routes = route_edges(&graph, &chains, RoutingStyle::Bezier);

// Use graph.nodes[i].x and graph.nodes[i].y for rendering
// Use routes for edge drawing
```

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

| Algorithm         | Quality   | Speed    | Best Use Case                          |
| ----------------- | --------- | -------- | -------------------------------------- |
| Median Relaxation | Good      | Fast     | Interactive applications, large graphs |
| Brandes-Köpf      | Excellent | Moderate | Static diagrams, publication quality   |

The Brandes-Köpf algorithm produces more balanced layouts, especially for graphs with many long edges and dummy chains. It achieves symmetry by averaging four independent alignments computed from different corners.

## Performance Notes

- All phases are linear or near-linear in graph size
- The layout engine handles graphs with thousands of nodes efficiently
- For very large graphs (>10k nodes), consider reducing `relax_passes` or using fewer iterations in `order_layers`

## Breaking Changes (v2.0)

- `CoordinateConfig` replaced with `CoordConfig` supporting multiple algorithms
- Added `CoordAlgorithm` enum for algorithm selection
- `LayoutEdge` now includes a `reversed` field for cycle tracking

## License

MIT License - see LICENSE file for details.
