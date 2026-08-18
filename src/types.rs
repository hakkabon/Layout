//! Shared types for the layout engine.

/// Dense, zero-based index into [`LayoutGraph::nodes`]. Every phase in this
/// crate indexes nodes directly by this value, so `graph.nodes[id].id ==
/// id` must always hold — see [`LayoutGraph::add_node`].
pub type NodeId = usize;
/// Index of a layer within [`RankSystem::layers`], assigned by
/// `assign_ranks`.
pub type RankId = usize;

/// Whether a [`LayoutNode`] is part of the original input graph, or was
/// synthesized by `insert_dummy_nodes` as a waypoint for an edge spanning
/// more than one rank.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub enum NodeType {
    /// A node from the original input graph.
    #[default]
    Normal,
    /// A synthesized waypoint node inserted by `insert_dummy_nodes` so a
    /// long edge can be routed rank-by-rank. Not part of the caller's
    /// original graph; excluded from most caller-facing output.
    Dummy,
}

/// A single node in the graph being laid out.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct LayoutNode {
    /// This node's id — must equal its own index in [`LayoutGraph::nodes`].
    /// Prefer [`LayoutGraph::add_node`] over constructing this by hand to
    /// avoid violating that invariant.
    pub id: NodeId,
    /// Whether this is an original node or a dummy waypoint (see
    /// [`NodeType`]).
    pub node_type: NodeType,
    /// Visual width, used by coordinate assignment's separation math.
    pub width: f32,
    /// Visual height, used by coordinate assignment's separation math.
    pub height: f32,
    /// Assigned x-coordinate. Meaningless until `assign_coordinates` has
    /// run.
    pub x: f32,
    /// Assigned y-coordinate. Meaningless until `assign_coordinates` has
    /// run.
    pub y: f32,
    /// Which layer this node belongs to. `None` until `assign_ranks` has
    /// run.
    pub rank: Option<RankId>,
    /// Position of this node within its rank's layer. Used by, and updated
    /// by, the crossing-reduction pass. `None` until `assign_ranks` has run.
    pub order: Option<usize>,
}

/// A single edge in the graph being laid out.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct LayoutEdge {
    /// Source node id.
    pub from: NodeId,
    /// Target node id.
    pub to: NodeId,
    /// True if this edge was reversed by [`break_cycles`] and should be
    /// drawn with its arrowhead at the visually higher end.
    pub reversed: bool,
    /// Optional label width and height (for obstacle-free label placement).
    #[cfg_attr(feature = "serde", serde(default))]
    pub label_size: Option<(f32, f32)>,
}

/// A graph to lay out: a dense, zero-indexed node list plus a set of edges
/// between them.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone)]
pub struct LayoutGraph {
    /// All nodes, indexed by [`NodeId`] — `nodes[id].id == id` must hold
    /// for every node (see [`LayoutGraph::add_node`]).
    pub nodes: Vec<LayoutNode>,
    /// All edges between those nodes.
    pub edges: Vec<LayoutEdge>,
}

impl LayoutGraph {
    /// Appends a new node and returns its `NodeId`.
    ///
    /// This is the preferred way to build a graph: every phase in this
    /// crate assumes `node.id == nodes[node.id]`'s index (a dense,
    /// zero-based array). Constructing `LayoutNode`s by hand and pushing
    /// them onto `graph.nodes` directly makes it easy to violate that
    /// invariant; `add_node` assigns the id for you so it can't drift.
    ///
    /// ```
    /// use layout::{LayoutGraph, LayoutNode, NodeType};
    /// let mut graph = LayoutGraph::default();
    /// let a = graph.add_node(LayoutNode { width: 50.0, height: 30.0, ..Default::default() });
    /// let b = graph.add_node(LayoutNode { width: 50.0, height: 30.0, node_type: NodeType::Normal, ..Default::default() });
    /// graph.edges.push(layout::LayoutEdge { from: a, to: b, reversed: false, label_size: None });
    /// ```
    pub fn add_node(&mut self, mut node: LayoutNode) -> NodeId {
        let id = self.nodes.len();
        node.id = id;
        self.nodes.push(node);
        id
    }

    /// Removes and returns all self-loop edges (`from == to`).
    ///
    /// Sugiyama-style ranking has no meaningful notion of a self-loop: rank
    /// assignment's Kahn's-algorithm pass can never resolve one (its own
    /// in-degree contribution can only be cleared by processing the node
    /// itself first), so a self-loop left in place causes `assign_ranks`
    /// to report `LayoutError::CyclicGraph` even after `break_cycles` has
    /// run. Call this before `break_cycles` and render the returned edges
    /// separately (e.g. as a small loop decoration on the node) rather
    /// than feeding them through the ranking pipeline.
    pub fn extract_self_loops(&mut self) -> Vec<LayoutEdge> {
        let mut self_loops = Vec::new();
        self.edges.retain(|e| {
            if e.from == e.to {
                self_loops.push(e.clone());
                false
            } else {
                true
            }
        });
        self_loops
    }
}

/// The graph's nodes grouped by rank, in intra-layer order. Produced by
/// `assign_ranks`, consumed by every phase after it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone)]
pub struct RankSystem {
    /// `layers[rank]` is the ordered list of node ids in that layer.
    pub layers: Vec<Vec<NodeId>>,
}

/// Everything that can go wrong running the layout pipeline. Every variant
/// describes a problem with the *input* to some phase (a malformed graph,
/// or a phase called out of order) — the pipeline itself doesn't fail for
/// any other reason.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, PartialEq, Eq, Clone)]
#[non_exhaustive]
pub enum LayoutError {
    /// The input graph has a cycle that survived cycle breaking (or
    /// `break_cycles` was never run).
    CyclicGraph,
    /// `graph.nodes[i].id != i` for some `i`. Every phase indexes nodes
    /// directly by `NodeId`, so ids must be a dense, zero-based sequence
    /// matching each node's position in `graph.nodes`. Prefer
    /// `LayoutGraph::add_node` to avoid this.
    InvalidNodeId {
        /// The node's actual position in `graph.nodes`.
        index: usize,
        /// The mismatched id found at that position.
        id: NodeId,
    },
    /// An edge references a node id that doesn't exist in `graph.nodes`.
    DanglingEdge {
        /// The edge's source id.
        from: NodeId,
        /// The edge's target id.
        to: NodeId,
    },
    /// A self-loop (`from == to`) was found. Use
    /// `LayoutGraph::extract_self_loops` to remove these before ranking.
    SelfLoop(NodeId),
    /// A phase that requires `LayoutNode::rank` to be set (i.e. requires
    /// `assign_ranks` to have already run) encountered a node with `rank
    /// == None`.
    MissingRank(NodeId),
}

impl std::fmt::Display for LayoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LayoutError::CyclicGraph => write!(f, "graph contains a cycle"),
            LayoutError::InvalidNodeId { index, id } => write!(
                f,
                "node at index {index} has id {id}; ids must match their position in `nodes` (use LayoutGraph::add_node)"
            ),
            LayoutError::DanglingEdge { from, to } => {
                write!(f, "edge ({from} -> {to}) references a node id that doesn't exist")
            }
            LayoutError::SelfLoop(id) => write!(
                f,
                "node {id} has a self-loop edge; remove it with LayoutGraph::extract_self_loops before ranking"
            ),
            LayoutError::MissingRank(id) => write!(
                f,
                "node {id} has no rank assigned; call assign_ranks before this phase"
            ),
        }
    }
}

impl std::error::Error for LayoutError {}

/// Layout direction.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum LayoutDirection {
    /// Rank 0 at the top, increasing ranks flow downward. The x-axis is
    /// used for intra-layer (sibling) separation.
    #[default]
    TopToBottom,
    /// Rank 0 at the left, increasing ranks flow rightward. The y-axis is
    /// used for intra-layer (sibling) separation.
    LeftToRight,
}

/// Bounding rectangle for a layout.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LayoutRect {
    /// Minimum x across every node and route waypoint.
    pub min_x: f32,
    /// Minimum y across every node and route waypoint.
    pub min_y: f32,
    /// Maximum x across every node and route waypoint.
    pub max_x: f32,
    /// Maximum y across every node and route waypoint.
    pub max_y: f32,
    /// `max_x - min_x`.
    pub width: f32,
    /// `max_y - min_y`.
    pub height: f32,
}

/// Arrowhead geometry at an edge destination.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Arrowhead {
    /// Tip point where the arrow touches the target node boundary.
    pub tip_x: f32,
    /// Tip point where the arrow touches the target node boundary.
    pub tip_y: f32,
    /// Tangent angle in radians (pointing in the direction of the edge flow).
    pub angle: f32,
    /// Left wing endpoint.
    pub left_x: f32,
    /// Left wing endpoint.
    pub left_y: f32,
    /// Right wing endpoint.
    pub right_x: f32,
    /// Right wing endpoint.
    pub right_y: f32,
}

/// Tracks how a single original (possibly long) edge was decomposed into a
/// chain of single-rank segments by Rank Assignment.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct EdgeChain {
    /// Original edge's source node id.
    pub source: NodeId,
    /// Original edge's target node id.
    pub target: NodeId,
    /// Mirrored from the originating [`LayoutEdge::reversed`].
    pub reversed: bool,
    /// Whether this chain represents a self-loop (`source == target`),
    /// routed specially rather than through the usual rank-by-rank chain.
    pub is_self_loop: bool,
    /// Optional label width/height, mirrored from the originating
    /// [`LayoutEdge::label_size`], for obstacle-free label placement.
    pub label_size: Option<(f32, f32)>,
    /// All waypoint nodes in order: [source, dummy₁, dummy₂, …, target].
    pub chain: Vec<NodeId>,
}

/// Routing style for edge drawing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum RoutingStyle {
    /// Straight line, 2 points only — ignores any dummy waypoints.
    Straight,
    /// Right-angle polyline through dummy midpoints.
    Orthogonal,
    /// Smooth cubic bezier through dummy waypoints.
    #[default]
    Bezier,
}

/// Route for a single edge.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct EdgeRoute {
    /// Original edge's source node id.
    pub source: NodeId,
    /// Original edge's target node id.
    pub target: NodeId,
    /// Mirrored from the originating [`LayoutEdge::reversed`].
    pub reversed: bool,
    /// Whether this route represents a self-loop.
    pub is_self_loop: bool,
    /// Waypoints from source to target.
    /// - Straight:    exactly 2 points (source attachment, target attachment).
    /// - Orthogonal:  polyline starting at source attachment and ending at target attachment (>= 2 points).
    /// - Bezier:      cubic bezier quadruples packed as (P0, C1, C2, P1, C1', C2', P2, …)
    ///                starting at source attachment and ending at target attachment.
    pub waypoints: Vec<(f32, f32)>,
    /// Computed arrowhead geometry at the target node boundary.
    pub arrowhead: Option<Arrowhead>,
    /// Computed obstacle-free label center coordinate (x, y) if a label_size was provided.
    pub label_pos: Option<(f32, f32)>,
}

/// Configuration for coordinate assignment.
///
/// This is a type alias for `CoordConfig` for backward compatibility.
/// New code should use `CoordConfig` directly which supports both
/// the median relaxation and Brandes-Köpf algorithms.
#[deprecated(since = "2.0.0", note = "Use CoordConfig instead, which supports Brandes-Köpf algorithm")]
pub type CoordinateConfig = crate::coordinates::CoordConfig;
/// Re-export CoordConfig from coordinates for direct access.
pub use crate::coordinates::{CoordConfig, CoordAlgorithm};
// Re-export LayoutPipeline for backward compatibility
pub use crate::ranks::LayoutPipeline;

