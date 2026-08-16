//! Shared types for the layout engine.
pub type NodeId = usize;
pub type RankId = usize;
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum NodeType {
    #[default]
    Normal,
    Dummy,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct LayoutNode {
    pub id: NodeId,
    pub node_type: NodeType,
    pub width: f32,
    pub height: f32,
    pub x: f32,
    pub y: f32,
    pub rank: Option<RankId>,
    /// Position of this node within its rank's layer. Used by, and updated
    /// by, the crossing-reduction pass. `None` until `assign_ranks` has run.
    pub order: Option<usize>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct LayoutEdge {
    pub from: NodeId,
    pub to: NodeId,
    /// True if this edge was reversed by [`break_cycles`] and should be
    /// drawn with its arrowhead at the visually higher end.
    pub reversed: bool,
    /// Optional label width and height (for obstacle-free label placement).
    #[cfg_attr(feature = "serde", serde(default))]
    pub label_size: Option<(f32, f32)>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone)]
pub struct LayoutGraph {
    pub nodes: Vec<LayoutNode>,
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

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Default, Debug, Clone)]
pub struct RankSystem {
    pub layers: Vec<Vec<NodeId>>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, PartialEq, Eq, Clone)]
pub enum LayoutError {
    /// The input graph has a cycle that survived cycle breaking (or
    /// `break_cycles` was never run).
    CyclicGraph,
    /// `graph.nodes[i].id != i` for some `i`. Every phase indexes nodes
    /// directly by `NodeId`, so ids must be a dense, zero-based sequence
    /// matching each node's position in `graph.nodes`. Prefer
    /// `LayoutGraph::add_node` to avoid this.
    InvalidNodeId { index: usize, id: NodeId },
    /// An edge references a node id that doesn't exist in `graph.nodes`.
    DanglingEdge { from: NodeId, to: NodeId },
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
pub enum LayoutDirection {
    #[default]
    TopToBottom,
    LeftToRight,
}

/// Bounding rectangle for a layout.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LayoutRect {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub width: f32,
    pub height: f32,
}

/// Arrowhead geometry at an edge destination.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Arrowhead {
    /// Tip point where the arrow touches the target node boundary.
    pub tip_x: f32,
    pub tip_y: f32,
    /// Tangent angle in radians (pointing in the direction of the edge flow).
    pub angle: f32,
    /// Left wing endpoint.
    pub left_x: f32,
    pub left_y: f32,
    /// Right wing endpoint.
    pub right_x: f32,
    pub right_y: f32,
}

/// Tracks how a single original (possibly long) edge was decomposed into a
/// chain of single-rank segments by Rank Assignment.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct EdgeChain {
    pub source: NodeId,   // original edge source
    pub target: NodeId,   // original edge target
    pub reversed: bool,   // mirrored from the LayoutEdge
    pub is_self_loop: bool,
    pub label_size: Option<(f32, f32)>,
    /// All waypoint nodes in order: [source, dummy₁, dummy₂, …, target].
    pub chain: Vec<NodeId>,
}

/// Routing style for edge drawing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoutingStyle {
    Straight,     // straight line, 2 points only
    Orthogonal,   // right-angle polyline through dummy midpoints
    #[default]
    Bezier,       // smooth cubic bezier through dummy waypoints
}

/// Route for a single edge.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct EdgeRoute {
    pub source: NodeId,
    pub target: NodeId,
    pub reversed: bool,
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

