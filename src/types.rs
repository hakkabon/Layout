//! Shared types for the layout engine.
pub type NodeId = usize;
pub type RankId = usize;
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NodeType {
    Normal,
    Dummy,
}

#[derive(Debug, Clone)]
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

#[derive(Debug, Clone)]
pub struct LayoutEdge {
    pub from: NodeId,
    pub to: NodeId,
    /// True if this edge was reversed by [`break_cycles`] and should be
    /// drawn with its arrowhead at the visually higher end.
    pub reversed: bool,
}

#[derive(Default, Debug, Clone)]
pub struct LayoutGraph {
    pub nodes: Vec<LayoutNode>,
    pub edges: Vec<LayoutEdge>,
}

#[derive(Default, Debug, Clone)]
pub struct RankSystem {
    pub layers: Vec<Vec<NodeId>>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LayoutError {
    /// The input graph has a cycle. Cycle breaking (reversing back-edges)
    /// is its responsibility and must happen before ranking; this
    /// error means that step was skipped or the graph was mutated after it.
    CyclicGraph,
}

/// Tracks how a single original (possibly long) edge was decomposed into a
/// chain of single-rank segments by Rank Assignment.
#[derive(Debug, Clone)]
pub struct EdgeChain {
    pub source: NodeId,   // original edge source
    pub target: NodeId,   // original edge target
    pub reversed: bool,   // mirrored from the LayoutEdge
    /// All waypoint nodes in order: [source, dummy₁, dummy₂, …, target].
    pub chain: Vec<NodeId>,
}

/// Routing style for edge drawing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RoutingStyle {
    Straight,     // straight line, 2 points only
    Orthogonal,   // right-angle polyline through dummy midpoints
    Bezier,       // smooth cubic bezier through dummy waypoints
}

/// Route for a single edge.
#[derive(Debug, Clone)]
pub struct EdgeRoute {
    pub source: NodeId,
    pub target: NodeId,
    pub reversed: bool,
    /// Waypoints from source to target.
    /// - Straight:    exactly 2 points (source center, target center).
    /// - Orthogonal:  polyline through dummy-node midpoints (≥ 2 points).
    /// - Bezier:      cubic bezier quadruples packed as (P0, C1, C2, P1, …)
    ///                starting at source center and ending at target center.
    pub waypoints: Vec<(f32, f32)>,
}

/// Configuration for coordinate assignment.
///
/// This is a type alias for `CoordConfig` for backward compatibility.
/// New code should use `CoordConfig` directly which supports both
/// the median relaxation and Brandes-Köpf algorithms.
#[deprecated(since = "2.0.0", note = "Use CoordConfig instead, which supports Brandes-Köpf algorithm")]
pub type CoordinateConfig = crate::coordinates::CoordConfig;
/// Re-export CoordConfig from coordinatesfor direct access.
pub use crate::coordinates::{CoordConfig, CoordAlgorithm};
// Re-export LayoutPipeline for backward compatibility
pub use crate::ranks::LayoutPipeline;
