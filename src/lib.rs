//! Layout engine modules
pub mod types;
pub mod validate;
pub mod cycles;
pub mod ranks;
pub mod dummy_nodes;
pub mod crossings;
pub mod coordinates;
pub use types::*;
pub use validate::validate_graph;
pub use cycles::break_cycles;
pub use ranks::assign_ranks;
pub use dummy_nodes::insert_dummy_nodes;
pub use crossings::{order_layers, count_total_crossings};
pub use coordinates::{assign_coordinates, route_edges, CoordConfig, CoordAlgorithm};
