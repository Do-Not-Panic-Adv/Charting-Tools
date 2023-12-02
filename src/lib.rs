mod charted_map;
mod charted_coordinate;
mod graph;

use robotics_lib::interface::Tools;

/// # Tool: Charting tools
/// contains a selection of utilities that are useful for navigation around the world
///
/// ## Usage
/// you can instantiate one of the following utilities
///
/// - ChartedMap
///     used to save points of interest and retrieve them later
/// - ChartedPath?
///     pathfinding algorithms (god help us all)
/// - ChartedWorld?
///     matrix of discovered tiles
/// - ChartingBot?
///     a way to discover new tiles using energy
/// - ChartingSomethingElse idk (if we have time)
///
/// by calling
///
///     ChartingTools::new::<utility_name_here>();
///
/// The function will return the requested struct to be used in your code
///
/// ## Examples
///
///     let mut cm = ChartingTools::new::<ChartedMap>();
pub struct ChartingTools;

impl Tools for ChartingTools {}

impl ChartingTools {
    pub fn new<T>() -> T where T: ChartingTool {
        T::new()
    }
}

/// # Trait: ChartingTool
/// it is an internal trait that defines what can be used by ChartingTools::new
///
pub trait ChartingTool {
    fn new() -> Self;
}
