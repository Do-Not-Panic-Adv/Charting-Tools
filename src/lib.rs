use robotics_lib::interface::Tools;

mod charted_bot;
mod charted_coordinate;
mod charted_map;
mod charted_world;
mod graph;

/// # Tool: Charting tools
/// contains a selection of utilities that are useful for navigation around the world
///
/// ## Usage
/// you can instantiate one of the following utilities
///
/// - ChartedWorld
///     Vec-based map of discovered tiles
/// - ChartedMap
///     used to save points of interest and retrieve them later
/// - ChartedPaths
///     pathfinding algorithms (god help us all)
/// - ChartingBot
///     a way to discover new tiles using energy
/// - ChartingSomethingElse idk (if we have time)
///
/// by calling
///
///     ChartingTools::tool::<utility_name_here>();
///
/// The function will return the requested struct to be used in your code
///
/// ## Examples
///
///     let mut cm = ChartingTools::tool::<ChartedMap>();
pub struct ChartingTools;

impl Tools for ChartingTools {}

impl ChartingTools {
    pub fn tool<T>() -> T
    where
        T: ChartingTool,
    {
        T::new()
    }
}

/// # Trait: ChartingTool
/// it is an internal trait that defines what can be used by ChartingTools::tool
///
pub trait ChartingTool {
    fn new() -> Self;
}
