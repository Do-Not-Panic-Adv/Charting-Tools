use std::fmt::Debug;
use std::sync::Mutex;

use robotics_lib::interface::Tools;

pub mod charted_bot;
pub mod charted_coordinate;
pub mod charted_map;
pub mod charted_paths;
pub mod charted_world;

const LIMIT: u8 = 3;
lazy_static::lazy_static! {
    static ref NUMBER: Mutex<u8> = Mutex::new(0 as u8);
}

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
    pub fn tool<T: ChartingTool>() -> Result<T, u8> {
        if let Ok(mut n) = NUMBER.lock() {
            if *n < LIMIT {
                *n = *n + 1;
                return Ok(T::new());
            } else {
                Err(n.clone())
            }
        } else {
            Err(0)
        }
    }
}

/// # Trait: ChartingTool
/// it is an internal trait that defines what can be used by ChartingTools::tool
pub trait ChartingTool: Debug + Drop + hidden::New {}

pub(crate) mod hidden {
    pub trait New {
        fn new() -> Self;
    }
}
