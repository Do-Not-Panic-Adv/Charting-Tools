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
/// contains a selection of utilities that are useful for navigation around the world and discovery
///
/// ## Usage
/// you can instantiate one of the following utilities
///
/// - ChartedWorld
///
///     editable vec-based map of discovered tiles
/// - ChartedMap
///
///     used to save points of interest and retrieve them later
/// - ChartedPaths
///
///     pathfinding algorithms
/// - ChartedBot
///
///     a way to discover new tiles using energy
///
/// by calling
///
/// ```
///     charting_tools::ChartingTools::tool::</*utility_name_here*/>()
///                 .expect("too many tools used!");
/// ```
///
/// The function will return the requested struct to be used in your code
///
/// ## Examples
/// ```
/// use robotics_lib::world::tile::Content;
/// use charting_tools::charted_map::ChartedMap;
/// use charting_tools::ChartingTools;
///
/// let mut result = ChartingTools::tool::<ChartedMap<Content>>();
/// match result {
///     Ok(_) => {}
///     Err(_) => {}
/// }
/// ```
pub struct ChartingTools;

impl Tools for ChartingTools {}

impl ChartingTools {
    /// # Constructor
    /// if called, it will return the desired ChartingTool,
    /// while checking that you have not reached the maximum allowed
    /// number
    ///
    /// ## Example
    /// ```
    /// use charting_tools::charted_world::ChartedWorld;
    /// use charting_tools::ChartingTools;
    ///
    /// let result = ChartingTools::tool::<ChartedWorld>();
    /// match result {
    ///     Ok(world) => println!("got a ChartedWorld"),
    ///     Err(num) => eprintln!("there are currently {num} tools active, you cannot get another")
    /// }
    /// ```
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
/// implemented for all 4 sub-tools,
/// it is a trait that defines what can be used by ChartingTools::tool
///
pub trait ChartingTool: Debug + Drop + hidden::New {}

pub(crate) mod hidden {
    pub trait New {
        /// # Do not use
        /// Internal function, only intended for use inside the tool crate
        ///
        /// **It will throw a compile-time error if used inside your code**
        fn new() -> Self;
    }
}
