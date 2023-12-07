use robotics_lib::{
    interface::{discover_tiles, Direction},
    runner::Runnable,
    world::World,
};

use crate::{
    charted_coordinate::ChartedCoordinate,
    charted_map::{ChartedMap, MapKey},
};

pub struct ChartedBot {
    energy: u32,
}

impl ChartedBot {
    pub fn new(energy: u32) -> Self {
        ChartedBot { energy }
    }

    fn discover(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        start_pos: ChartedCoordinate,
        dir: Direction,
    ) {
        discover_tiles(robot, world, &vec![(0usize, 1usize)]);
    }
}
