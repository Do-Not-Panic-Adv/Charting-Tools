use robotics_lib::{
    energy::Energy,
    interface::{discover_tiles, Direction},
    runner::Runnable,
    world::World,
};

use crate::charted_coordinate::ChartedCoordinate;

pub struct ChartedBot {
    energy: Energy,
    coordinates: ChartedCoordinate,
}

impl ChartedBot {
    pub fn new(energy: Energy, coordinates: ChartedCoordinate) -> Self {
        ChartedBot {
            energy,
            coordinates,
        }
    }

    pub fn discover(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        start_pos: ChartedCoordinate,
        dir: Direction,
    ) {
        //let res = discover_tiles(robot, world, &vec![(0usize, 1usize)]);
    }
}
