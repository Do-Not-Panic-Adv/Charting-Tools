use robotics_lib::{
    interface::{discover_tiles, Direction},
    runner::Runnable,
    utils::LibError,
    world::{coordinates::Coordinate, World},
};

use crate::charted_coordinate::ChartedCoordinate;

pub struct ChartedBot {
    width: u32,
    length: u32,
    coordinates: ChartedCoordinate,
}

impl ChartedBot {
    pub fn new(length: u32, coordinates: ChartedCoordinate) -> Self {
        ChartedBot {
            width: 3,
            length,
            coordinates,
        }
    }

    pub fn discover(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        dir: Direction,
    ) -> Result<(), LibError> {
        //Calculates cost for the discovery

        let to_visit = match dir {
            Direction::Up => {}
            Direction::Down => {
                let x: usize;
                let y: usize;
                println!("col: {}", self.coordinates.get_col());
                if self.coordinates.get_col() < 1 {
                    return Err(LibError::OutOfBounds);
                }
                for x in self.coordinates.get_col() - 1..=self.coordinates.get_col() + 1 {
                    for y in self.coordinates.get_row()
                        ..self.coordinates.get_row() + self.length as usize
                    {
                        println!("Tile to discover: ({x},{y})");
                    }
                }
            }
            Direction::Left => todo!(),
            Direction::Right => todo!(),
        };

        if robot
            .get_energy()
            .has_enough_energy((self.length * self.width * 3) as usize)
        {
            let res = discover_tiles(robot, world, &vec![(5, 5)]);
            Ok(())
        } else {
            Err(LibError::NotEnoughEnergy)
        }
    }
}
