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
            Direction::Up => {
                let mut tiles: Vec<(usize, usize)> = vec![];
                tiles
            }
            Direction::Down => {
                if self.coordinates.get_col() < 1 {
                    return Err(LibError::OutOfBounds);
                }
                let mut tiles: Vec<(usize, usize)> = vec![];
                for y in
                    self.coordinates.get_row()..self.coordinates.get_row() + self.length as usize
                {
                    for x in self.coordinates.get_col() - 1..=self.coordinates.get_col() + 1 {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
            Direction::Left => todo!(),
            Direction::Right => todo!(),
        };

        if robot
            .get_energy()
            .has_enough_energy((self.length * self.width * 3) as usize)
        {
            //let res = discover_tiles(robot, world, &vec![(4, 5)]);

            for t in to_visit {
                print!("discovering: {:?}", t);
                println!(" discoveries left: {}", world.get_discoverable());
                let _ = discover_tiles(robot, world, &[t]);
            }

            Ok(())
        } else {
            Err(LibError::NotEnoughEnergy)
        }
    }
}
