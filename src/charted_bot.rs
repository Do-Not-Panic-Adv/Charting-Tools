use std::usize;

use robotics_lib::{
    interface::{discover_tiles, robot_map, Direction},
    runner::Runnable,
    utils::LibError,
    world::{coordinates::Coordinate, tile::Tile, World},
};

use crate::charted_coordinate::ChartedCoordinate;

pub struct ChartedBot {
    width: u32,
    length: u32,
    coordinates: ChartedCoordinate,
    direction: Direction,
}

impl ChartedBot {
    pub fn new(length: u32, coordinates: ChartedCoordinate, direction: Direction) -> Self {
        ChartedBot {
            width: 3,
            length,
            coordinates,
            direction,
        }
    }

    pub fn discover_line(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
    ) -> Result<i32, LibError> {
        //Calculates cost for the discovery

        let to_visit = match self.direction {
            Direction::Up => {
                if self.coordinates.get_col() < 1 {
                    return Err(LibError::OutOfBounds);
                }

                let int;
                let mut tiles: Vec<(usize, usize)> = vec![];

                if (self.coordinates.get_row() as i32 - self.length as i32) < 0 {
                    int = (0..=self.coordinates.get_row()).rev();
                } else {
                    int = ((self.coordinates.get_row() - self.length as usize + 1)
                        ..=self.coordinates.get_row())
                        .rev();
                }

                for y in int {
                    for x in self.coordinates.get_col() - 1..=self.coordinates.get_col() + 1 {
                        println!("pushing {:?}", (y, x));
                        tiles.push((y, x))
                    }
                }
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
            Direction::Right => {
                let int;
                if self.coordinates.get_row() < 1 {
                    int = (0..=self.coordinates.get_row() + 1).rev();
                } else {
                    int = (self.coordinates.get_row() - 1..=self.coordinates.get_row() + 1).rev();
                }
                let mut tiles: Vec<(usize, usize)> = vec![];
                for x in self.coordinates.get_col()
                    ..=self.coordinates.get_col() + self.length as usize - 1usize
                {
                    for y in int.clone() {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
            Direction::Left => {
                let int_y;
                let int_x;

                if self.coordinates.get_row() < 1 {
                    int_y = (0..=self.coordinates.get_row() + 1).rev();
                } else {
                    int_y = (self.coordinates.get_row() - 1..=self.coordinates.get_row() + 1).rev();
                }

                if (self.coordinates.get_col() as i32 - self.length as i32) < 0 {
                    int_x = (0..=self.coordinates.get_col()).rev();
                } else {
                    int_x = (self.coordinates.get_col() - self.length as usize + 1
                        ..=self.coordinates.get_col())
                        .rev();
                }

                let mut tiles: Vec<(usize, usize)> = vec![];
                for x in int_x {
                    for y in int_y.clone() {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
        };

        let mut discovered = 0;
        if robot
            .get_energy()
            .has_enough_energy((self.length * self.width * 3) as usize)
        {
            for t in to_visit {
                if world.get_discoverable() <= 0 {
                    return Err(LibError::NoMoreDiscovery);
                }
                //println!("tile: {:?} in {:?}", robot_map(world).unwrap()[t.0][t.1], t);
                if !Self::check_discovered(world, t) {
                    let _ = discover_tiles(robot, world, &[t]);
                    discovered += 1;
                }
            }

            Ok(discovered)
        } else {
            Err(LibError::NotEnoughEnergy)
        }
    }

    /// .
    ///
    /// # Panics
    ///
    /// Panics if .
    pub(crate) fn check_discovered(world: &World, coordinate: (usize, usize)) -> bool {
        match &robot_map(world).unwrap()[coordinate.0][coordinate.1] {
            Some(t) => {
                println!("tile exists: {:?}", t);
                true
            }
            None => false,
        }
    }
}
