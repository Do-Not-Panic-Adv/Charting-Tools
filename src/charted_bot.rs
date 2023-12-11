use std::usize;

use robotics_lib::{
    interface::{discover_tiles, robot_map, Direction},
    runner::Runnable,
    utils::LibError,
    world::World,
};

use crate::charted_coordinate::ChartedCoordinate;
use crate::{hidden::New, ChartingTool, NUMBER};

#[derive(Debug, Clone)]
pub struct ChartedBot {
    coordinates: ChartedCoordinate,
}

impl Drop for ChartedBot {
    fn drop(&mut self) {
        if let Ok(mut n) = NUMBER.lock() {
            if *n > 0 {
                *n = *n - 1;
            }
        }
    }
}

impl ChartingTool for ChartedBot {}

impl New for ChartedBot {
    fn new() -> Self {
        ChartedBot {
            coordinates: ChartedCoordinate(0, 0),
        }
    }
}

impl ChartedBot {
    pub fn init(&mut self, coordinates: ChartedCoordinate) {
        self.coordinates = coordinates;
    }
    pub fn discover_line(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        length: usize,
        width: usize,
        direction: Direction,
    ) -> Result<i32, LibError> {
        //Calculates cost for the discovery

        let to_visit = match direction {
            | Direction::Up => {
                let iter_x;
                if self.coordinates.get_col() < 1 {
                    iter_x = 0..=self.coordinates.get_col() + 1
                } else {
                    iter_x = self.coordinates.get_col() - 1..=self.coordinates.get_col() + 1;
                }

                let iter_y;
                let mut tiles: Vec<(usize, usize)> = vec![];

                if (self.coordinates.get_row() as i32 - length as i32) < 0 {
                    iter_y = (0..=self.coordinates.get_row()).rev();
                } else {
                    iter_y = ((self.coordinates.get_row() - length as usize + 1)..=self.coordinates.get_row()).rev();
                }

                for y in iter_y {
                    for x in iter_x.clone() {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
            | Direction::Down => {
                let inter_x;
                let inter_y;
                if self.coordinates.get_col() < 1 {
                    inter_x = 0..=self.coordinates.get_col() + 1
                } else {
                    inter_x = self.coordinates.get_col() - 1..=self.coordinates.get_col() + 1;
                }
                if (self.coordinates.get_row() + length as usize - 1usize) >= robot_map(world).unwrap()[0].len() {
                    inter_y = self.coordinates.get_row()..=robot_map(world).unwrap()[0].len() - 1
                } else {
                    inter_y = self.coordinates.get_row()..=self.coordinates.get_row() + length as usize - 1usize
                }

                let mut tiles: Vec<(usize, usize)> = vec![];
                for y in inter_y {
                    for x in inter_x.clone() {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
            | Direction::Right => {
                let inter_y;
                let inter_x;

                if self.coordinates.get_row() < 1 {
                    inter_y = (0..=self.coordinates.get_row() + 1).rev();
                } else {
                    inter_y = (self.coordinates.get_row() - 1..=self.coordinates.get_row() + 1).rev();
                }

                if (self.coordinates.get_col() + length as usize - 1usize) >= robot_map(world).unwrap().len() {
                    inter_x = self.coordinates.get_col()..=robot_map(world).unwrap().len() - 1
                } else {
                    inter_x = self.coordinates.get_col()..=self.coordinates.get_col() + length as usize - 1usize
                }

                let mut tiles: Vec<(usize, usize)> = vec![];
                for x in inter_x {
                    for y in inter_y.clone() {
                        tiles.push((y, x))
                    }
                }
                tiles
            }
            | Direction::Left => {
                let int_y;
                let int_x;

                if self.coordinates.get_row() < 1 {
                    int_y = (0..=self.coordinates.get_row() + 1).rev();
                } else {
                    int_y = (self.coordinates.get_row() - 1..=self.coordinates.get_row() + 1).rev();
                }

                if (self.coordinates.get_col() as i32 - length as i32) < 0 {
                    int_x = (0..=self.coordinates.get_col()).rev();
                } else {
                    int_x = (self.coordinates.get_col() - length as usize + 1..=self.coordinates.get_col()).rev();
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
        if robot.get_energy().has_enough_energy((length * width * 3) as usize) {
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
            | Some(_) => true,
            | None => false,
        }
    }

    pub fn discover_path(&mut self, robot: &mut impl Runnable, world: &mut World, width: usize, path: Vec<Direction>) {
        for d in path {
            Self::move_bot(self, &d);
            let _ = Self::discover_line(self, robot, world, 1, width, d);
        }
    }
    pub(crate) fn move_bot(&mut self, direction: &Direction) {
        match direction {
            | Direction::Up => self.coordinates.0 -= 1,
            | Direction::Down => self.coordinates.0 += 1,
            | Direction::Left => self.coordinates.1 -= 1,
            | Direction::Right => self.coordinates.1 += 1,
        }
        println!("DiscoveryBot moved to: {:?}", self.coordinates)
    }
}
