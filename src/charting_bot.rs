use std::usize;

use robotics_lib::{
    interface::{discover_tiles, robot_map, Direction},
    runner::Runnable,
    utils::LibError,
    world::World,
};

use crate::charted_coordinate::ChartedCoordinate;
use crate::{reserved::New, ChartingTool, NUMBER};

#[derive(Debug, Clone)]
pub struct ChartingBot {
    coordinates: ChartedCoordinate,
}

impl Drop for ChartingBot {
    fn drop(&mut self) {
        if let Ok(mut n) = NUMBER.lock() {
            if *n > 0 {
                *n = *n - 1;
            }
        }
    }
}

impl ChartingTool for ChartingBot {}
impl New for ChartingBot {
    fn new() -> Self {
        ChartingBot {
            coordinates: ChartedCoordinate(0, 0),
        }
    }
}

impl ChartingBot {
    /// # Initializes the ChartingBot
    /// The starting position of the bot is set to the position of the Robot.
    pub fn init(&mut self, robot: &impl Runnable) {
        self.coordinates = ChartedCoordinate::from(robot.get_coordinate());
        //println!("DiscoveryBot placed in {:?}", self.coordinates)
    }

    /// # Performs a line discovery
    /// Starts to discover the map following the given direction
    ///
    /// # Parameters
    /// - robot: A mutable reference to the robot whose personal map has to be discovered.
    /// - world: A mutable reference to the world.
    /// - lenght: The lenght of the strip of tiles to be discovered.
    /// - width: The width of the strip of tiles to discovered (should an odd number).
    /// - direction: The direction in wich the Bot will be heading.
    ///
    /// # Errors
    /// This function will return an error if during the discovery of the maps,
    /// the maximum ammount of discoverable tiles is reached  (`LibError::NoMoreDiscoverable`) or the robot does
    /// not have enough energy to complete the discovery (`LibError::NotEnoughEnergy`).
    ///
    /// # Returns
    /// - The number of discovered tiles or an error.
    ///
    /// ## Notes
    /// - Tiles that are already present in the robots map will not be checked.
    /// - Using an even number for the width value will result in a strip as wide as the next odd
    /// number to one inserted.

    pub fn discover_line(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        length: usize,
        width: usize,
        direction: Direction,
    ) -> Result<usize, LibError> {
        let world_dim = robot_map(world).unwrap()[0].len();

        let to_visit = match direction {
            | Direction::Up => {
                let iter_x;
                if self.coordinates.get_col() < width / 2 {
                    iter_x = 0..=self.coordinates.get_col() + width / 2;
                } else {
                    if self.coordinates.get_col() + width / 2 > world_dim {
                        iter_x = self.coordinates.get_col() - width / 2..=world_dim - 1;
                    } else {
                        iter_x = self.coordinates.get_col() - width / 2..=self.coordinates.get_col() + width / 2;
                    }
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
                if self.coordinates.get_col() < width / 2 {
                    inter_x = 0..=self.coordinates.get_col() + width / 2;
                } else {
                    if self.coordinates.get_col() + width / 2 >= world_dim {
                        inter_x = self.coordinates.get_col() - width / 2..=world_dim - 1;
                    } else {
                        inter_x = self.coordinates.get_col() - width / 2..=self.coordinates.get_col() + width / 2;
                    }
                }
                if (self.coordinates.get_row() + length as usize - 1usize) >= world_dim {
                    inter_y = self.coordinates.get_row()..=world_dim - 1
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

                if self.coordinates.get_row() < width / 2 {
                    inter_y = (0..=self.coordinates.get_row() + width / 2).rev();
                } else {
                    if self.coordinates.get_row() + width / 2 >= world_dim {
                        inter_y = (self.coordinates.get_row() - width / 2..=world_dim - 1).rev();
                    } else {
                        inter_y =
                            (self.coordinates.get_row() - width / 2..=self.coordinates.get_row() + width / 2).rev();
                    }
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

                if self.coordinates.get_row() < width / 2 {
                    int_y = (0..=self.coordinates.get_row() + width / 2).rev();
                } else {
                    if self.coordinates.get_row() + width / 2 >= world_dim {
                        int_y = (self.coordinates.get_row() - width / 2..=world_dim).rev();
                    } else {
                        int_y = (self.coordinates.get_row() - width / 2..=self.coordinates.get_row() + width / 2).rev();
                    }
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

        let mut discovered: usize = 0;

        for t in to_visit {
            match Self::check_discovered(world, t) {
                | Ok(_) => match discover_tiles(robot, world, &[t]) {
                    | Ok(_) => discovered += 1,
                    | Err(e) => return Err(e),
                },
                | Err(e) => return Err(e),
            }
        }
        Ok(discovered)
    }

    /// Checks if a tile in a given coordinale is already present in the robots personal map.
    pub(crate) fn check_discovered(world: &World, coordinate: (usize, usize)) -> Result<bool, LibError> {
        if coordinate.0 < robot_map(world).unwrap().len() && coordinate.1 < robot_map(world).unwrap()[0].len() {
            match &robot_map(world).unwrap()[coordinate.0][coordinate.1] {
                | Some(_) => Ok(false),
                | None => Ok(false),
            }
        } else {
            Err(LibError::OutOfBounds)
        }
    }

    /// # Performs a path discovery
    /// Starts to discover the map following a given set of directions
    ///
    /// # Parameters
    /// - robot: A mutable reference to the robot whose personal map has to be discovered.
    /// - world: A mutable reference to the world.
    /// - width: The width of the strip of tiles to discovered (should an odd number).
    /// - path: A list of directions that the Bot will follow.
    ///
    /// # Errors
    /// This function will return an error if during the discovery of the maps,
    /// the maximum ammount of discoverable tiles is reached  (`LibError::NoMoreDiscoverable`) or the robot does
    /// not have enough energy to complete the discovery (`LibError::NotEnoughEnergy`).
    ///
    /// # Returns
    /// - The number of discovered tiles or an error.
    ///
    /// ## Notes
    /// - Tiles that are already present in the robots map will not be checked.
    /// - Using an even number for the width value will result in a strip as wide as the next odd
    /// number to one inserted.

    pub fn discover_path(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        width: usize,
        path: Vec<Direction>,
    ) -> Result<usize, LibError> {
        let mut discovered: usize = 0;
        for d in path {
            Self::move_bot(self, &d);
            match Self::discover_line(self, robot, world, 1, width, d) {
                | Ok(_) => discovered += 1,
                | Err(e) => return Err(e),
            }
        }
        Ok(discovered)
    }

    //Alters the position of the carting bot given the movements direction.
    pub(crate) fn move_bot(&mut self, direction: &Direction) {
        match direction {
            | Direction::Up => self.coordinates.0 -= 1,
            | Direction::Down => self.coordinates.0 += 1,
            | Direction::Left => self.coordinates.1 -= 1,
            | Direction::Right => self.coordinates.1 += 1,
        }
        // println!("DiscoveryBot moved to: {:?}", self.coordinates)
    }
}
