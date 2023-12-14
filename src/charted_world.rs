use std::collections::HashMap;

use robotics_lib::interface::{discover_tiles, robot_map, robot_view};
use robotics_lib::runner::Runnable;
use robotics_lib::utils::LibError;
use robotics_lib::world::tile::Tile;
use robotics_lib::world::World;

use crate::{ChartingTool, NUMBER, reserved::New};
use crate::charted_coordinate::ChartedCoordinate;

/// struct: ChartedWorld
///
/// fairly simple implementation of a custom map for the world,
/// it contains functions to save tiles at specific coordinates
#[derive(Debug, Clone)]
pub struct ChartedWorld {
    map: Vec<Vec<Option<Tile>>>,
    len: usize,
}

impl Drop for ChartedWorld {
    fn drop(&mut self) {
        if let Ok(mut n) = NUMBER.lock() {
            if *n > 0 {
                *n = *n - 1;
            }
        }
    }
}

impl ChartingTool for ChartedWorld {}

impl New for ChartedWorld {
    fn new() -> Self {
        Self { map: Vec::default(), len: 0 }
    }
}

impl ChartedWorld {
    /// clears the map completely, setting all tiles to None
    pub fn clear(&mut self) {
        for row in self.map.iter_mut() {
            for tile in row.iter_mut() {
                *tile = None;
            }
        }
    }

    /// initializes the map to the one currently obtainable from the world via `robot_map()`
    pub fn init(&mut self, world: &World) -> Result<(), &str> {
        match robot_map(world) {
            | None => Err("This literally should not be able to happen..."),
            | Some(map) => {
                self.len = map.len();
                self.map = map;
                Ok(())
            }
        }
    }

    fn check_bounds(&self, coordinate: ChartedCoordinate) -> bool {
        coordinate < self.len
    }

    /// returns the tile at the specified coordinate. It returns
    /// - `LibError::OutOfBounds` if the coordinate exceeds the world dimensions
    /// - `None` if the desired tile has not yet been discovered or set
    /// - the tile at the provided coordinate otherwise
    pub fn at(&self, coordinate: ChartedCoordinate) -> Result<Option<Tile>, LibError> {
        if !self.check_bounds(coordinate) { return Err(LibError::OutOfBounds); }
        Ok(self.map[coordinate.0][coordinate.1].clone())
    }

    /// returns the whole map currently saved in the data structure
    pub fn get_map(&self) -> &Vec<Vec<Option<Tile>>> {
        &self.map
    }

    /// sets the tile at the specified coordinate to the specified Tile.
    ///
    /// it will fail if the tile at said position has already been set or discovered (ie it is Some),
    /// or if the coordinates are invalid
    pub fn set(&mut self, tile: &Tile, coordinate: ChartedCoordinate) -> Result<(), (LibError, Option<Tile>)> {
        if !self.check_bounds(coordinate) { return Err((LibError::OutOfBounds, None)); }
        match self.at(coordinate) {
            Ok(None) => {
                self.map[coordinate.0][coordinate.1] = Some(tile.clone());
                Ok(())
            },
            Ok(Some(old_tile)) => Err((LibError::OperationNotAllowed, Some(old_tile.clone()))),
            Err(err) => Err((err, None))
        }
    }

    /// sets the tile at the specified coordinate to the specified Tile.
    ///
    /// it will **not** fail if the tile at said position has already been set or discovered (ie it is Some),
    /// but it will if the coordinates are invalid
    pub fn set_overwrite(&mut self, tile: &Tile, coordinate: ChartedCoordinate) -> Result<(), LibError> {
        if !self.check_bounds(coordinate) { return Err(LibError::OutOfBounds); }
        self.map[coordinate.0][coordinate.1] = Some(tile.clone());
        Ok(())
    }

    /// sets the tile at the specified coordinate to the specified Tile.
    ///
    /// it will fail at the first coordinate at which a tile has already been set or discovered (ie it is Some),
    /// or at the first invalid coordinate
    pub fn set_multiple(
        &mut self,
        to_change: &Vec<(&Tile, ChartedCoordinate)>,
    ) -> Result<(), (LibError, Option<Tile>, ChartedCoordinate)> {
        for i in to_change {
            match self.set(i.0, i.1) {
                | Ok(()) => {}
                | Err((err, option)) => {
                    return Err((err, option, i.1));
                }
            }
        }
        Ok(())
    }

    /// these names are getting out of hand...
    /// sets the tile at the specified coordinate to the specified Tile.
    ///
    /// it will only fail at the first invalid coordinate
    pub fn set_multiple_overwrite(&mut self, to_change: &Vec<(Tile, ChartedCoordinate)>) -> Result<(), (LibError, ChartedCoordinate)> {
        for (tile, coordinate) in to_change.iter() {
            match self.set_overwrite(tile, *coordinate) {
                Ok(_) => {}
                Err(err) => { return Err((err, *coordinate)); }
            }
        }
        Ok(())
    }

    /// updates the tile at the specified coordinate to the one currently discovered in the world, be it Some(Tile) or None
    pub fn update(&mut self, world: &World, coordinates: &Vec<ChartedCoordinate>) -> Result<(), (LibError, ChartedCoordinate)> {
        let option = robot_map(world);
        if option.is_none() {
            return Err((LibError::OutOfBounds, ChartedCoordinate::default()));
        }

        let map = option.unwrap();
        for point in coordinates.iter() {
            if !self.check_bounds(*point) {
                return Err((LibError::OutOfBounds, *point));
            }
            if map[point.0][point.1].is_some() && self.map[point.0][point.1] != map[point.0][point.1] {
                self.map[point.0][point.1] = map[point.0][point.1].clone();
            } else if self.map[point.0][point.1].is_some() && map[point.0][point.1].is_none() {
                self.map[point.0][point.1] = None;
            }
        }
        Ok(())
    }

    /// it will update all tiles that are currently visible to the robot according to the `robot_map` interface
    pub fn update_viewed(&mut self, robot: &impl Runnable, world: &World) {
        let view = robot_view(robot, world);
        let robot_coordinate = ChartedCoordinate::from(robot.get_coordinate());
        let conversion_coordinate = robot_coordinate - (1, 1);

        for i in 0..view.len() {
            for j in 0..view.len() {
                if view[i][j].is_some() {
                    self.map[i + conversion_coordinate.0][j + conversion_coordinate.1] = view[i][j].clone();
                }
            }
        }
    }

    /// it will update all tiles at the provided coordinates by discovering them using the `discover_tiles` interface.
    ///
    /// It will return the same Result of said interface, at the same conditions, so reading the documentation for `discover_tiles` is suggested
    pub fn update_discover(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        to_discover: &Vec<ChartedCoordinate>,
    ) -> Result<HashMap<ChartedCoordinate, Option<Tile>>, LibError> {
        return match discover_tiles(
            robot,
            world,
            &to_discover.iter().map(|c| (c.0, c.1)).collect::<Vec<(usize, usize)>>(),
        ) {
            | Ok(hm) => {
                for ((x, y), tile) in hm.iter() {
                    self.map[*x][*y] = tile.clone();
                }
                Ok(hm
                    .iter()
                    .map(
                        |(tuple, option)|
                            (ChartedCoordinate::from(*tuple), option.clone()))
                    .collect())
            }
            | Err(err) => Err(err),
        };
    }

    /// resets the map to be exactly like the one available via `robot_map`
    ///
    /// it is advisable to call this function at the end of the game loop to reset any changes made, as well as to update the map to
    /// any new discoveries made in the past tick
    pub fn update_overwrite(&mut self, world: &World) -> Result<(), LibError> {
        let map = robot_map(world);
        if map.is_some() {
            let map = map.unwrap();
            for (i, row) in map.iter().enumerate() {
                for (j, tile) in row.iter().enumerate() {
                    if tile.is_some() && self.map[i][j] != map[i][j] {
                        self.map[i][j] = map[i][j].clone();
                    } else if self.map[i][j].is_some() && tile.is_none() {
                        self.map[i][j] = None;
                    }
                }
            }
            Ok(())
        } else {
            Err(LibError::OutOfBounds)
        }
    }
}
