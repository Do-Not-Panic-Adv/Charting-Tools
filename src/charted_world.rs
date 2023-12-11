use std::collections::HashMap;

use robotics_lib::interface::{discover_tiles, robot_map, robot_view};
use robotics_lib::runner::Runnable;
use robotics_lib::utils::LibError;
use robotics_lib::world::tile::Tile;
use robotics_lib::world::World;

use crate::charted_coordinate::ChartedCoordinate;
use crate::{ChartingTool, hidden::New};

#[derive(Debug, Clone)]
pub struct ChartedWorld {
    map: Vec<Vec<Option<Tile>>>,
}

impl ChartingTool for ChartedWorld {}

impl New for ChartedWorld {
    fn new() -> Self {
        Self { map: Vec::default() }
    }
}

impl ChartedWorld {
    pub fn clear(&mut self) {
        self.map.clear()
    }
    pub fn init(&mut self, world: &World) -> Result<(), &str> {
        match robot_map(world) {
            | None => Err("This literally should not be able to happen..."),
            | Some(map) => {
                self.map = map;
                Ok(())
            }
        }
    }

    pub fn at(&self, coordinate: ChartedCoordinate) -> Option<Tile> {
        self.map[coordinate.0][coordinate.1].clone()
    }

    pub fn get_map(&self) -> &Vec<Vec<Option<Tile>>> {
        &self.map
    }

    pub fn set(&mut self, tile: &Tile, coordinate: ChartedCoordinate) -> Result<(), Tile> {
        match self.at(coordinate) {
            | None => {
                self.map[coordinate.0][coordinate.1] = Some(tile.clone());
                Ok(())
            }
            | Some(old_tile) => Err(old_tile.clone()),
        }
    }

    pub fn set_unchecked(&mut self, tile: &Tile, coordinate: ChartedCoordinate) {
        self.map[coordinate.0][coordinate.1] = Some(tile.clone());
    }

    pub fn set_multiple(
        &mut self,
        to_change: &Vec<(&Tile, ChartedCoordinate)>,
    ) -> Result<(), (Tile, ChartedCoordinate)> {
        for i in to_change {
            match self.set(i.0, i.1) {
                | Ok(_) => {}
                | Err(_) => {
                    return Err((i.0.clone(), i.1));
                }
            }
        }
        Ok(())
    }

    // these names are getting out of hand...
    pub fn set_multiple_unchecked(&mut self, to_change: &Vec<(&Tile, ChartedCoordinate)>) {
        for i in to_change {
            self.set_unchecked(i.0, i.1);
        }
    }

    pub fn update(&mut self, world: &World, coordinates: &Vec<ChartedCoordinate>) {
        let option = robot_map(world);
        if option.is_none() {
            return;
        }

        let map = option.unwrap();
        for point in coordinates.iter() {
            if map[point.0][point.1].is_some() && self.map[point.0][point.1] != map[point.0][point.1] {
                self.map[point.0][point.1] = map[point.0][point.1].clone();
            } else if self.map[point.0][point.1].is_some() && map[point.0][point.1].is_none() {
                self.map[point.0][point.1] = None;
            }
        }
    }

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

    pub fn update_discover(
        &mut self,
        robot: &mut impl Runnable,
        world: &mut World,
        to_discover: &Vec<ChartedCoordinate>,
    ) -> Result<HashMap<(usize, usize), Option<Tile>>, LibError> {
        return match discover_tiles(
            robot,
            world,
            &to_discover.iter().map(|c| (c.0, c.1)).collect::<Vec<(usize, usize)>>(),
        ) {
            | Ok(hm) => {
                for ((x, y), tile) in hm.iter() {
                    self.map[*x][*y] = tile.clone();
                }
                Ok(hm)
            }
            | Err(err) => Err(err),
        };
    }

    pub fn update_all(&mut self, world: &World) {
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
        }
    }
}
