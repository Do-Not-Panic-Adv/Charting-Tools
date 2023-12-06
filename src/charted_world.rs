use std::collections::HashMap;

use robotics_lib::interface::{discover_tiles, robot_map, robot_view};
use robotics_lib::runner::Runnable;
use robotics_lib::utils::LibError;
use robotics_lib::world::tile::{Content, Tile, TileType};
use robotics_lib::world::World;

use crate::charted_coordinate::ChartedCoordinate;
use crate::ChartingTool;

pub(crate) struct ChartedWorld {
    map: Vec<Vec<Option<Tile>>>,
}

impl ChartingTool for ChartedWorld {
    fn new() -> Self {
        Self {
            map: Vec::default(),
        }
    }
}

impl ChartedWorld {
    pub fn init(&mut self, world: &World, dimension: usize) {
        self.map = robot_map(world).unwrap_or(vec![vec![None; dimension]; dimension]);
    }

    pub fn at(&self, coordinate: ChartedCoordinate) -> Option<Tile>{
        self.map[coordinate.get_row()][coordinate.get_col()].clone()
    }

    pub fn get(&self) -> &Vec<Vec<Option<Tile>>> {
        &self.map
    }

    pub fn update(&mut self, world: &World, coordinates: &Vec<ChartedCoordinate>) {
        let option = robot_map(world);
        if option.is_none() { return; }

        let map = option.unwrap();
        for point in coordinates.iter() {
            if let Some(tile) = map[point.get_row()][point.get_col()].as_ref() {
                self.map[point.get_row()][point.get_col()] = Some(tile.clone());
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
                    self.map[i + conversion_coordinate.get_row()][j + conversion_coordinate.get_col()] = view[i][j].clone();
                }
            }
        }
    }

    // pub fn update_discover(&mut self, robot: &mut impl Runnable, world: &mut World, to_discover: &Vec<ChartedCoordinate>)
    //                        -> Result<HashMap<(usize, usize), Option<(TileType, Content)>>, LibError> {
    //     todo!();
    //     return match discover_tiles(robot, world, &to_discover.iter().map(|c| (c.get_row(), c.get_col())).collect::<Vec<(usize, usize)>>()) {
    //         Ok(hm) => {
    //             todo!();
    //             for ((x, y), tile) in hm.iter() {
    //                 // self.map[*x][*y] = tile.clone();
    //             }
    //             Ok(hm)
    //         }
    //         Err(err) => {
    //             Err(err)
    //         }
    //     }
    // }

    pub fn update_all(&mut self, world: &World) {
        let map = robot_map(world);
        if map.is_some() {
            let map = map.unwrap();
            for (i, row) in map.iter().enumerate() {
                for (j, tile) in row.iter().enumerate() {
                    if tile.is_some() {
                        self.map[i][j] = map[i][j].clone();
                    }
                }
            }
        }
    }
}

#[test]
fn test() {}