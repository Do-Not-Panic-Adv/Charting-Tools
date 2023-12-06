use std::collections::HashMap;

use petgraph::{Graph, Undirected};
use petgraph::algo::{astar, dijkstra};
use petgraph::graph::{EdgeIndex, NodeIndex, UnGraph};
use robotics_lib::interface::look_at_sky;
use robotics_lib::utils::calculate_cost_go_with_environment;
use robotics_lib::world::tile::{Content, Tile, TileType};
use robotics_lib::world::World;

use crate::charted_coordinate::ChartedCoordinate;
use crate::ChartingTool;

/// -----Welcome to the Pathfinder!-----
/// The idea behind the Pathfinder is to allow the user to better interact with the robot_map
/// that contains the tiles he has visited. In particular the purpose behind this structure is to
/// help the robot evaluate what is the cost to go from a visited position to another one and to
/// find what is the path to do so!
///
/// Let's analyze the public functions provided with the Pathfinder structure
///
/// 1) from_map(&Vec<Vec<Option<Tile>>>)-> PathFinder
///     Robotic_lib provide a function called robot_map(..) that returns a matrix nxn in which
///     are "stored" the discovered tiles (seen or walked over) of the robot while
///     the other ones are set to None.
///     The function takes this map as parameter and crate a weighted graph of tiles choosing only
///     the tiles that the robot can walk over. The weight is relative to the actual
///     energy consumption.
///     The function also takes care of the teleport functionality.
///
///     Example:
///         let pathfinder=PathFinder::from_map(&robot_map);
///
///
/// 2) shortest_path_cost(&self, (usize, usize), (usize, usize)) ->Option<u32>
///     Takes as parameter two coordinates, "from" and "to".
///     Evaluates the cost of the shortest path between two coordinates using
///     Dijkstra algorithm (Complexity: O((V+E) log V). If the coordinates are out of bounds
///     or if there isn't a path between them it returns None
///
///     Example:
///         let pathfinder=PathFinder::from_map(&robot_map);
///         let path_cost:Option<u32> = pathfinder.shortest_path_cost((0,0), (0,4));
///
/// 3) shortest_path_cost_a_star(&self, (usize, usize), (usize, usize)) ->Option<u32>
///     Same as shortest_path_cost but inside it used the A* algorithm
///
/// 4) shortest_path(&self, (usize,usize), (usize,usize))->Option<(usize,Vec<(usize,usize)>)>
///    Takes as parameter two coordinates, "from" and "to".
///    Evaluates both the COST to go from the first coordinate to the second and the PATH using the
///    A* algorithm If the coordinates are out of bounds or if there isn't a path between them
///    it returns None otherwise it returns a Vector of tuples in which each tuple is a coordinate
///    of the robot_map that can be used to move the robot from one coordinate to another with the
///    best possible energy consumption


pub struct ChartedPaths {
    pub graph: Graph<ChartedCoordinate, u32, Undirected>,
    pub indexes: Vec<Vec<Option<NodeIndex>>>,
    pub teleports_edges: HashMap<EdgeIndex, bool>,
}

impl ChartingTool for ChartedPaths {
    fn new() -> Self {
        ChartedPaths {
            graph: Default::default(),
            indexes: Vec::new(),
            teleports_edges: HashMap::new(),
        }
    }
}

#[allow(unused)]
fn eval_weight(from: &ChartedCoordinate, to: &ChartedCoordinate, map: &Vec<Vec<Option<Tile>>>, world: &World) -> u32 {
    if ChartedCoordinate::is_close_to(from, to) {
        let env_cond = look_at_sky(world);

        match (map[from.0][from.1].as_ref(), map[to.0][to.1].as_ref()) {
            (Some(tile_from), Some(tile_to)) => {
                let base_cost = tile_from.tile_type.properties().cost();
                let base_cost = calculate_cost_go_with_environment(base_cost, env_cond, tile_from.tile_type) as u32;
                if tile_from.elevation < tile_to.elevation {
                    let elevation_cost = ((tile_from.elevation - tile_to.elevation) as i32).pow(2) as u32;
                    return (base_cost + elevation_cost);
                }
                return base_cost;
            }
            _ => { return u32::MAX; }
        }
    } else {
        u32::MAX
    }
}


impl ChartedPaths {
    pub fn init(&mut self, robot_map: &Vec<Vec<Option<Tile>>>, world: &World) {
        self.graph = UnGraph::<ChartedCoordinate, u32>::new_undirected();

        let mut teleports = Vec::new();

        let dimension = robot_map.len(); //the world is a square

        ChartedPaths::adds_nodes(&robot_map, dimension, &mut self.indexes, &mut self.graph, &mut teleports);

        // Add vertices
        for i in 0..dimension {
            for j in 0..dimension {

                // check if the robot discovered that Tile
                match self.indexes[i][j].as_ref() {
                    None => {}
                    Some(present_tile) => {
                        // this checks if the robot walked over the tile or if he has
                        // seen it. but it also checks the walk-ability, since, not walkable
                        // nodes have not been added

                        // CHECK RIGHT NODE
                        if j != dimension - 1 { // border check
                            match self.indexes[i][j + 1].as_ref() {
                                None => {}
                                Some(next_tile) => {
                                    // this checks if the robot walked over the tile or if he has
                                    // seen it.
                                    self.graph.add_edge(*present_tile,
                                                        *next_tile,
                                                        eval_weight(&ChartedCoordinate(i, j), &ChartedCoordinate(i, j + 1), &robot_map, &world));
                                }
                            }
                        }
                        // CHECK NODE BELOW
                        if i != dimension - 1 { // border check
                            match self.indexes[i + 1][j].as_ref() {
                                None => {}
                                Some(next_tile) => {
                                    // this checks if the robot walked over the tile or if he has
                                    // seen it. but it also checks the walk-ability, since, not walkable
                                    // nodes have not been added

                                    self.graph.add_edge(*present_tile,
                                                        *next_tile,
                                                        eval_weight(&ChartedCoordinate(i, j), &ChartedCoordinate(i + 1, j), &robot_map, &world));
                                }
                            }
                        }
                    }
                }
            }
        }

        for (index, current_teleport) in teleports.iter().enumerate() {
            for i in index..teleports.len() - 1 {
                let next_teleport = teleports[i + 1];
                let teleports_edge = self.graph.add_edge(self.indexes[current_teleport.0][current_teleport.1].unwrap(),
                                                         self.indexes[next_teleport.0][next_teleport.1].unwrap(),
                                                         30); // teleport always consumes 30 energy
                self.teleports_edges.insert(teleports_edge, true);
            }
        }
    }

    pub fn shortest_path_cost(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> Option<u32> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }
        let result = dijkstra(&self.graph,
                              self.indexes[from.0][from.1].unwrap(),
                              self.indexes[to.0][to.1], |e| *e.weight());
        let cost = result.get(&self.indexes[0][4].unwrap());
        return match cost {
            None => { None }
            Some(x) => { Some(*x) }
        };
    }
    pub fn shortest_path_cost_a_star(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> Option<u32> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }
        let path_info = astar(&self.graph, self.indexes[from.0][from.1].unwrap(),
                              |finish| finish == self.indexes[to.0][to.1].unwrap(),
                              |e| *e.weight(),
                              |_| 0,
        );
        return match path_info {
            None => {
                None
            }
            Some(info) => {
                Some(info.0)
            }
        };
    }
    pub fn shortest_path(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> Option<(usize, Vec<ChartedCoordinate>)> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }

        let path_info = astar(&self.graph, self.indexes[from.0][from.1].unwrap(),
                              |finish| finish == self.indexes[to.0][to.1].unwrap(),
                              |e| *e.weight(),
                              |_| 0,
        );

        return match path_info {
            None => { None }
            Some(a_star_result) => {
                let cost = a_star_result.0 as usize;
                let nodes = a_star_result.1;

                let mut path = Vec::new();

                for i in nodes.iter() {
                    let converted = ChartedPaths::index_to_coordinate(self, i);
                    match converted {
                        None => {}
                        Some(x) => {
                            path.push(x);
                        }
                    }
                }
                Some((cost, path))
            }
        };
    }

    fn check_boundaries(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> bool {
        if (from.0 >= self.indexes.len()) || (from.1 >= self.indexes.len()) ||
            (to.0 >= self.indexes.len()) || (to.1 >= self.indexes.len()) {
            return false;
        }
        return true;
    }

    fn index_to_coordinate(&self, node_index: &NodeIndex) -> Option<ChartedCoordinate> {
        let dim = self.indexes.len();
        for i in 0..dim {
            for (index, current_node) in self.indexes[i].iter().enumerate() {
                match current_node {
                    None => {}
                    Some(node) => {
                        if node == node_index {
                            return Some(ChartedCoordinate(i, index));
                        }
                    }
                }
            }
        }
        None
    }
    fn adds_nodes(matrix: &Vec<Vec<Option<Tile>>>, dim: usize, indexes: &mut Vec<Vec<Option<NodeIndex>>>, graph: &mut UnGraph<ChartedCoordinate, u32>, teleports: &mut Vec<ChartedCoordinate>) {
        // takes matrix as a reference of the robot map and the dimension of the map.
        // creates a graph with the walkable seen nodes,
        // changes the matrix of Node-indexes of the pathfinder that will be used to retrieve graph Indexes
        // add the teleports NodeIndexes
        for i in 0..dim {
            let mut row: Vec<Option<NodeIndex>> = Vec::with_capacity(dim);
            for j in 0..dim {
                match matrix[i][j].as_ref() {
                    // check if the robot discovered that Tile
                    None => {
                        row.push(None);
                    }
                    Some(present_tile) => {
                        // this checks if the robot walked over the tile or if he has
                        // seen it.
                        if !present_tile.tile_type.properties().walk() {
                            // Since the vector contains also Tiles that the robot has seen
                            // we have to check if the tile we are looking at is walkable or not
                            // if not i don't need it in the graph but i still need in indexes
                            // to keep the matrix dim x dim
                            row.push(None);
                            continue;
                        }

                        let current_node = graph.add_node(ChartedCoordinate(i, j));
                        if present_tile.tile_type == TileType::Teleport(true) {
                            teleports.push(ChartedCoordinate(i, j));
                        }
                        row.push(Some(current_node));
                    }
                }
            }
            indexes.push(row);
        }
    }
}

// ----------------Test usage only------------------------
macro_rules! set_tile_type {
    ($map:expr, $row:expr, $col:expr, $tile_type:expr) => {
        if let Some(row) = $map.get_mut($row) {
            if let Some(tile_option) = row.get_mut($col) {
                if let Some(tile) = tile_option.as_mut() {
                    tile.tile_type = $tile_type;
                }
            }
        }
    };
}


#[test]
fn test_correct_calls() {

    // ------------ Creating the map example at: ./../docfiles/world_example.png ------------
    let walkable = Tile {
        tile_type: TileType::Sand,
        content: Content::Rock(1),
        elevation: 0,
    };

    let not_walkable = Tile {
        tile_type: TileType::DeepWater,
        content: Content::Coin(1),
        elevation: 10,
    };

    let mut robot_map = Vec::new();

    for i in 0..5 {
        let mut row_vector = Vec::new();
        for _ in 0..5 {
            if i == 2 || i == 4 {
                row_vector.push(Some(not_walkable.clone())); //t10 -->t14 && t20 -->t24
            } else {
                row_vector.push(Some(walkable.clone()));
            }
        }
        robot_map.push(row_vector);
    }

    set_tile_type!(robot_map, 0, 3, TileType::DeepWater);
    //t3
    set_tile_type!(robot_map, 0, 4, TileType::Teleport(true));
    // t4
    set_tile_type!(robot_map, 1, 1, TileType::DeepWater);
    //t6
    set_tile_type!(robot_map, 1, 2, TileType::Teleport(true));
    //t7
    set_tile_type!(robot_map, 3, 4, TileType::Teleport(true)); //t18

    // ------------ End of "robot_map" initialization  ------------

    // let pathfinder=PathFinder::from_map(&robot_map);
    // // Builds the PathFinder from the robot_map
    //
    // let cost_one = pathfinder.shortest_path_cost((0,0), (0,4));
    // // evaluates cost from tile t0 to t4. NB: there is the teleport.
    //
    // let cost_two = pathfinder.shortest_path_cost_a_star((0,0), (1,4));
    // // evaluates cost from tile t0 to t9
    //
    // println!("The cost from (0,0) to (0,4) is: {:?}", cost_one.unwrap());
    // assert_eq!(6,cost_one.unwrap());
    //
    // println!("The cost from (0,0) to (1,4) is: {:?}", cost_two.unwrap());
    // assert_eq!(5,cost_two.unwrap());
    //
    // let path=pathfinder.shortest_path((0,0), (0,4));
    // // evaluates the cost and the shortest path from t0 to t4. NB: there is the teleport
    //
    // for i in path.clone().unwrap().1{
    //     // iterate over the shortest path coordinates
    //     println!("{:?}",i);
    // }
    //
    // assert_eq!(path.unwrap().1,vec![
    //     (0, 0),
    //     (0, 1),
    //     (0, 2),
    //     (1, 2),
    //     (1,3),
    //     (1,4),
    //     (0, 4)
    // ])
}