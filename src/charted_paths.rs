use std::collections::HashMap;

use petgraph::algo::{astar, dijkstra};
use petgraph::graph::{EdgeIndex, NodeIndex, UnGraph};
use petgraph::{Graph, Undirected};
use robotics_lib::interface::look_at_sky;
use robotics_lib::interface::Direction;
use robotics_lib::utils::calculate_cost_go_with_environment;
use robotics_lib::world::tile::{Tile, TileType};
use robotics_lib::world::World;

use crate::charted_coordinate::ChartedCoordinate;
use crate::{reserved::New, ChartingTool, NUMBER};

/// -----Welcome to the ChartedPaths!-----
/// The idea behind the ChartedPaths is to allow the user to better interact with the robot_map
/// that contains the tiles he has visited. In particular the purpose behind this structure is to
/// help the robot evaluate what is the cost to go from a visited position to another one and to
/// find what is the path to do so! This tool finds some interesting functionalities if it's used
/// with the charted_map, for example, if the AI is a Firefighter, it can use the charted_map to
/// save only the tiles with Water and then use the charted_path to find which ones is reachable
/// with fewer cost and what is the path to it.
/// Nb: This tool is designed to be used "one shot" when needed,
/// meaning that the the structure should be initialized in the process tick and. The reason behind
/// this is that the discovered world and the costs change very quickly so keeping updated the graph
/// would be not very efficient and not very useful.
///
/// Let's analyze the fields of the struct:
///     - graph: Graph<ChartedCoordinate, u32, Undirected>.
///          represents the undirected graph
///          the node contains a ChartedCoordinate (usize,usize) of the discovered tile
///          the edge between two nodes is the cost (u32) that the robot should spend by going
///          from the coordinate of node_1 to the coordinate of node_2
///     - indexes: Vec<Vec<Option<NodeIndex>>>.
///          it's a map nxn that contains the "conversion" of the map's coordinates to their
///          references in the graph.
///     - teleports_edges: HashMap<EdgeIndex, bool>.
///          contains the references to the edges in which one of the node is a teleport. Can be
///          used for further development or to easily changing the cost of teleport operations.
///
/// NB: since this struct is part of the Charting tool, the "new" function of its trait
///  implementation
///  ///     Example:
/// ```
///
///          use charting_tools::charted_paths::ChartedPaths;
///          use charting_tools::ChartingTool;
///          let mut charted_paths=ChartedPaths::new();
///
/// ```
///
/// Let's analyze the public functions provided with the ChartedPaths structure
///
/// 1) pub fn init(&mut self, &Vec<Vec<Option<Tile>>>, &World)
///     Robotic_lib provides a function called robot_map(..) that returns a matrix nxn in which
///     are "stored" the discovered tiles (seen or walked over) of the robot while
///     the other ones are set to None.
///     The function takes this map as parameter and crate a weighted graph of tiles choosing only
///     the tiles that the robot can walk over. The weight is relative to the actual
///     energy consumption. Since the cost is evaluated also with respect to the environmental
///     conditions is necessary to pass the World & reference
///     The function also takes care of the teleport functionality.
///     Example:
///
///         fn process_tick(& mut self, world:&mut World){
///             let mut charted_path = ChartedPaths::new();
///             charted_path.init(&robot_map(world).unwrap(), world);
///         }
///
///
///
/// 2) shortest_path_cost(&self, ChartedCoordinate, ChartedCoordinate) -> Option<u32>
///     Takes as parameter two coordinates, "from" and "to" as ChartedCoordinates.
///     Evaluates the cost of the shortest path between two coordinates using
///     Dijkstra algorithm (Complexity: O((V+E) log V). If the coordinates are out of bounds
///     or if there isn't a path between them it returns None.
///     ***NOTE***: as said in the introduction the entire structure and functions works on the
///     discovered tiles, so both coordinates passed to the function must be in the robot_map passed
///     in the function initialization.
///     Example:
///
///         fn process_tick(& mut self, world:&mut World){
///             let my_coordinate = ChartedCoordinate::from(self.get_coordinate());
///             let destination = ChartedCoordinate(1, 2);
///             let distance=cp.shortest_path_cost(my_coordinate,destination);
///         }
///
///
///
/// 3) shortest_path_cost_a_star(&self, ChartedCoordinate, ChartedCoordinate) -> Option<u32>
///     Same as shortest_path_cost but inside it uses the A* algorithm
///
///
///
/// 4) pub fn shortest_path(&self, ChartedCoordinate, ChartedCoordinate) ->
///         Option<(usize, Vec<ChartedCoordinate>)>
///    Takes as parameter two coordinates, "from" and "to".
///    Evaluates both the COST to go from the first coordinate to the second and the PATH using the
///    A* algorithm. If the coordinates are out of bounds or if there isn't a path between them
///    it returns None. The path is expressed as a Vector of tuples in which each tuple is a
///    coordinate of the robot_map (equal to the world's one) that can be used to move the robot
///    from one coordinate to another with the best possible energy consumption. NOTE: the path
///    comprehend both the from  coordinate and the end coordinate (So if the robot wants to move
///    to the objective tile it can skip to move to the first coordinate).
///    ***NOTE***: as said in the introduction the entire structure and functions works on the
///     discovered tiles, so both coordinates passed to the function must be in the robot_map passed
///     in the function initialization.
///    Example:
///
///         fn process_tick(& mut self, world:&mut World){
///             let my_coordinate = ChartedCoordinate::from(self.get_coordinate());
///             let destination = ChartedCoordinate(1, 2);
///             let best_path=cp.shortest_path(my_coordinate,destination);
///             match best_path {
///                None => {
///                     //path not found
///                }
///                Some(path) => {//cost = path.0, coordinates=path.1
///                     for i in 1..path.1.len(){ //moving the robot
///                         let  my_coordinate =ChartedCoordinate::from(self.get_coordinate());
///                         let updated_view= match go(self, world,
///                                     ChartedPaths::coordinates_to_direction(
///                                         my_coordinate,path.1[i]).unwrap()){
///                             Ok((view,_)) => {Some(view)}
///                             Err(_) => {None}
///                         };
///                     }
///                 }
///             }
///         }
///
///
/// 5) pub fn coordinates_to_direction(ChartedCoordinate, ChartedCoordinate) -> Result<Direction, ()>
///     This function converts what is the direction the robot need to move if he want to go from
///     a coordinate to another one. For example if the robot is in (0,0) and he wants to move to
///     (1,0) then he needs to pass Direction::Down to the go interface.
///     Example
///
#[derive(Debug, Clone)]
pub struct ChartedPaths {
    pub graph: Graph<ChartedCoordinate, u32, Undirected>,
    pub indexes: Vec<Vec<Option<NodeIndex>>>,
    pub teleports_edges: HashMap<EdgeIndex, bool>,
}

impl Drop for ChartedPaths {
    fn drop(&mut self) {
        if let Ok(mut n) = NUMBER.lock() {
            if *n > 0 {
                *n = *n - 1;
            }
        }
    }
}

impl ChartingTool for ChartedPaths {}

impl New for ChartedPaths {
    fn new() -> Self {
        ChartedPaths {
            graph: Default::default(),
            indexes: Vec::new(),
            teleports_edges: HashMap::new(),
        }
    }
}

#[allow(unused)]
impl ChartedPaths {
    pub fn init(&mut self, robot_map: &Vec<Vec<Option<Tile>>>, world: &World) {
        self.graph = UnGraph::<ChartedCoordinate, u32>::new_undirected();

        let mut teleports = Vec::new();

        let dimension = robot_map.len(); //the world is a square

        ChartedPaths::adds_nodes(
            &robot_map,
            dimension,
            &mut self.indexes,
            &mut self.graph,
            &mut teleports,
        );

        // Add vertices
        for i in 0..dimension {
            for j in 0..dimension {
                // check if the robot discovered that Tile
                match self.indexes[i][j].as_ref() {
                    | None => {}
                    | Some(present_tile) => {
                        // this checks if the robot walked over the tile or if he has
                        // seen it. but it also checks the walk-ability, since, not walkable
                        // nodes have not been added

                        // CHECK RIGHT NODE
                        if j != dimension - 1 {
                            // border check
                            match self.indexes[i][j + 1].as_ref() {
                                | None => {}
                                | Some(next_tile) => {
                                    // this checks if the robot walked over the tile or if he has
                                    // seen it.
                                    self.graph.add_edge(
                                        *present_tile,
                                        *next_tile,
                                        ChartedPaths::eval_weight(
                                            &ChartedCoordinate(i, j),
                                            &ChartedCoordinate(i, j + 1),
                                            &robot_map,
                                            &world,
                                        ),
                                    );
                                }
                            }
                        }
                        // CHECK NODE BELOW
                        if i != dimension - 1 {
                            // border check
                            match self.indexes[i + 1][j].as_ref() {
                                | None => {}
                                | Some(next_tile) => {
                                    // this checks if the robot walked over the tile or if he has
                                    // seen it. but it also checks the walk-ability, since, not walkable
                                    // nodes have not been added

                                    self.graph.add_edge(
                                        *present_tile,
                                        *next_tile,
                                        ChartedPaths::eval_weight(
                                            &ChartedCoordinate(i, j),
                                            &ChartedCoordinate(i + 1, j),
                                            &robot_map,
                                            &world,
                                        ),
                                    );
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
                let teleports_edge = self.graph.add_edge(
                    self.indexes[current_teleport.0][current_teleport.1].unwrap(),
                    self.indexes[next_teleport.0][next_teleport.1].unwrap(),
                    30,
                ); // teleport always consumes 30 energy
                self.teleports_edges.insert(teleports_edge, true);
            }
        }
    }

    pub fn shortest_path_cost(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> Option<u32> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }
        let result = dijkstra(
            &self.graph,
            self.indexes[from.0][from.1].unwrap(),
            self.indexes[to.0][to.1],
            |e| *e.weight(),
        );
        let cost = result.get(&self.indexes[to.0][to.1].unwrap());
        return match cost {
            | None => None,
            | Some(x) => Some(*x),
        };
    }
    pub fn shortest_path_cost_a_star(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> Option<u32> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }
        let path_info = astar(
            &self.graph,
            self.indexes[from.0][from.1].unwrap(),
            |finish| finish == self.indexes[to.0][to.1].unwrap(),
            |e| *e.weight(),
            |_| 0,
        );
        return match path_info {
            | None => None,
            | Some(info) => Some(info.0),
        };
    }
    pub fn shortest_path(
        &self,
        from: ChartedCoordinate,
        to: ChartedCoordinate,
    ) -> Option<(usize, Vec<ChartedCoordinate>)> {
        if ChartedPaths::check_boundaries(self, from, to) == false {
            return None;
        }

        let path_info = astar(
            &self.graph,
            self.indexes[from.0][from.1].unwrap(),
            |finish| finish == self.indexes[to.0][to.1].unwrap(),
            |e| *e.weight(),
            |_| 0,
        );

        return match path_info {
            | None => None,
            | Some(a_star_result) => {
                let cost = a_star_result.0 as usize;
                let nodes = a_star_result.1;

                let mut path = Vec::new();

                for i in nodes.iter() {
                    let converted = ChartedPaths::index_to_coordinate(self, i);
                    match converted {
                        | None => {}
                        | Some(x) => {
                            path.push(x);
                        }
                    }
                }
                Some((cost, path))
            }
        };
    }

    pub fn coordinates_to_direction(from: ChartedCoordinate, to: ChartedCoordinate) -> Result<Direction, ()> {
        if from.1 > to.1 {
            return Ok(Direction::Left);
        }
        if from.1 < to.1 {
            return Ok(Direction::Right);
        }
        if from.0 > to.0 {
            return Ok(Direction::Up);
        }
        if from.0 < to.0 {
            return Ok(Direction::Down);
        }

        // err casa: i pass the same coordinate
        return Err(());
    }

    fn check_boundaries(&self, from: ChartedCoordinate, to: ChartedCoordinate) -> bool {
        if (from.0 >= self.indexes.len())
            || (from.1 >= self.indexes.len())
            || (to.0 >= self.indexes.len())
            || (to.1 >= self.indexes.len())
        {
            return false;
        }
        return true;
    }

    fn index_to_coordinate(&self, node_index: &NodeIndex) -> Option<ChartedCoordinate> {
        let dim = self.indexes.len();
        for i in 0..dim {
            for (index, current_node) in self.indexes[i].iter().enumerate() {
                match current_node {
                    | None => {}
                    | Some(node) => {
                        if node == node_index {
                            return Some(ChartedCoordinate(i, index));
                        }
                    }
                }
            }
        }
        None
    }
    fn adds_nodes(
        matrix: &Vec<Vec<Option<Tile>>>,
        dim: usize,
        indexes: &mut Vec<Vec<Option<NodeIndex>>>,
        graph: &mut UnGraph<ChartedCoordinate, u32>,
        teleports: &mut Vec<ChartedCoordinate>,
    ) {
        // takes matrix as a reference of the robot map and the dimension of the map.
        // creates a graph with the walkable seen nodes,
        // changes the matrix of Node-indexes of the pathfinder that will be used to retrieve graph Indexes
        // add the teleports NodeIndexes
        for i in 0..dim {
            let mut row: Vec<Option<NodeIndex>> = Vec::with_capacity(dim);
            for j in 0..dim {
                match matrix[i][j].as_ref() {
                    // check if the robot discovered that Tile
                    | None => {
                        row.push(None);
                    }
                    | Some(present_tile) => {
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
    fn eval_weight(
        from: &ChartedCoordinate,
        to: &ChartedCoordinate,
        map: &Vec<Vec<Option<Tile>>>,
        world: &World,
    ) -> u32 {
        if ChartedCoordinate::is_close_to(from, to) {
            let env_cond = look_at_sky(world);

            match (map[from.0][from.1].as_ref(), map[to.0][to.1].as_ref()) {
                | (Some(tile_from), Some(tile_to)) => {
                    let base_cost = tile_from.tile_type.properties().cost();
                    let base_cost = calculate_cost_go_with_environment(base_cost, env_cond, tile_from.tile_type) as u32;
                    if tile_from.elevation < tile_to.elevation {
                        let elevation_cost = ((tile_from.elevation - tile_to.elevation) as i32).pow(2) as u32;
                        return (base_cost + elevation_cost);
                    }
                    return base_cost;
                }
                | _ => {
                    return u32::MAX;
                }
            }
        } else {
            u32::MAX
        }
    }
}
