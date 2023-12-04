use std::collections::HashMap;
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::{Graph, Undirected};
use petgraph::algo::dijkstra;
use petgraph::dot::{Config, Dot};
use robotics_lib::world::coordinates::Coordinate;
use robotics_lib::world::tile::{Content, Tile, TileType};
/// Purpose of this function is to take as input an Option<Vec<Vec<Option<Tile>>>> (output of robot_map)
/// and return a weighted graph of connected tiles (meaning tiles in which go_allowed() is true).
///


struct PathFinder{
    graph:Graph<(usize, usize), u32, Undirected>,
    indexes: Vec<Vec<Option<NodeIndex>>>,
}

fn eval_weight(c1:(usize,usize), c2:(usize,usize))->u32{
    return 1;
}

fn adds_nodes(matrix:&Vec<Vec<Option<Tile>>>, dim:usize,indexes:&mut Vec<Vec<Option<NodeIndex>>>, graph: &mut UnGraph::<(usize,usize), u32>, teleports:&mut Vec<NodeIndex>){
    // takes matrix as a reference of the robot map and the dimension of the map.
    // creates a graph with the walkable seen nodes,
    // changes the matrix of Nodeindexes of the pathfinder that will be used to retrieve graph Indexes
    // add the teleports NodeIndexes
    for i in  0..dim{
        let mut row: Vec<Option<NodeIndex>> = Vec::with_capacity(dim);
        for j in 0..dim{
            match matrix[i][j].as_ref() {
                // check if the robot discovered that Tile
                None => {
                    row.push(None);
                }
                Some(present_tile) => {
                    // this checks if the robot walked over the tile or if he has
                    // seen it.
                    if !present_tile.tile_type.properties().walk(){
                        // Since the vector contains also Tiles that the robot has seen
                        // we have to check if the tile we are looking at is walkable or not
                        // if not i don't need it in the graph but i still need in indexes
                        // to keep the matrix dimxdim
                        row.push(None);
                        continue;
                    }

                    let current_node=graph.add_node((i, j));
                    if present_tile.tile_type == TileType::Teleport(true){
                        teleports.push(current_node);
                    }
                    row.push(Some(current_node));
                }
            }
        }
        indexes.push(row);
    }
}


fn into_graph(robot_map: &Option<Vec<Vec<Option<Tile>>>>) -> PathFinder {
    let mut pathfinder = PathFinder{
        graph: UnGraph::<(usize,usize), u32>::new_undirected(),
        indexes: Vec::new(),
    };

    let mut teleports= Vec::new();

    match robot_map {
        None => {
            // TODO: handle empty map
        }
        Some(vec) => {

            let dimension=vec.len(); //the world is a square

            adds_nodes(&vec, dimension, &mut pathfinder.indexes, &mut pathfinder.graph, &mut teleports);
            //println!("INDEXES: {:?}", pathfinder.indexes);
            //println!("ORIGINAL: {:?}", vec);

            // Add vertices
            for i in 0..dimension{
                for j in 0..dimension{

                    // check if the robot discovered that Tile
                    match pathfinder.indexes[i][j].as_ref() {
                        None => {}
                        Some(present_tile) => {
                            // this checks if the robot walked over the tile or if he has
                            // seen it. but it also checks the walkability, since, not walkable
                            // nodes have not been added

                            // CHECK RIGHT NODE
                            if j!=dimension-1 { // border check
                                match pathfinder.indexes[i][j+1].as_ref(){
                                    None => {}
                                    Some(next_tile) => {
                                        // this checks if the robot walked over the tile or if he has
                                        // seen it.
                                        pathfinder.graph.add_edge(*present_tile,
                                                                  *next_tile,
                                                       eval_weight((i,j),(i,j+1)));
                                    }
                                }
                            }
                            // CHECK NODE BELOW
                            if i!=dimension-1{ // bordere check
                                match pathfinder.indexes[i+1][j].as_ref(){
                                    None => {}
                                    Some(next_tile) => {
                                        // this checks if the robot walked over the tile or if he has
                                        // seen it. but it also checks the walkability, since, not walkable
                                        // nodes have not been added

                                        pathfinder.graph.add_edge(*present_tile,
                                                                  *next_tile,
                                                       eval_weight((i,j),(i+1,j)));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for (index,current_teleport) in teleports.iter().enumerate(){
        for i in index+1..teleports.len()-1{
            let c1=pathfinder.graph.node_weight(*current_teleport).unwrap();
            let c2=pathfinder.graph.node_weight(teleports[i+1]).unwrap();
            pathfinder.graph.add_edge(*current_teleport,teleports[i+1], eval_weight(*c1,*c2));
        }
    }

    pathfinder

}

#[test]
fn test_library_usage(){
    let mut graph= UnGraph::<(usize,usize), u32>::new_undirected();
    let coordiante_1= (0,0);
    let coordiante_2= (0,1);
    let coordiante_3= (1,0);
    let coordiante_4= (1,1);
    let coordiante_5= (2,0);
    let coordiante_6= (2,1);

    let node_1= graph.add_node(coordiante_1);
    let node_2= graph.add_node(coordiante_2);
    let node_3= graph.add_node(coordiante_3);
    let node_4= graph.add_node(coordiante_4);
    let node_5= graph.add_node(coordiante_5);
    let node_6= graph.add_node(coordiante_6);

    graph.add_edge(node_1, node_2, eval_weight(coordiante_1,coordiante_2));
    graph.add_edge(node_1, node_3, eval_weight(coordiante_1,coordiante_3));
    graph.add_edge(node_2, node_4, eval_weight(coordiante_2,coordiante_4));
    graph.add_edge(node_3, node_4, eval_weight(coordiante_3,coordiante_4));
    graph.add_edge(node_3, node_5, eval_weight(coordiante_3,coordiante_5));
    graph.add_edge(node_5, node_6, eval_weight(coordiante_5,coordiante_6));
    graph.add_edge(node_4, node_4, eval_weight(coordiante_4,coordiante_6));

    // Print the graph in DOT format
    //println!("{:?}", Dot::with_config(&graph, &[Config::EdgeNoLabel]));

    let start_node = node_1;
    let target_node = node_6;

    let result= dijkstra(&graph, start_node, None, |e| *e.weight());
    let cost=result.get(&node_6).unwrap();

    println!("Il costo da (0,0) a (2,1) e': {:?}", cost); //3 step perchè non posso andare in diagonale.

}

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

fn find_node_by_data(graph: &UnGraph<(usize,usize), u32>, target_data: (usize,usize)) -> Option<petgraph::graph::NodeIndex> {
    graph.node_indices().find(|&node| graph[node] == target_data)
}


#[test]
fn test_into_graph_v1(){
    // see example at: ./../docfiles/world_example.png
    let mut walkable=Tile{
        tile_type: TileType::Sand,
        content: Content::Rock(1),
        elevation: 0,
    };

    let not_walkable=Tile{
        tile_type: TileType::DeepWater,
        content: Content::Coin(1),
        elevation: 10,
    };

    let mut robot_map= Vec::new();

    for i in 0..5{
        let mut row_vector= Vec::new();
        for j in 0..5{
            if i==2 || i==4{
                row_vector.push(Some(not_walkable.clone())); //t10 -->t14 && t20 -->t24
            }
            else{
                row_vector.push(Some(walkable.clone()));
            }
        }
        robot_map.push(row_vector);
    }

    set_tile_type!(robot_map, 0, 3, TileType::DeepWater); //t3
    set_tile_type!(robot_map, 0, 4, TileType::Teleport(true)); // t4
    set_tile_type!(robot_map, 1, 1, TileType::DeepWater); //t6
    set_tile_type!(robot_map, 1, 2, TileType::Teleport(true)); //t7
    set_tile_type!(robot_map, 3, 4, TileType::Teleport(true)); //t18

    println!("Robot map: {:?}", robot_map);

    let pathfinder=into_graph(&Some(robot_map));
    println!("The converted graph is: {:?}", pathfinder.graph);

    let result = dijkstra(&pathfinder.graph,pathfinder.indexes[0][0].unwrap(), pathfinder.indexes[0][4], |e| *e.weight());

    let cost=result.get(&pathfinder.indexes[0][4].unwrap()).unwrap_or(&1000);
    println!("The cost from (0,0) to (0,4) is: {:?}", cost);
}