use robotics_lib::interface::look_at_sky;
use crate::charted_coordinate::ChartedCoordinate;
use robotics_lib::world::World;
fn distance_to(who:(usize,usize), to:(usize,usize))->(usize,usize){
    ((who.0-to.0), (who.1-to.1))
}
fn is_close_to(who:(usize,usize), to:(usize,usize))->bool{
    if ((distance_to(who, to).0)as i32).pow(2)+(((distance_to(who, to).1)as i32).pow(2))<2{
        true
    }
    false
}

fn eval_weight(from:&ChartedCoordinate, to:&ChartedCoordinate, map:&Vec<Vec<Option<Tile>>>)->Option<u32>{

    match map[from.0][from.1] {
        Some(X) => {if is_close_to(from,to) {
            let env_cond = look_at_sky(world);//dove
            let base_cost = map[from.0][from.1].unwrap().properties().cost();
            if map[from.0][from.1].unwrap().elevation < map[to.0][to.1].unwrap().elevation{
                let elevation_cost = ((map[to.0][to.1].unwrap().elevation - map[from.0][from.1].unwrap().elevation)as i32).pow(2);
                Some(base_cost + elevation_cost)
            }
            Some(base_cost)}
        else{
            None
        }
        }
        None => panic!()
    }
}

fn weight2(from:&ChartedCoordinate, to:&ChartedCoordinate, map:&Vec<Vec<Option<Tile>>>)->u32{
    let base_cost = map[from.0][from.1].unwrap().properties().cost();
    if map[from.0][from.1].unwrap().elevation < map[to.0][to.1].unwrap().elevation{
        let elevation_cost = ((map[to.0][to.1].unwrap().elevation - map[from.0][from.1].unwrap().elevation)as i32).pow(2);
        base_cost + elevation_cost
    }
    base_cost
}