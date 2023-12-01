use std::collections::HashMap;
use std::fmt::{Debug, Display, Formatter};
use std::hash::Hash;

use robotics_lib::runner::Robot;
use robotics_lib::world::tile::{Content, Tile, TileType};
use crate::charted_coordinate::ChartedCoordinate;
use crate::ChartingTool;

/// # Trait: ChartingTool
/// it is an internal trait that defines what can be used as a generic for ChartedMap
///
pub trait MapKey: Clone + Debug + Hash + Eq + PartialEq {
    fn to_default(&self) -> Self;
    fn get_quantity(&self) -> usize;

    fn from(tile: &Tile) -> Self;
}

impl MapKey for Tile {
    fn to_default(&self) -> Self {
        self.clone()
    }

    fn get_quantity(&self) -> usize {
        self.content.get_quantity()
    }

    fn from(tile: &Tile) -> Self {
        tile.clone()
    }
}

impl MapKey for Content {
    fn to_default(&self) -> Self {
        self.to_default()
    }

    fn get_quantity(&self) -> usize {
        match self {
            Content::Rock(x)
            | Content::Tree(x)
            | Content::Garbage(x)
            | Content::Coin(x)
            | Content::Water(x)
            | Content::Market(x)
            | Content::Fish(x) => x.to_owned(),
            Content::Fire | Content::Bin(_) | Content::Crate(_) | Content::Bank(_) => 1,
            Content::None => 0,
        }
    }

    fn from(tile: &Tile) -> Self {
        MapKey::to_default(&tile.content)
    }
}

impl MapKey for TileType {
    fn to_default(&self) -> Self {
        self.clone()
    }

    fn get_quantity(&self) -> usize {
        1usize
    }

    fn from(tile: &Tile) -> Self {
        MapKey::to_default(&tile.tile_type)
    }
}

/// # struct: ChartedMap
///
/// Allows you to save some type of point of interest and its coordinate, so that the robot can remember where it found it and in which quantity.
/// These are the types of point of interest the map can use:
/// - Content
/// - TileType
/// - Tile
///
/// ## Examples
///
///     let mut cm = ChartingTools::new::<Tile>();
///
///     let my_tile = robotics_lib::interface::robot_view(&my_robot, &my_world);
///     let my_coordinate = &ChartedCoordinate::from(my_robot.get_coordinate);
///
///     cm.save(&my_tile, &my_coordinate);
///
///     let retrieved = cm.get(&my_tile).unwrap();
///
///     assert_eq!(retrieved, my_tile)
///
pub(crate) struct ChartedMap<K: MapKey> {
    map: HashMap<K, Vec<(ChartedCoordinate, usize)>>,
}

impl<K: MapKey> ChartingTool for ChartedMap<K> {
    fn new() -> Self {
        Self {
            map: HashMap::new(),
        }
    }
}

impl<K: MapKey> From<Vec<Vec<Tile>>> for ChartedMap<K> {
    fn from(value: Vec<Vec<Tile>>) -> Self {
        let mut ret_map = ChartedMap::new();
        for (i, row) in value.iter().enumerate() {
            for (j, tile) in row.iter().enumerate() {
                ret_map.save(&K::from(tile), &ChartedCoordinate::new(i, j))
            }
        }
        ret_map
    }
}

impl<K: MapKey> From<Vec<Vec<Option<Tile>>>> for ChartedMap<K> {
    fn from(value: Vec<Vec<Option<Tile>>>) -> Self {
        let mut ret_map = ChartedMap::new();
        for (i, row) in value.iter().enumerate() {
            for (j, option) in row.iter().enumerate() {
                match option {
                    None => {}
                    Some(tile) => {
                        ret_map.save(&K::from(tile), &ChartedCoordinate::new(i, j))
                    }
                }
            }
        }
        ret_map
    }
}

impl<K: MapKey> ChartedMap<K> {
    pub fn save(&mut self, poi: &K, coordinate: &ChartedCoordinate) {
        let num= poi.get_quantity();
        let poi = poi.to_default();
        match self.get_mut(&poi) {
            None => { self.map.insert(poi, vec![(coordinate.clone(), num)]); }
            Some(v) => { v.push((coordinate.clone(), num)); }
        };
    }

    pub fn get(&self, poi: &K) -> Option<&Vec<(ChartedCoordinate, usize)>> {
        self.map.get(poi)
    }

    pub fn get_mut(&mut self, poi: &K) -> Option<&mut Vec<(ChartedCoordinate, usize)>> {
        self.map.get_mut(poi)
    }

    pub fn get_closest(&self, robot: &Robot, poi: &K) -> Option<ChartedCoordinate> {
        match self.get(poi) {
            None => { None }
            Some(pois) => {
                let mut closest = ChartedCoordinate::default();
                for (_, _) in pois.iter(){
                    todo!()
                }
                Some(closest)
            }
        }
    }
    pub fn get_most(&self, poi: &K) -> Option<(ChartedCoordinate, usize)> {
        match self.get(poi) {
            None => { None }
            Some(pois) => {
                let mut coordinate = ChartedCoordinate::default();
                let mut max = 0usize;
                for (c, s) in pois.iter() {
                    if *s > max {
                        max = *s;
                        coordinate = ChartedCoordinate::from(c);
                    }
                }
                Some((coordinate, max))
            }
        }
    }
}

impl<K: MapKey> Display for ChartedMap<K> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();
        s += "The Charted map contains:\n";
        for (poi, coordinates) in self.map.iter() {
            s += "Item: ";
            s += format!("{:?}\n", poi).as_str();
            for (coordinate, quantity) in coordinates {
                s += format!("at {} with quantity {}\n", coordinate, quantity).as_str();
            }
        }
        write!(f, "{}", s)
    }
}

#[test]
fn charted_map_test() {
    let mut map_content = ChartedMap::new();
    let mut map_tile_type = ChartedMap::new();

    let content1 = Content::Rock(12);
    let content2 = Content::Coin(11);
    let content3 = Content::Coin(10);

    let tile_type1=TileType::Grass;
    let tile_type2=TileType::Street;
    let tile_type3=TileType::ShallowWater;

    let c1 = ChartedCoordinate::new(12, 21);
    let c2 = ChartedCoordinate::new(1, 2);
    let c3 = ChartedCoordinate::new(4, 44);

    map_content.save(&content1, &c1);
    map_content.save(&content2, &c2);
    map_content.save(&content3, &c3);

    map_tile_type.save(&tile_type1, &c1);
    map_tile_type.save(&tile_type2, &c2);
    map_tile_type.save(&tile_type3, &c3);

    println!("{}", map_content);
    println!("{}",map_tile_type);
    let c = map_content.get(&Content::Coin(0));
    println!("{c:?}");
}