use std::collections::hash_map::Iter;
use std::collections::HashMap;
use std::fmt::{Debug, Display, Formatter};
use std::hash::Hash;
use std::ops::Range;

use robotics_lib::world::tile::{Content, Tile, TileType};

use crate::charted_coordinate::ChartedCoordinate;
use crate::{hidden::New, ChartingTool, NUMBER};

/// # Trait: MapKey
/// it is an internal trait that defines what can be used as a generic for ChartedMap
///
/// implemented for:
/// -       robotics_lib::world::tile::Content
/// -       robotics_lib::world::tile::TileType
/// -       robotics_lib::world::tile::Tile
pub trait MapKey: Clone + Debug + Hash + Eq + PartialEq {
    /// returns a default value for the relevant datatype
    fn to_default(&self) -> Self;
    /// returns a quantity for the relevant datatype
    fn get_quantity(&self) -> SavedQuantity;
    /// obtains the desired type from a Tile
    fn from(tile: &Tile) -> Self;
}

impl MapKey for Tile {
    fn to_default(&self) -> Self {
        Tile {
            tile_type: self.tile_type,
            content: self.content.to_default(),
            elevation: 0,
        }
    }

    fn get_quantity(&self) -> SavedQuantity {
        SavedQuantity::TileElevation(self.elevation)
    }

    fn from(tile: &Tile) -> Self {
        tile.clone()
    }
}

impl MapKey for Content {
    fn to_default(&self) -> Self {
        self.to_default()
    }

    fn get_quantity(&self) -> SavedQuantity {
        match self {
            | Content::Rock(x)
            | Content::Tree(x)
            | Content::Garbage(x)
            | Content::Coin(x)
            | Content::Water(x)
            | Content::Market(x)
            | Content::JollyBlock(x)
            | Content::Bush(x)
            | Content::Fish(x) => SavedQuantity::ContentQuantity(x.to_owned()),
            | Content::Bin(x) | Content::Crate(x) | Content::Bank(x) => SavedQuantity::ContentRange(x.to_owned()),
            | Content::Fire | Content::Building | Content::Scarecrow | Content::None => SavedQuantity::None,
        }
    }

    fn from(tile: &Tile) -> Self {
        tile.content.clone()
    }
}

impl MapKey for TileType {
    fn to_default(&self) -> Self {
        self.clone()
    }

    fn get_quantity(&self) -> SavedQuantity {
        SavedQuantity::None
    }

    fn from(tile: &Tile) -> Self {
        MapKey::to_default(&tile.tile_type)
    }
}

/// # Enum: SavedType
/// describes the possible values saved inside the ChartedMap
/// possible values are:
/// -       None
/// -       ContentQuantity(usize),
/// -       ContentRange(Range<usize>),
/// -       TileElevation(usize),
///
/// ## Example
///```
/// use charting_tools::ChartingTools;
/// use robotics_lib::world::tile::TileType;
/// use charting_tools::charted_coordinate::ChartedCoordinate;
/// use charting_tools::charted_map::ChartedMap;
///
/// let mut cm = ChartingTools::tool::<ChartedMap<TileType>>();
///     cm.save(TileType::Snow, ChartedCoordinate::default());
///
///     let res = cm.get(TileType::Snow);
///     if res.unwrap()[0].1.is_none() {
///         println!("quack");
///     }
/// ```
#[derive(Debug, Clone)]
pub enum SavedQuantity {
    None,
    ContentQuantity(usize),
    ContentRange(Range<usize>),
    TileElevation(usize),
}

impl SavedQuantity {
    /// returns true if there is any saved value
    /// which is not
    ///
    ///     SavedQuantity::None
    pub fn is_some(&self) -> bool {
        match self {
            | SavedQuantity::None => false,
            | _ => true,
        }
    }
    /// returns true if the value is of type
    ///
    ///     SavedQuantity::None
    pub fn is_nome(&self) -> bool {
        match self {
            | SavedQuantity::None => true,
            | _ => false,
        }
    }

    /// returns true if the value is of type
    ///
    ///     SavedQuantity::TileElevation(_)
    /// or
    ///
    ///     SavedQuantity::ContentQuantity(_)
    pub fn is_usize(&self) -> bool {
        match self {
            | SavedQuantity::TileElevation(_) | SavedQuantity::ContentQuantity(_) => true,
            | _ => false,
        }
    }

    /// returns true if the value is of type
    ///
    ///     SavedQuantity::ContentRange(_)
    pub fn is_range(&self) -> bool {
        match self {
            | SavedQuantity::ContentRange(_) => true,
            | _ => false,
        }
    }
}

impl PartialEq for SavedQuantity {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            | (SavedQuantity::ContentRange(a), SavedQuantity::ContentRange(b)) => a == b,
            | (SavedQuantity::ContentQuantity(a), SavedQuantity::ContentQuantity(b))
            | (SavedQuantity::TileElevation(a), SavedQuantity::TileElevation(b)) => *a == *b,
            | _ => false,
        }
    }
}

impl Display for SavedQuantity {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                | SavedQuantity::None => "None".to_string(),
                | SavedQuantity::ContentQuantity(q) => q.to_string(),
                | SavedQuantity::ContentRange(r) => format!("{:?}", r),
                | SavedQuantity::TileElevation(e) => e.to_string(),
            }
        )
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
/// ```
/// use robotics_lib::runner::Robot;
/// use robotics_lib::world::tile::Content;
/// use robotics_lib::world::World;
/// use charting_tools::charted_coordinate::ChartedCoordinate;
/// use charting_tools::charted_map::ChartedMap;
/// use charting_tools::ChartingTools;
///
///
/// let mut cm = ChartingTools::tool::<ChartedMap<Content>>();
///
/// // inside loop
/// # let my_robot = ();
/// # let my_world = ();
/// let my_tile = robotics_lib::interface::robot_view(&my_robot, &my_world);
/// let my_coordinate = &ChartedCoordinate::from(my_robot.coordinate);
///
/// cm.save(&my_tile, &my_coordinate);
///
/// let retrieved = cm.get(&my_tile).unwrap();
///
/// assert_eq!(retrieved, my_tile)
///```
#[derive(Debug, Clone, PartialEq)]
pub struct ChartedMap<K: MapKey> {
    map: HashMap<K, Vec<(ChartedCoordinate, SavedQuantity)>>,
}

impl<K: MapKey> Drop for ChartedMap<K> {
    fn drop(&mut self) {
        if let Ok(mut n) = NUMBER.lock() {
            if *n > 0 {
                *n = *n - 1;
            }
        }
    }
}

impl<K: MapKey> ChartingTool for ChartedMap<K> {}

impl<K: MapKey> New for ChartedMap<K> {
    fn new() -> Self {
        Self { map: HashMap::new() }
    }
}

impl<K: MapKey> ChartedMap<K> {
    pub fn copy(&mut self, value: Vec<Vec<Option<Tile>>>) {
        self.clear();
        for (i, row) in value.iter().enumerate() {
            for (j, option) in row.iter().enumerate() {
                match option {
                    | None => {}
                    | Some(tile) => self.save(&K::from(tile), &ChartedCoordinate::new(i, j)),
                }
            }
        }
    }
    pub fn iter(&self) -> Iter<'_, K, Vec<(ChartedCoordinate, SavedQuantity)>> {
        self.map.iter()
    }
    pub fn save(&mut self, poi: &K, coordinate: &ChartedCoordinate) {
        let num = poi.get_quantity();
        let poi = poi.to_default();
        match self.get_mut(&poi) {
            | None => {
                self.map.insert(poi, vec![(coordinate.clone(), num)]);
            }
            | Some(v) => {
                v.push((coordinate.clone(), num));
            }
        };
    }

    pub fn get(&self, poi: &K) -> Option<&Vec<(ChartedCoordinate, SavedQuantity)>> {
        self.map.get(&poi.to_default())
    }

    fn get_mut(&mut self, poi: &K) -> Option<&mut Vec<(ChartedCoordinate, SavedQuantity)>> {
        self.map.get_mut(&poi.to_default())
    }

    pub fn get_most(&self, poi: &K) -> Option<(ChartedCoordinate, usize)> {
        match self.get(poi) {
            | None => None,
            | Some(pois) => {
                let mut coordinate = ChartedCoordinate::default();
                let mut max = 0usize;
                for (c, s) in pois.iter() {
                    match s {
                        | SavedQuantity::None => {}
                        | SavedQuantity::ContentQuantity(q) | SavedQuantity::TileElevation(q) => {
                            if *q > max {
                                max = *q;
                                coordinate = ChartedCoordinate::from(c);
                            }
                        }
                        | SavedQuantity::ContentRange(r) => {
                            if r.len() > max {
                                max = r.len();
                                coordinate = ChartedCoordinate::from(c);
                            }
                        }
                    }
                }
                Some((coordinate, max))
            }
        }
    }

    pub fn remove(&mut self, poi: &K, coordinate: ChartedCoordinate) -> Result<(), u8> {
        match self.get_mut(poi) {
            | None => Err(1),
            | Some(found) => {
                for (i, (c, _)) in found.iter().enumerate() {
                    if *c == coordinate {
                        found.remove(i);
                        return Ok(());
                    }
                }
                Err(2)
            }
        }
    }

    pub fn clear(&mut self) {
        self.map.clear()
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
