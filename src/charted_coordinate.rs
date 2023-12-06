use std::fmt::{Display, Formatter};
use std::ops::{Add, Sub};

use robotics_lib::world::coordinates::Coordinate;

#[derive(Copy, Clone, Debug)]
/// struct: ChartedCoordinate
///
/// it is simply a custom type compatible with robotics_lib::world::coordinates::Coordinate,
/// implemented for more liberty and ease of use inside this crate
///
/// ## Usage:
///
///     let c = ChartedCoordinate::default();
///
///     let c = ChartedCoordinate::new(0, 1);
///
///     let c = ChartedCoordinate::from(robot.coordinate);
///
///     let tuple = (0usize, 0usize);
///     let c = ChartedCoordinate::from(tuple);
///
pub struct ChartedCoordinate(pub usize, pub usize);

impl Default for ChartedCoordinate {
    fn default() -> Self {
        ChartedCoordinate::new(0, 0)
    }
}

impl From<&Coordinate> for ChartedCoordinate {
    fn from(value: &Coordinate) -> Self {
        Self(value.get_row(), value.get_col())
    }
}

impl From<&ChartedCoordinate> for ChartedCoordinate {
    fn from(value: &ChartedCoordinate) -> Self {
        Self(value.0, value.1)
    }
}

impl From<(usize, usize)> for ChartedCoordinate {
    fn from(value: (usize, usize)) -> Self {
        Self(value.0, value.1)
    }
}

impl Display for ChartedCoordinate {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}", self.0, self.1)
    }
}

impl ChartedCoordinate {
    pub fn new(row: usize, col: usize) -> Self {
        Self(row, col)
    }

    pub fn get_row(&self) -> usize {
        self.0
    }

    pub fn get_col(&self) -> usize {
        self.1
    }

    // pub fn get_distance(&self, other:&Self)->(usize,usize){
    //     (other.0-self.0, other.1-self.1)
    // }
    //
    // pub fn is_close_to(&self, other: &Self)->bool{
    //     if ((self.get_distance(other).0).pow(2) + (self.get_distance(other).1).pow(2))<2 { true }
    //     false
    // }
}

impl Add for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl Add<Coordinate> for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn add(self, rhs: Coordinate) -> Self::Output {
        Self(self.0 + rhs.get_row(), self.1 + rhs.get_col())
    }
}

impl Add<(usize, usize)> for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn add(self, rhs: (usize, usize)) -> Self::Output {
        Self(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl Sub for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0, self.1 - rhs.1)
    }
}

impl Sub<Coordinate> for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn sub(self, rhs: Coordinate) -> Self::Output {
        Self(self.0 - rhs.get_row(), self.1 - rhs.get_col())
    }
}

impl Sub<(usize, usize)> for ChartedCoordinate {
    type Output = ChartedCoordinate;

    fn sub(self, rhs: (usize, usize)) -> Self::Output {
        Self(self.0 - rhs.0, self.1 - rhs.1)
    }
}