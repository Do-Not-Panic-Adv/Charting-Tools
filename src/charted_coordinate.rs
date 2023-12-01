use std::fmt::{Display, Formatter};
use robotics_lib::world::coordinates::Coordinate;

#[derive(Clone, Debug)]
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
pub struct ChartedCoordinate(usize, usize);

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
}
