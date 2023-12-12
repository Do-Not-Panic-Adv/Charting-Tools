# Charting Tools

> Imagine seamlessly recording and analyzing every location your robot explores, evaluating optimal paths and associated
> costs. But that's not all! what if your robot misses an area? Fear not, our discovery bot is here to uncover
> unexplored
> territories! Whether it's for firefighting, exploration, or any other mission, our code empowers your robot to save,
> assess, and discover with unparalleled efficiency.
> Elevate your robotic experience and redefine the possibilities today!

#### now that the ChatGPT generated ad is over, here is what the tool currently offers:

you can instantiate any of the following utilities:

- ChartedWorld

  Editable vec-based map of discovered tiles. It allows you to store the map your robot has explored,
  but also to edit it by placing any tile at any valid coordinate. This of course doesn't change the actual world,
  but it allows for a fake-copy on which you can conduct calculations (would it be better if I placed a street there?)
  <br><br>
- ChartedMap

  Used to save points of interest and retrieve them later. It uses an hashmap internally to save
    - Content
    - TileType
    - Tile
      and the coordinates at which you found them. You can then query the data structure for any specific
      type of point of interest and get all the places where you found it and in which quantity
      <br><br>
- ChartedPaths

  Pathfinding algorithms. It converts your provided map to a graph and performs shortest path algorithms
  on it
  <br><br>

- ChartingBot

  A way to discover new tiles without moving the robot itself. When instantiated, a ChartingBot id place Robots
  coordinate and performs a given operation of discovery, chosen between 'discovery_line' and 'discovery_path'.
  The discovered tiles are added to the Robots personal map.

Since this is a bundle of utilities, we realized it may be unfair to leave the user total freedom to use
all tools at all times, so there is a system in place to limit the amount of tools used at the same time
(as in, there cannot be more than some amount of charting tools structs alive at the same moment in any thread.
If one instance is dropped, another charting tool can be created at its place)
 
### the current limit is 3, but it is definitely subject to change based on feedback