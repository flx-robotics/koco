# Koco

Core ROS packages of the Flexible Control Framework for Industrial Robotic Applications, where Koco stands for "**K**oco **o**ffers **co**mpatibility".  

## Packages
* `koco`: The meta package.
* `koco_core_flexbe_states`: High-level FlexBE states for composing complex behaviors, including:
  * States to program robot motions
  * States to interact with permanent storage
* `koco_nodes`: Nodes that offer additional functionality of the Koco system:
  * Interface to link the permanent storage (database) to TF
* `koco_services`: Nodes that offer services to interact with the Koco system:
  * Service for requesting and storing complex trajectory plans through MoveIt
* `koco_utils`: A set of useful Python libraries that provide handy methods and classes for programming robotic tasks:
  * Action interface template
  * Pose and message conversions
  * Geometric distances and averages
  * TF and Database proxies
  * ROS path resolution
