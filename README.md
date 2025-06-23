# Turtlesim "Catch Them All" Project
This project implements a ROS 2-based simulation using the ``turtlesim`` package, where a master turtle ("turtle1") navigates to catch other turtles spawned randomly on the screen. The project consists of three nodes: ``turtlesim_node``, ``turtle_controller``, and ``turtle_spawner``, housed in a custom package named ``turtlesim_catch_them_all``.
## Project Overview
**Nodes**

1. **turtlesim_node:** Provided by the turtlesim package, it spawns the master turtle ("turtle1") and offers services like /spawn and /kill.
2. **turtle_controller:** A custom node that controls "turtle1" to chase and catch other turtles using a proportional (P) controller.
3. **turtle_spawner:** A custom node that spawns new turtles, tracks alive turtles, and handles their removal when caught.

**Custom Interfaces** (in ``my_robot_interfaces`` package)

- **Turtle.msg:** Defines a turtle with name and coordinates (x, y).
- **TurtleArray.msg:** Contains an array of ``Turtle`` messages for the ``/alive_turtles`` topic.
- **CatchTurtle.srv:** Service to send the name of a caught turtle from ``turtle_controller`` to ``turtle_spawner``.

**Functionality**

- **turtle_spawner:**
    - Spawns turtles at random coordinates (x, y between 0.0 and 11.0) using the ``/spawn`` service.
    - Publishes the list of alive turtles (name + coordinates) on the ``/alive_turtles`` topic.
    - Provides a ``/catch_turtle`` service to remove a caught turtle via the ``/kill`` service and update the alive turtles list.


- **turtle_controller:**
    - Subscribes to ``/turtle1/pose`` to track the master turtle’s position.
    - Publishes velocity commands to ``/turtle1/cmd_vel`` using a P controller to reach a target turtle.
    - Subscribes to ``/alive_turtles`` to select a target turtle (initially the first, later the closest).
    - Calls the ``/catch_turtle`` service when a turtle is caught.



## Development Steps

1. **Step 1: turtle_controller Node**

- Subscribe to ``/turtle1/pose`` to get the master turtle’s position.
- Implement a control loop (using a timer) to reach an arbitrary target point.
- Calculate distance and angle to the target, then publish velocity commands to ``/turtle1/cmd_vel`` using a P controller.


2. **Step 2: turtle_spawner Node**

- Create a timer to periodically call the ``/spawn`` service, spawning turtles at random coordinates.


3. **Step 3: Alive Turtles Tracking**

- In ``turtle_spawner``, maintain an array of alive turtles and publish it on ``/alive_turtles``.
- In ``turtle_controller``, subscribe to ``/alive_turtles`` and select the first turtle as the target.


4. **Step 4: Catch Turtle Service**

- Implement the ``/catch_turtle`` service in ``turtle_spawner`` to handle turtle removal.
- When ``turtle_controller`` reaches a turtle, call ``/catch_turtle`` with the turtle’s name.
- In ``turtle_spawner``, call ``/kill``, remove the turtle from the array, and publish the updated array.


5. **Step 5: Closest Turtle Selection**

- Modify ``turtle_controller`` to select the closest turtle from ``/alive_turtles`` instead of the first one.


6. **Step 6: Parameters and Launch File**

- Add parameters (e.g., spawn rate, controller gains) to both nodes.
- Create a launch file and a YAML parameter file to configure the nodes.


## Prerequisites

- ROS 2 (Humble or later recommended)
- ``turtlesim`` package
- Custom interfaces in ``my_robot_interfaces`` package

## Installation

1. Create a ROS 2 workspace:
````
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
````

2. Clone or create the ``turtlesim_catch_them_all`` package:
````
ros2 pkg create --build-type ament_cmake turtlesim_catch_them_all
````

3. Build the workspace:
````
cd ~/ros2_ws
colcon build
source install/setup.bash
````

## Running the Project

1. Launch the project:
````
ros2 launch turtlesim_catch_them_all catch_them_all_launch.py
````

2. Observe the ``turtlesim`` window where "turtle1" chases and catches other turtles.

## Parameters
Defined in ``config/params.yaml``:

- ``turtle_spawner``:
    - ``spawn_rate``: Frequency of spawning new turtles (e.g., 0.5 Hz).


- ``turtle_controller``:
    - ``linear_gain``: P controller gain for linear velocity.
    - ``angular_gain``: P controller gain for angular velocity.



## Future Improvements
- Add obstacle avoidance for turtles.
- Implement a more advanced controller (e.g., PID).
- Visualize caught turtles’ paths.

## Contact
For questions or suggestions, please open an issue on the GitHub repository or contact [shamreen.tabassum@mailbox.tu-dresden.de].

