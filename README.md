# Kinematic-Planning-under-Non-Holonomic-Conditions

The objective is to create a simulated world and park three vehicles into a compact space. The vehicle kinematics and collision need to be taken into consideration by the path planning algorithm. Kinematic planning under non-holonomic constraints have been used in this assignment for efficient parking of the vehicles. A* algorithm has been used as the planner algorithm for the cars in this project. The three vehicles implemented in this project are:

1)	A di-wheel configured delivery robot.
2)	A standard car using Ackermann’s steering principle.
3)	A truck with a trailer attached.


Set the project directory as the workspace,

`cd ./Kinematic-Planning-under-Non-Holonomic-Conditions`

To simulate the di-wheel delivery robot, 

`python main.py --robotType 1`

The simualation of the di-wheel delivery robot and the path taken is shown below:

<!-- ![Alt Text](animations/delivery_robot.gif) -->
<img src="animations/delivery_robot.gif" alt="Delivery Robot" width="400">

To simulate the standard car using Ackermann's principle,

`python main.py --robotType 2`

The simualation of the standard car and the path taken is shown below:

<img src="animations/car.gif" alt="Car" width="400">


To simulate the truck with a trailer attached, 

`python main.py --robotType 3`

The simualation of the truck and the path taken is shown below:

<!-- <img src="animations/truck.gif" alt="Truck" width="400"> -->
<!-- <div style="display: flex;">
  <img src="animations/truck.gif" alt="Truck" width="50%">
  <img src="paths/truck.png" alt="Truck" width="50%">
</div> -->
<img src="animations/truck.gif" alt="Truck" style="float: left; margin-right: 10px;" width="50%">
<img src="paths/truck.png" alt="Truck" style="float: right; margin-right: 10px;" width="50%">

