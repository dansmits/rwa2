Package: rwa2-Smits
Nodes: my_bot_controller.py
Launch: my_bot_controller.launch

Program Summary: This package provides the capability to navigate TurtleBot3 through the
  turtlebot3_world in Gazebo using the lidar sensors on the robot for collision avoidance to reach 5 pre-determined
  navigational points:
    Pillar 1 at (1,-2)
    Pillar 2 at (-1,2)
    Pillar 3 at (2,0)
    Pillar 4 at (-1,-2)
    Pillar 5 at (1,2)
  After the robot reaches the 5th point the user will be notified and the application will terminate.

  Instructions:
    1. Extract and Copy package to /catkin_ws/src directory
    2. Run: catkin_make
    3. Run: rospack profile
    4. Run: roslaunch rwa2-Smits my_bot_controller.launch
    6. Sit back, relax, and watch TurtleBot3 travel to pillars 1-5 with ninja like precision using lidar obstacle
       avoidance ;-)
    7. application will terminate at completion (including Gazebo) and return to terminal, as it should.
