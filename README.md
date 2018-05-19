![logo](https://flytte.github.io/logo.png)

Flytte is an application for declarative drone control.

It has the goal to improve the user experience by controlling the drones from the 3rd person point of view. The system combines an intuitive user interface based on Augmented Reality techniques, Artificial Neural Networks that are responsible for online drone's pose detection and ROS-based core unit.

Right now it is only possible to use Flytte with [Sphinx](https://developer.parrot.com/docs/sphinx/index.html), which is a simulator for [Parrot](https://www.parrot.com/) drones.

# Dependencies

### Rosbridge
[ROS Wiki](http://wiki.ros.org/rosbridge_suite)

**Installation**:
> `sudo apt-get install ros-\<rosdistro\>-rosbridge-server`

### Bebop Autonomy
[How to install](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)

### Web Video Server
[ROS Wiki](http://wiki.ros.org/web_video_server)

_OpenCV3 needs to be installed in the system_

**Installation**:
> `sudo apt-get install ros-\<rosdistro\>-web-video-server`

### Sphinx
[How to install](https://developer.parrot.com/docs/sphinx/installation.html)

[How to use](https://developer.parrot.com/docs/sphinx/firststep.html)

# How to start
1. Build the application
> `git clone https://github.com/Flytte/flytte.git`  
> `cd flytte/backend/drone_ros_ws`  
> `ln -s /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt`  
> `catkin_make`  
> `source devel/setup.bash`

2. Start the simulator
> `sphinx backend/simulation/empty.world backend/simulation/bebop2.drone`

_The firmwared should be running._

_The default name of the network interface is_ `eth0`_. To know more, look [here](https://developer.parrot.com/docs/sphinx/firststep.html)._

3. Start the Rosbridge
> `roslaunch rosbridge_server rosbridge_websocket.launch`

4. Start the Web Video Server
> `rosrun web_video_server web_video_server`

5. Start the Bebop Autonomy driver
> `roslaunch bebop_driver bebop_node.launch`

6. Start the Flytte backend
> `roslaunch drone_app drone_app.launch`

7. Open Flytte in the browser

You don't really need a web server for this. Just open the `index.html` file.
> `file:///<frontend folder>/index.html`

_Note that Flytte has a special mode for a simulated environment. In this mode the virtual camera in the simulator is used instead of the real one on the client. To start Flytte in simulation mode simple add_ `?simulation` _after the URL._
> `file:///<frontend folder>/index.html?simulation`
