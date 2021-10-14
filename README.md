# store-planner

## Description
This repository contains ROS Code to manage planning in retail stores.

## Prerequisites
### TIAGo-base setup
* Follow the official installation of the robot [here](http://wiki.ros.org/Robots/PMB-2/Tutorials/Installation/PMB2Simulation) and install it into the ROS workspace.
* Clone this repository in the same workspace and build both
* Comment and add/modify the line as shown
```bash
<!--  <xacro:include filename="$(find pmb2_description)/urdf/base/base.urdf.xacro"/> -->
<xacro:include filename="$(find store-planner)/robot_description/base.urdf.xacro"/>
```
in the file 
```bash
.../pmb2_public_ws/src/pmb2_robot/pmb2_description/urdf/base/base_sensors.urdf.xacro
```
This enables to use our robot model consisting into the TIAGo-base plus three additional RGBD cameras mounted on top.
* modify as you please the tolerances for the final pose of the robot on a given goal: 

```bash
# GoalTolerance
  xy_goal_tolerance: 0.2
  yaw_goal_tolerance: 0.2
```
in the file located at
```bash
.../pmb2_public_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_pmb2/config/base/teb/
```
To have more flexibility on the robot, I suggest to set ```yaw_goal_tolerance: 6.28``` so any yaw is allowed. This setup won't affect the final pose of the robot once on the goal.

* Use the following command to spawn the robot within the store map
```bash
roslaunch store-planner visual.launch
```
At this step only the robot is present and the resulting world is empty. By following the next steps, a custom world will be created and it will be shown with the above command together with the robot.

---

## Usage

### Navigation
The navigation consists into sending a sequence of waypoints to the robot where it will perform the shelf captures.
To make the robot navigate inside a new simulated store it is required to run two scripts (only the first time). 

First of all it is necessary to place the heatmap (an ```.csv``` file) inside the ```store_csv``` folder. This file should contain cells coordinates (x,y), cells type ("Wall", "Shelf", "Free") and cells intensity of the query store.

#### HeatMap generation
The ```generate_heatmap``` script takes as input the csv file and transform it into an heatmap by performing some image processing on it. This script is interactive since a visual feedback from the user is required.
The output consists into three different images: the static map, the hotmap and the coldmap.

Usage example:
```bash
cd ..../store_planner/scripts
python generate_heatmap.py -csv <csv_filename>
```
The csv file must be under the store_csv folder
The static map will be placed under the subfolder ```maps/<csv_filename>/``` together with all the map files required for the navigation under ROS. The hotmap and the coldmap will be under the ```heatmpaps/<csv_filename>``` subfolder. Their usage will be explained next.
For this step it is possible to select the desired filter type (default is Majority) together with its kernel size to smooth the raw heatmap coming from the data on the csv file.

#### Navigation Points generation

##### QuadTree Decomposition
To generate the waypoints for the robot, it is necessary to run two different scripts. The first one is a MATLAB script and it is responsible for the quadtree decomposition of the hotmap and coldmap. Also this script needs to be run only when a new map is used.
Usage:
```bash
cd ..../store_planner/scripts
matlab -nodisplay -r "quadtree_decomp('filename')"
```
the above command will execute the matlab script without the display option and will exit automatically.
It will generate a list of blocks holding the navigation waypoints for both the hotmap and coldmap.

##### Final points generation
The last script to be run is the Point of Interest generator. This python script takes as input the store folder and retrieves its heatmap previously generated, together with the waypoints coming from the quadtree decomposition.
It outputs an optimized sequence of filtered waypoints (under the folder results) on which the robot has to navigate and perform the shelf captures with the on-board cameras.

Usage Example:
```bash
cd ..../store_planner/scripts
python POIs_generator.py -f <folder_name>
```
Together with the points calculation (which is automatic), the user has the possibility to select the shelves on which the robot has to perform the captures and the forbidden areas (namely the areas on which the robot is not allowed to pass or simply areas which are not of interest from an inventory point of view).

**Remark: these scripts have to be just the first time a new map (or store) is being taken into account and they have to be run offline.**

#### Generate a Gazebo world
To simulate a real environment (i.e. a real store) in Gazebo, it is necessary to generate a mesh of the store (a ```.dae``` file).
It is possible to generate this mesh starting from the binary image (static map) of the store which has been inferred with the first script.

For this work, to convert the image into a mesh we have used the [png23d](http://manpages.ubuntu.com/manpages/bionic/man1/png23d.1.html) tool.

Usage Example:
```bash
png23d <input_img> <output_mesh> -o stl -d 2
```
Using this tool , the resulting mesh dimensions are given in pixels. To adjust the ratios and the extension (required for Gazebo) a 3D mesh processing tool such as [MeshLab](https://www.meshlab.net/) could be used.

---

## Navigation Step
Once both the static map and the world are available, it is possible to navigate inside the store. To this aim, it is possible to launch the navigation step with the following command:
```bash
roslaunch store-planner nav_store.launch 
```

This will launch the ROS node responsible for the navigation (```store_planner_node.py```). All the navigation points (together with those where the shelves have to be captured) are sent directly by the node to the ```move_base``` package which will be responsible also for path planning and obstacle avoidance. 

Once the robot reaches the ith waypoint, it will rotate such to face with the side cameras the query shelf and will perform the capture with the cameras properly synchronized through ROS.
Visual results will be saved under the subfolder acquisitions. Results are composed by the three images plus the camera informations.

The parameters to be changed inside the file config/params.json are: the initial robot position (depot or charging station) and the desired planning mode.
Mode = 0 -> customer behavioural mimic
Mode = 1 -> customer avoidance
The map resolution must match the one inside the map parameter for rviz (map.yaml).

**Remember to change the launch file to have the correct folder name**

---

### Additional notes

The pixel to meter ratio must be known in advance. In this respect,

Pal Robotics uses a custom Rviz plugin for displaying covariance ellipses that is out-of-date. To avoid errors on the simulations (this however does not impact the simulation itself, just visualization) you should install an additional package in the workspace and build it
```bash
cd ~/<your_workspace>/src/pmb2_public_ws/src/
git clone https://github.com/pal-robotics/rviz_plugin_covariance.git
cd ~/<your_workspace>
catkin build
```