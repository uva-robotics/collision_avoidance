## Connecting to Pepper

<b>Make sure you are always connected to the wifi-network 'robolab'!!</b>

Run the following code in your uva-robotics folder:

`source main/set_network.sh 146.50.60.54`


(if all went well running `rostopic list` should output a large list of all topics)

## Creating collision_avoidance package
`cd uva-robotics/src`

`git clone https://github.com/uva-robotics/collision_avoidance` clone this repository in your src folder

The files 'CMakeLists.txt', 'package.xml' and 'setup.py' have been changed (using the boilerplate templates) to setup the collision_avoidance package.

In order to make the package you need to run the following code:

`catkin_make`

`chmod +x src/collision_avoidance/main.py`

`source devel/setup.bash`

## Running code

`rosrun collision_avoidance main.py`  