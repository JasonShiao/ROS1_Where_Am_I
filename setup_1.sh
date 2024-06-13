# Make sure you've already in this folder (workspace)
mkdir src
cd src
catkin_init_workspace

catkin_create_pkg my_robot
cd my_robot
mkdir launch
mkdir worlds

# Create an empty Gazebo World
cd worlds
touch empty.world

cat << 'EOF' >> empty.world
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
EOF

cd ..
cd launch
touch world.launch

cat << 'EOF' >> world.launch
<launch>

  <!-- World File -->
  <arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

</launch>
EOF

cd ../../.. # Back to the workspace folder layer
catkin_make

# To launch
#source devel/setup.bash
#roslaunch my_robot world.launch
