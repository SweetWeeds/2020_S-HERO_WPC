# 2020_S-HERO_WPC# 2020_S-HERO_WPC

필요 패키지 설치

sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo* ros-kinetic-moveit* ros-kinetic-industrial-core

에러메세지
siop@siop-15N540-RNB7L:~/catkin_ws$ catkin_make
Base path: /home/siop/catkin_ws
Source space: /home/siop/catkin_ws/src
Build space: /home/siop/catkin_ws/build
Devel space: /home/siop/catkin_ws/devel
Install space: /home/siop/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/siop/catkin_ws/build"
####
####
#### Running command: "make -j8 -l8" in "/home/siop/catkin_ws/build"
####
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_MXExt
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_DynamixelState
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_DynamixelStateList
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_DynamixelInfo
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_AX
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_PROExt
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_MX2
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_XL
[  0%] Built target std_msgs_generate_messages_lisp
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_MX
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_EX
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_XM
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_GetDynamixelInfo
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_DynamixelCommand
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_MX2Ext
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_XMExt
[  0%] Built target std_msgs_generate_messages_nodejs
[  0%] Built target std_msgs_generate_messages_eus
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_DynamixelLoadInfo
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_PRO
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_XL320
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_RX
[  0%] Built target _dynamixel_workbench_msgs_generate_messages_check_deps_XH
[  0%] Built target geometry_msgs_generate_messages_nodejs
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_SetKinematicsPose
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_OpenManipulatorState
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_GetKinematicsPose
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_SetJointPosition
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_SetActuatorState
[  0%] Built target geometry_msgs_generate_messages_py
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_JointPosition
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_GetJointPosition
[  0%] Built target geometry_msgs_generate_messages_eus
[  0%] Built target geometry_msgs_generate_messages_lisp
[  0%] Built target geometry_msgs_generate_messages_cpp
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_KinematicsPose
[  0%] Built target rosgraph_msgs_generate_messages_py
[  0%] Built target _open_manipulator_msgs_generate_messages_check_deps_SetDrawingTrajectory
[  0%] Built target roscpp_generate_messages_nodejs
[  0%] Built target roscpp_generate_messages_lisp
[  0%] Built target roscpp_generate_messages_cpp
[  0%] Built target roscpp_generate_messages_eus
[  0%] Built target rosgraph_msgs_generate_messages_nodejs
[  0%] Built target _turtlebot3_msgs_generate_messages_check_deps_VersionInfo
[  0%] Built target _turtlebot3_msgs_generate_messages_check_deps_SensorState
[  0%] Built target _turtlebot3_msgs_generate_messages_check_deps_Sound
[  0%] Built target rosgraph_msgs_generate_messages_eus
[  0%] Built target roscpp_generate_messages_py
[  0%] Built target rosgraph_msgs_generate_messages_cpp
[  0%] Built target rosgraph_msgs_generate_messages_lisp
[  0%] Built target controller_manager_msgs_generate_messages_cpp
[  0%] Built target controller_manager_msgs_generate_messages_eus
[  0%] Built target controller_manager_msgs_generate_messages_nodejs
[  0%] Built target controller_manager_msgs_generate_messages_lisp
[  0%] Built target controller_manager_msgs_generate_messages_py
[  0%] Built target trajectory_msgs_generate_messages_nodejs
[  0%] Built target sensor_msgs_generate_messages_nodejs
[  0%] Built target sensor_msgs_generate_messages_py
[  0%] Built target sensor_msgs_generate_messages_eus
[  0%] Built target sensor_msgs_generate_messages_lisp
[  0%] Built target trajectory_msgs_generate_messages_eus
[  0%] Built target sensor_msgs_generate_messages_cpp
[  0%] Built target trajectory_msgs_generate_messages_py
[  0%] Built target trajectory_msgs_generate_messages_cpp
[  0%] Built target trajectory_msgs_generate_messages_lisp
[  0%] Built target std_srvs_generate_messages_cpp
[  0%] Built target std_srvs_generate_messages_eus
[  0%] Built target std_srvs_generate_messages_lisp
[  0%] Built target std_srvs_generate_messages_py
[  0%] Built target std_srvs_generate_messages_nodejs
[  0%] Built target timer_test
[  0%] Built target bullet_server_gencfg
[  1%] Built target sim_clock_gencfg
[  1%] Built target _bullet_server_generate_messages_check_deps_SetMaterial
[  1%] Built target _bullet_server_generate_messages_check_deps_AddCompound
[  1%] Built target _bullet_server_generate_messages_check_deps_Face
[  1%] Built target _bullet_server_generate_messages_check_deps_Heightfield
[  1%] Built target _bullet_server_generate_messages_check_deps_SetTransform
[  1%] Built target _bullet_server_generate_messages_check_deps_AddRaycast
[  1%] Built target _bullet_server_generate_messages_check_deps_AddConstraint
[  1%] Built target _bullet_server_generate_messages_check_deps_AddLaserScan
[  1%] Built target shape_msgs_generate_messages_cpp
[  1%] Built target _bullet_server_generate_messages_check_deps_Link
[  1%] Built target _bullet_server_generate_messages_check_deps_AddHeightfield
[  1%] Built target _bullet_server_generate_messages_check_deps_SoftConfig
[  1%] Built target _bullet_server_generate_messages_check_deps_AddImpulse
[  1%] Built target _bullet_server_generate_messages_check_deps_Body
[  1%] Built target _bullet_server_generate_messages_check_deps_AddBody
[  1%] Built target _bullet_server_generate_messages_check_deps_Tetra
[  1%] Built target _bullet_server_generate_messages_check_deps_Node
[  1%] Built target shape_msgs_generate_messages_py
[  1%] Built target shape_msgs_generate_messages_nodejs
[  1%] Built target _bullet_server_generate_messages_check_deps_Line
[  1%] Built target _bullet_server_generate_messages_check_deps_Material
[  1%] Built target _bullet_server_generate_messages_check_deps_Impulse
[  1%] Built target _bullet_server_generate_messages_check_deps_SoftBody
[  1%] Built target shape_msgs_generate_messages_eus
[  1%] Built target _bullet_server_generate_messages_check_deps_Constraint
[  1%] Built target _bullet_server_generate_messages_check_deps_Anchor
[  1%] Built target diagnostic_msgs_generate_messages_lisp
[  1%] Built target shape_msgs_generate_messages_lisp
[  1%] Built target diagnostic_msgs_generate_messages_nodejs
[  1%] Built target diagnostic_msgs_generate_messages_cpp
[  1%] Built target diagnostic_msgs_generate_messages_eus
[  1%] Built target diagnostic_msgs_generate_messages_py
[  1%] Built target actionlib_msgs_generate_messages_py
[  1%] Built target actionlib_msgs_generate_messages_lisp
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3ActionResult
[  1%] Built target actionlib_msgs_generate_messages_eus
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3Result
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3Action
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3Feedback
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3ActionFeedback
[  1%] Built target actionlib_msgs_generate_messages_cpp
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3Goal
[  1%] Built target _turtlebot3_example_generate_messages_check_deps_Turtlebot3ActionGoal
[  1%] Built target actionlib_msgs_generate_messages_nodejs
[  1%] Built target nav_msgs_generate_messages_nodejs
[  1%] Built target actionlib_generate_messages_py
[  1%] Built target tf2_msgs_generate_messages_py
[  1%] Built target tf2_msgs_generate_messages_eus
[  1%] Built target tf2_msgs_generate_messages_cpp
[  1%] Built target actionlib_generate_messages_eus
[  1%] Built target tf_generate_messages_cpp
[  1%] Built target tf_generate_messages_nodejs
[  1%] Built target actionlib_generate_messages_lisp
[  1%] Built target tf_generate_messages_eus
[  1%] Built target tf_generate_messages_lisp
[  1%] Built target tf2_msgs_generate_messages_nodejs
[  1%] Built target actionlib_generate_messages_nodejs
[  1%] Built target tf2_msgs_generate_messages_lisp
[  1%] Built target tf_generate_messages_py
[  1%] Built target actionlib_generate_messages_cpp
[  1%] Built target nav_msgs_generate_messages_eus
[  1%] Built target nav_msgs_generate_messages_lisp
[  1%] Built target nav_msgs_generate_messages_py
[  1%] Built target nav_msgs_generate_messages_cpp
[  1%] Built target gazebo_msgs_generate_messages_py
[  1%] Built target dynamic_reconfigure_generate_messages_lisp
[  1%] Built target dynamic_reconfigure_generate_messages_eus
[  1%] Built target dynamic_reconfigure_generate_messages_cpp
[  1%] Built target dynamic_reconfigure_generate_messages_py
[  1%] Built target dynamic_reconfigure_gencfg
[  1%] Built target dynamic_reconfigure_generate_messages_nodejs
[  1%] Built target gazebo_msgs_generate_messages_eus
[  1%] Built target gazebo_msgs_generate_messages_cpp
[  1%] Built target gazebo_ros_gencfg
[  1%] Built target gazebo_msgs_generate_messages_nodejs
[  1%] Built target gazebo_msgs_generate_messages_lisp
[  1%] Built target octomap_msgs_generate_messages_nodejs
[  1%] Built target moveit_msgs_generate_messages_eus
[  1%] Built target octomap_msgs_generate_messages_eus
[  1%] Built target control_msgs_generate_messages_py
[  1%] Built target object_recognition_msgs_generate_messages_eus
[  1%] Built target moveit_msgs_generate_messages_cpp
[  1%] Built target object_recognition_msgs_generate_messages_cpp
[  1%] Built target control_msgs_generate_messages_lisp
[  1%] Built target moveit_msgs_generate_messages_nodejs
[  1%] Built target control_msgs_generate_messages_eus
[  1%] Built target control_msgs_generate_messages_cpp
[  1%] Built target control_msgs_generate_messages_nodejs
[  1%] Built target object_recognition_msgs_generate_messages_nodejs
[  1%] Built target object_recognition_msgs_generate_messages_lisp
[  1%] Built target octomap_msgs_generate_messages_lisp
[  1%] Built target object_recognition_msgs_generate_messages_py
[  1%] Built target octomap_msgs_generate_messages_cpp
[  1%] Built target moveit_msgs_generate_messages_py
[  1%] Built target moveit_msgs_generate_messages_lisp
[  1%] Built target octomap_msgs_generate_messages_py
[  1%] Built target moveit_ros_manipulation_gencfg
[  1%] Built target flat_world_imu_node
[  1%] Built target visualization_msgs_generate_messages_py
[  1%] Built target visualization_msgs_generate_messages_eus
[  1%] Built target visualization_msgs_generate_messages_cpp
[  1%] Built target visualization_msgs_generate_messages_lisp
[  1%] Built target visualization_msgs_generate_messages_nodejs
[  1%] Built target open_manipulator_controllers
[  1%] Built target moveit_ros_planning_gencfg
[  1%] Built target open_manipulator_gazebo_xacro_generated_to_devel_space_
[  1%] Built target control_toolbox_generate_messages_nodejs
[  1%] Built target control_toolbox_generate_messages_lisp
[  1%] Built target control_toolbox_generate_messages_cpp
[  1%] Built target control_toolbox_generate_messages_eus
[  1%] Built target control_toolbox_gencfg
[  1%] Built target control_toolbox_generate_messages_py
[  1%] Built target turtlebot3_description_xacro_generated_to_devel_space_
[  6%] Built target dynamixel_workbench_msgs_generate_messages_eus
[ 11%] Built target dynamixel_workbench_msgs_generate_messages_lisp
[ 15%] Built target dynamixel_workbench_msgs_generate_messages_cpp
[ 20%] Built target dynamixel_workbench_msgs_generate_messages_nodejs
[ 25%] Built target dynamixel_workbench_msgs_generate_messages_py
[ 27%] Built target open_manipulator_msgs_generate_messages_nodejs
[ 30%] Built target open_manipulator_msgs_generate_messages_py
[ 32%] Built target open_manipulator_msgs_generate_messages_eus
[ 35%] Built target open_manipulator_msgs_generate_messages_lisp
[ 36%] Built target turtlebot3_msgs_generate_messages_cpp
[ 39%] Built target open_manipulator_msgs_generate_messages_cpp
[ 40%] Built target turtlebot3_msgs_generate_messages_py
[ 40%] Built target turtlebot3_msgs_generate_messages_lisp
[ 41%] Built target turtlebot3_msgs_generate_messages_nodejs
[ 42%] Built target turtlebot3_msgs_generate_messages_eus
[ 44%] Built target dynamixel_sdk
[ 44%] Built target joint_operator
[ 45%] Built target open_manipulator_teleop_keyboard
[ 46%] Built target wheel_operator
[ 46%] Built target open_manipulator_teleop_joystick
[ 47%] Built target clock
[ 48%] Built target robotis_manipulator
[ 53%] Built target bullet_server_generate_messages_cpp
[ 59%] Built target bullet_server_generate_messages_py
[ 61%] Built target open_manipulator_control_gui
[ 68%] Built target bullet_server_generate_messages_eus
[ 73%] Built target bullet_server_generate_messages_lisp
[ 78%] Built target bullet_server_generate_messages_nodejs
[ 80%] Built target turtlebot3_example_generate_messages_py
[ 81%] Built target turtlebot3_example_generate_messages_lisp
[ 82%] Built target turtlebot3_diagnostics
[ 84%] Built target turtlebot3_example_generate_messages_eus
[ 86%] Built target turtlebot3_example_generate_messages_cpp
[ 88%] Built target turtlebot3_example_generate_messages_nodejs
[ 88%] Built target turtlebot3_drive
[ 88%] Built target turtlebot3_fake_node
[ 89%] Built target turtlebot3_manipulation_bringup
[ 89%] Built target dynamixel_workbench_msgs_generate_messages
[ 90%] Built target omx_gripper_sub_publisher
[ 90%] Built target turtlebot3_msgs_generate_messages
[ 90%] Built target open_manipulator_msgs_generate_messages
[ 90%] Built target bullet_server_gencpp
[ 91%] Built target dynamixel_workbench_toolbox
[ 91%] Built target bullet_server_generate_messages
[ 91%] Built target turtlebot3_example_generate_messages
[ 92%] Built target find_dynamixel
[ 93%] Built target dynamixel_workbench_controllers
[ 94%] Built target omx_control_node
[ 95%] Built target open_manipulator_libs
[ 97%] Built target turtlebot3_manipulation_gui
[ 97%] Building CXX object simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/src/raycast.cpp.o
[ 98%] Building CXX object simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/src/soft_body.cpp.o
Scanning dependencies of target open_manipulator_controller
[ 98%] Building CXX object open_manipulator/open_manipulator_controller/CMakeFiles/open_manipulator_controller.dir/src/open_manipulator_controller.cpp.o
/home/siop/catkin_ws/src/simple_sim_ros/bullet_server/src/raycast.cpp:29:49: fatal error: tf2_geometry_msgs/tf2_geometry_msgs.h: No such file or directory
compilation terminated.
simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/build.make:134: recipe for target 'simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/src/raycast.cpp.o' failed
make[2]: *** [simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/src/raycast.cpp.o] Error 1
make[2]: *** Waiting for unfinished jobs....
CMakeFiles/Makefile2:6441: recipe for target 'simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/all' failed
make[1]: *** [simple_sim_ros/bullet_server/CMakeFiles/bullet_server.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
[100%] Linking CXX executable /home/siop/catkin_ws/devel/lib/open_manipulator_controller/open_manipulator_controller
[100%] Built target open_manipulator_controller
Makefile:138: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j8 -l8" failed
siop@siop-15N540-RNB7L:~/catkin_ws$
