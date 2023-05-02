## execute rviz and gazebo
```
roslaunch ur5_moveit demo_gazebo.launch
```
## sample for points of certain range of marker, move end_effector to sampled points
```
rosrun ur5_moveit sampleforrange
```
## check can see and identify the marker or not
```
rosrun ur5_moveit publisharucotfsimu.py
```
## if can identify, save the data as calibration required
```
rosrun ur5_moveit getdata
```
## add to .bashrc
```
export GAZEBO_MODEL_PATH=~/<workspace_name>/src/ur5_moveit/aruco_gazebo:{GAZEBO_MODEL_PATH}
```

## add to package.xml
```
<run_depend>gazebo_ros</run_depend> 
<export> 
    <gazebo_ros gazebo_model_path="${prefix}/aruco_gazebo"/> 
    <gazebo_ros gazebo_media_path="${prefix}/aruco_gazebo"/> 
</export>
```
## execute
```
roslaunch ur5_moveit demo_gazebo.launch
rosrun ur5_moveit sampleforrange
```
