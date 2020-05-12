# terrain_mapping_rgb

![Screenshot](/files/step_rotation.png)


stepping stone detect & API for gazelle bipedal robot.

## Running

```
$ roscd realsense2_camera/launch/
$ roslaunch rs_d400_and_t265_gazelle.launch
$ roscd gogo_gazelle/src/
$ rosrun gogo_gazelle com_estimate
$ roscd box_detect/src/
$ rosrun box_detect local_plane_pose
$ roscd gogo_gazelle/src/
$ rosrun gogo_gazelle gogo_gazelle
$ rosrun gogo_gazelle test_footstep_vision
```
