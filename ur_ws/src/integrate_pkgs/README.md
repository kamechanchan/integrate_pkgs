### integration_run(土田)
```
roslaunch tf_publish spawn_object.launch object_name:=sekai_small_box_and_50
rosrun w2d_to_3d_ros move_saikyou_object.py 
roslaunch cloud_practice planar_segmentation.launch
roslaunch all_estimator yolo_first_input.launch
roslaunch all_estimator integ_second.launch
roslaunch all_estimator integ_third.launch
roslaunch estimator color_get.launch
roslaunch estimator pcl_pub.launch 
```

### integration_run(ishiyama_deocclusion)
```
roslaunch tf_publish spawn_object.launch object_name:=sekai_small_box_and_50
rosrun w2d_to_3d_ros move_saikyou_object.py 
roslaunch cloud_practice planar_segmentation.launch
rosrun all_estimator yolo_server_2D 
roslaunch all_estimator yolo_first_input.launch
rosrun all_estimator integ_client.py
rosrun all_estimator SemSeg_2D.py --save-path
rosrun all_estimator Deocclusion.py
roslaunch all_estimator integ_second.launch
roslaunch estimator color_get.launch
roslaunch estimator pcl_pub.launch 
```

### integration_run(最新(誤差計算))
```
roslaunch setup_simulator ur_and_phoxi_sensor_setup.launch
rosrun setup_simulator box_and_object_move.py
roslaunch all_estimator yolo_first_input.launch
roslaunch all_estimator integ_second.launch
roslaunch all_estimator integ_third.launch
rosrun all_estimator integ_client.py
```

### integration_run(最新)
```
roslaunch tf_publish spawn_object.launch object_name:=sekai_small_box_and_50
rosrun setup_simulator box_and_object_move.py
roslaunch all_estimator yolo_first_input.launch
roslaunch all_estimator integ_second.launch
roslaunch all_estimator integ_third.launch
roslaunch cloud_practice planar_segmentation.launch
rosrun all_estimator quaternion_convert
rosrun all_estimator integ_client.py
roslaunch estimator color_get.launch 
roslaunch estimator pcl_pub.launch
```