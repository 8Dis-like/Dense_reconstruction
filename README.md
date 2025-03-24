# Dense_reconstruction

This is a ROS package for stereo dense reconstruction tailored for ORBSLAM

Please compile both nodes with CMakeLists.txt in your local workspace "X_ws".

Then, run the tailored ORBSLAM3 stereo [here](https://github.com/8Dis-like/ORB_SLAM3/tree/master). 

While ORBSLAM3 is running, you can run each node with following command.



```bash
rosrun dense_reconstruction dense_reconstruction
```

```bash
rosrun dense_reconstruction point_cloud_saver
```

