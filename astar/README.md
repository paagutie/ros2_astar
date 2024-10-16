# astar
===
  
## 1. Introduction
This package is the ROS2 adaptation of the A-Star algorithm found [here](https://github.com/lh9171338/Astar).

## 2. Usage
### 2.1 Subscribed Topics  
map([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/humble/p/nav_msgs/interfaces/msg/OccupancyGrid.html))    
- Receive the map via this topic.

initialpose ([geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/humble/p/geometry_msgs/interfaces/msg/PoseWithCovarianceStamped.html))  
- Receive the start point via this topic. Active if ros_service = false

goal_pose ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/humble/p/geometry_msgs/interfaces/msg/PoseStamped.html))   
- Receive the target point via this topic. Active if ros_service = false

### 2.2 Published Topics  
mask ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/humble/p/nav_msgs/interfaces/msg/OccupancyGrid.html))  
- Publish the inflation map(mask) via this topic.  

nav_path ([nav_msgs/msg/Path](https://docs.ros.org/en/humble/p/nav_msgs/interfaces/msg/Path.html))  
- Publish the navigation path via this topic.

initialpose ([geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/humble/p/geometry_msgs/interfaces/msg/PoseWithCovarianceStamped.html))  
- Send the start point via this topic. Active if ros_service = true.

goal_pose ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/humble/p/geometry_msgs/interfaces/msg/PoseStamped.html))   
- Send the target point via this topic. Active if ros_service = true.

### 2.3 Services
astar/get_path (path_msgs/srv/GetPath)
- Receive start and goal positions and return the path found.

### 2.4 Parameters  
~Euclidean(bool; default: "true")  
- Using Euclidean distance or Manhattan distance When calculating the H value.

~OccupyThresh(int; default: -1)  
- Threshold of the image binarization. When OccupyThresh is less than zero(OccupyThresh < 0), using Otsu method to generate threshold.

~InflateRadius(double; defalut: -1)  
- InflateRadius is the inflation radius(unit: m). When InflateRadius is less than or equal to zero(InflateRadius <= 0), no inflation operation is taken.

~ros_service(bool; default: true)  
- Use the path_msgs/srv/GetPath service instead of using subscribers.

~rate(int; default: 10)  
- The rate of publishing path and mask topic when ros_service = false.

- The parameters are included in the [params.yaml](config/params.yaml) file.

### 2.5 Example  
```
ros2 launch astar astar.launch.py
```



