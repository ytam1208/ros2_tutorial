# ROS2 
## 1. Create Workspace command
```
mkdir -p ros2_ws/src
cd ros2_ws/src
```

## 2. Create Package command
```
ros2 pkg create [pkg_name] --build-type [build type] --node-name [node_name] --license Apache-2.0

******************  ******************
**  build type  **  ** license type **
******************  ******************
*   ament_cmake  *  *   Apache-2.0   *
*   ament_python *  *      ...       *
******************  ******************
```

## 3. Build a package command
```
cd ~/workspace
colcon build
```
#### 3.1 Option
```
colcon build --packages-select [pkg_name]
colcon build --symlink-install
```

## Turotial Code
### 1. Publisher
[Publisher](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/publisher/member_function.cpp)

[Publisher use Lambda](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/publisher/Lambda_member_function.cpp)

[Publisher with type adapter](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/publisher/member_function_with_type_adapter.cpp)

[Publisher with unique network flow endpoints](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/publisher/member_function_with_unique_network_flow_endpoints.cpp)

[Publisher with wait for all acked](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/publisher/member_function_with_wait_for_all_acked.cpp)


### 2. Subscriber
[Subscriber](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/subscriber/member_function.cpp)

[Subscriber use Lambda](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/subscriber/Lambda_member_function.cpp)

[Subscriber with type adapter](https://github.com/ytam1208/ros2_tutorial/blob/main/src/cpp_pubsub/src/subscriber/member_function_with_type_adapter.cpp)

[Subscriber with wait set]()
