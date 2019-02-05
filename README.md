# tracking_robots



## track_robot

Simple package which uses a pendant switch to act as a switch for the node.

Host Computer <-> (rosserial_arduino) <-> Arduino Uno <-> Pendant Switch

```
roslaunch track_robot track_robot.launch
```

### Parameters

endpoint_topic_name: name of the topic you want to rosbag record when switch is engaged

trigger_topic_name: name of the topic which the trigger (using track_robot_msgs/Trigger.msg) comes from (look at rosserial_Arduino)

rosbag_name: name of saved rosbag (and its location)

trigger_buffer: how many callbacks before node ends once switch is off
