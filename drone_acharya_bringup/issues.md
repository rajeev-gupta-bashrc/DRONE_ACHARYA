# Issues:

## 01/08/24
### ros_gz_bridge for ApplyJointForce plugin doesn't work
```
<plugin filename="gz-sim-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
  <joint_name>iris_with_standoffs::rotor_0_joint</joint_name>
</plugin>
```
#### tried JointController
```
    <plugin filename="gz-sim-joint-controller-system"
      name="gz::sim::systems::JointController">
      <joint_name>rotor_0_joint</joint_name>
      <initial_velocity>0.0</initial_velocity>
      <topic>/custom_drone/rotor_0_joint/cmd_roll</topic>
    </plugin>
```
Note: use this plugin directly to the model, don't use under namespace, use merge="true" under the model include statement.
Failed: 
```
[parameter_bridge-3] [WARN] [1722516803.361026682] [ros_gz_bridge]: Failed to create a bridge for topic [/drone_v1/rotor_0_joint/cmd_roll] with ROS2 type [std_msgs/msg/Float32] to topic [/drone_v1/rotor_0_joint/cmd_roll] with Gazebo Transport type [gz.msgs.Double]

```


