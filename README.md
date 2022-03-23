# Spatial Maths for ROS

This ROS package is a collection of conversion functions that wrap the [Spatial Maths for Python package](https://github.com/petercorke/spatialmath-python).

## Installing

Clone the package into a Catkin workspace:

```
u@pc:~/catkin_ws/src$ git clone https://github.com/qcr/spatialmath-ros
```

And then install as usual using `catkin_make`:

```
u@pc:~/catkin_mws$ catkin_make
```

## Usage

We use a consistent naming system in all functions for predictable usage:

```python
output = <input_identifier>_to_<output_identifier>(input)
```

Where:

- `output` is an object of type corresponding to `output_identifier`
- `input` is an object of type corresponding to `input_identifier`

For example, the following creates an SE3 object corresponding to a ROS pose message:

```python
my_SE3 = pose_msg_to_SE3(my_pose_msg)
```

The following tables list the supported identifiers.

### ROS Identifiers

| Identifier | ROS Message Type           |
| ---------- | -------------------------- |
| `pose_msg` | `geometry_msgs/Pose`       |
| `quat_msg` | `geometry_msgs/Quaternion` |
| `tf_msg`   | `geometry_msgs/Transform`  |

### Spatial Maths for Python Identifiers

| Identifier | Spatial Maths package Type   |
| ---------- | ---------------------------- |
| `quat`     | `spatialmath.UnitQuaternion` |
| `SE3`      | `spatialmath.SE3`            |
| `SE2`      | `spatialmath.SE2`            |
