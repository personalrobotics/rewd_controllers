# rewd_controllers

Inverse dynamics controllers to accompany ros_control.


## Dependencies: DART and Aikido

This package depends on DART 6.0 and a recent version of Aikido.


## Dependencies: ros_control 0.10.0 or later

This package requires
[the `MultiInterfaceController` class][MultiInterfaceController],
which was added in
[version 0.10.0 of `controller_interface`][controller_interface_version]. If
you are using ROS `indigo`, you will need to build a newer version of
`ros_control` from source.
rom source.

Add the following package to your workspace and re-build it:
```console
- git:
    local-name: ros_control
    uri: https://github.com/ros-controls/ros_control.git
    version: jade-devel
```

Note that version 0.10.0 of `controller_manager_msgs` is not backwards
compatible with previous versions of that package. Therefore, you must update
your checkout of `ros_control` in *any workspaces that are used to communicate
with the controller manager over ROS*.

Additionally, you should update `ros_controllers` if you plan to use any of the
controllers distributed with `ros_control` (e.g. `JointStateController`):
```console
- git:
    local-name: ros_controllers
    uri: https://github.com/ros-controls/ros_controllers.git
    version: jade-devel
```

## Authors

rewd_controllers is developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) in the
[Robotics Institute](http://ri.cmu.edu/) at
[Carnegie Mellon University](http://www.cmu.edu/). The controllers were
written by
Clint Liddick ([**@ClintLiddick**](https://github.com/ClintLiddick))
with some contributions from
Michael Koval ([**@mkoval**](https://github.com/mkoval)).

[MultiInterfaceController]: https://github.com/ros-controls/ros_control/pull/204
[controller_interface_version]: https://github.com/ros-controls/ros_control/blob/0.10.0/controller_interface/CHANGELOG.rst#0100-2015-11-20

