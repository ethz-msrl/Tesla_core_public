# Tesla_core Utilities

This package contains some generic utilities that are used in other Tesla_core packages.
For now, to keep the number of packages small, we keep all the utilities here,
regardless of whether they are CMake code, python code etc...

## Contents

### Python

#### tsc_utils package

The tsc_utils package contains the following modules

* Conversions module with helper functions converting geometry_msgs types and Numpy types
* Rotations module containing some rotations conversions
* mag_utils: module with helper functions for generating lists of magnetic fields
* rosbag_extract: module with helper file for extracting rosbag data into pandas DataFrams

### Python Utility Scripts

#### rosbag_processor.py

Usage:

```bash
rosrun tsc_utils rosbag_processor.py <input_dir> <output_dir>
```

This script can be used to extract numerical data from a folder of rosbags to CSV files. A single CSV file is created
for each detected topic in the bag file.

If you give the script a folder of bags as the input, a subfolder with the bag name will be created and inside will be a
CSV file for each detected topic.

See

```bash
rosrun tsc_utils rosbag_processor.py -h
```

For a list of options.

#### broadcast_transform_node.py

ROS node similar to tf/static_transform_publisher that broadcasts a static transformation based on a rosparam
definition.

#### currents_list_node.py

ROS node that takes a CSV file with the following format
*current0 ... currentN* duration and publishes a mag_msgs/CurrentsStamped message. This node is useful for sending a
predefined set of current values to an eMNS.

#### fields_list_node.py

ROS node that takes a CSV file with the following format
`fieldX fieldY fieldZ posX posY posZ duration`
and publishes a mag_msgs/FieldStamped message.

#### poses_custom_grid_node.py

ROS node that publishes poses on a regular grid based on params in the param server.

#### generate_rotating_fields.py

Script that outputs a CSV file with magnetic field values based on roparams. Can be used in combination with
fields_list_node.py to statically generate a rotating field.

#### set_param_from_transform_node.py

ROS node that looks up a tf.transform and saves it as params in rosparam. Can be used with
broadcast_transform_node.py

#### publish_rviz_mesh_marker_node.py

ROS node that publishes RViz markers based on a mesh file.

#### test_field_array_pub_node.py

ROS node that publishes a FieldArrayStamped with random vectors for testing.

### Maintenance Utilities

These scripts are used for maintenance of the Tesla repo.

* **generate_docs.py** is used to export the API docs for each package
* **update_license_tags.py** is used to update all the license tags in the package.xml files
* **update_version_number.py** is used to update all the version numbers in the package.xml folders.
* **get_python_package_info.py** is used to output info about Python catkin packages to a CSV file. This was used for
the Python 3 migration.
* **print_package_and_maintainers.py** is used to output info about catkin packages and their maintainers.

### CMake

* Code for generating python bindings via [SWIG](http://www.swig.org/)
