# Package mag_calculator

## Overview

The package contains the nodes :

* backward_model: to calculate a set of current to reach a desired magnetic
field
* forward_model: to model the magnetic field for a set of currents
* compute_max_field: An example node showing how you can compute the maximum
field in a direction
* ComputeMaxField.py: The same as above but done in python (to be removed)

The computation is done in the mns frame. If the incoming FieldStamped message
has a different frame it will use the transform tree to convert the magnetic
field to the mns frame.

## Usage

### Magnetic Controller Node Example

Start backward model node:

```bash
rosrun mag_calculator backward_model
```

Load a magnetic calibration file

```bash
rosservice call /backward_model/load_calibration_file
~/tesla_ws/src/mag_control/mpem/cal/OctoMag_Calibration.yaml
```

### Magnetic Model Node Example

Start the forward model node.

```bash
rosrun mag_calculator forward_model
```

Call service get_mag_field(position,currents):

```bash
rosservice call /forward_model/get_mag_field  '[0, 0, 0]' '[4, 5, 5, 5,
    5, 5, 5, 3]'
```

### Starting Without Launch File

If the note is started without launch file the calibration file need to be set
manually

```bash
rosrun mag_calculator forward_model rosservice call
/forward_model/load_calibration_file
'~/tesla_ws/src/mag_control/mpem/cal/OctoMag_Calibration.yaml'
```

## Nodes

### backward_model

#### Subscribed Topics

* **`~field`** (mag_msgs/FieldStamped) Input currents for which to calculate
currents

#### Published Topics

* **`~currents`** (mag_msgs/CurrentsStamped) Currents that are calculated from a
magnetic quantity

* **`/ECB/desired_currents_block`** (ecb_msgs/CurrentsBlock) Currents converted
to an ECB block. To be used by the ECB controller.  Note that the block is just
the currents repeated over the entire block.

#### Services

* **`~load_calibration_file`** (mag_calculator/loadCalibrationSrv)

#### Parameters

* **`~calibration_path`** (string, default: "") The path to the calibration
file.

* **`~max_field_intensity`** (double, default: 0.04) The maximum allowed field
intensity. Fields above this are rescaled to the maximum field intensity.

* **`~max_gradient_intensity`** (double, default: 0.5) The maximum allowed
gradient intensity. Gradients above this are rescaled to the maximum.

### forward_model

#### Subscribed Topics

* **`~pose`** (geometry_msgs::PoseArray) Positions at which to calculate the
magnetic field
* **`~currents`** (mag_msgs::CurrentStamped) Current vector on the
electromagnets

#### Published Topics

* **`~field_array`** (mag_msgs::FieldArrayStamped) Fields calculated at the
given positions
* **`~field_aligned_gradient_array`** (mag_msgs::FieldArrayStamped) Gradient
vectors aligned to the field direction at the given positions

#### Services

* **`load_calibration_file`** (magnetic_controller/loadCalibrationSrv) Reloads
the magnetic calibration from another YAML file.

* **`get_field`** (magnetic_controller/getMagneticFieldSrv) Gets the magnetic
field at a given position for a given current vector.

#### Parameters

* **`calibration_path`** (string, default: "") The path to the calibration file.
* **`publish_field_aligned_grad_array`** (bool, default: true) If this is true,
    the aligned gradients are also calculated and published as a FieldArray

### Subscribed Topics

* **'`backward_model/field`** (mag_msgs/FieldStamped) The input field sets the
direction in which to calculate the maximum available field

### Published Topics

* **`max_field`** (std_msgs/Float32) The maximum field available in the
direction of /backward_model/field
