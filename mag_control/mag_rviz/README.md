# Package mag_rviz

## Overview

The package contains Rviz plugins to control and display magnetic fields, magnetic field gradients, and currents of electromagnetic navigation systems (eMNS).

## Rviz plugins

### Displays

* `magnetics/Magnetic Field`: Displays a vector for the applied magnetic field.
* `magnetics/Magnetic Field Array`: Displays vectors for the applied magnetic fields.
* `magnetics/Gradient 3`:  Displays a vector for the 3D force in gradient units.
* `magnetics/Projected Gradient`:  Displays a projection of the 5D gradient in a direction given by the alignement vector parameter.
* `magnetics/Currents`: Displays a text overlay of current values in Amps.

### Panels

* `magnetics/Send Currents Panel`:  Manually send current values.
* `magnetics/Currents Panel`: Displays current values in Amps.
* `magnetics/Magnetic Field Panel`: Displays and control for magnetic field.
* `magnetics/Magnetic Field Rotation Panel`: Displays rotation control for magnetic field.
* `magnetics/Dipole Gradient Panel`: Displays control for dipole gradient.
* `magnetics/Dipole Gradient`:  Displays vectors for the applied magnetic field and dipole gradient.
* `magnetics/Tracking Update Panel`:  Trigger a position update.
* `magnetics/PS4 Visual`: Visual information to ps4 controller.

### View Controller

* `magnetics/View Publisher`: View Publisher.

## Launch files

`demo_panels_and_displays.launch` demonstrates the use of the plugin in a preset rviz scene.
