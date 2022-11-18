# Changelog

## [v3.1.0](https://github.com/ethz-msrl/Tesla_core/tree/v3.1.0) (2021-06-23)

[Full Changelog](https://github.com/ethz-msrl/Tesla_core/compare/v3.0.0...v3.1.0)

**Breaking changes:**

- currents\_display rviz plugin still in mag\_rviz package [\#152](https://github.com/ethz-msrl/Tesla_core/issues/152)
- Number of coils now saved in mag\_rviz currents panel [\#153](https://github.com/ethz-msrl/Tesla_core/pull/153) ([qboehler](https://github.com/qboehler))

**Implemented enhancements:**

- Removed hard coded number of electromagnets in currents\_list\_node [\#157](https://github.com/ethz-msrl/Tesla_core/pull/157) ([qboehler](https://github.com/qboehler))
- Added conversion functions to go between numpy and colorrgba message [\#154](https://github.com/ethz-msrl/Tesla_core/pull/154) ([samlcharreyron](https://github.com/samlcharreyron))
- Add number of coils on saving and loading of config of send\_currents\_panel [\#150](https://github.com/ethz-msrl/Tesla_core/pull/150) ([qboehler](https://github.com/qboehler))
- Adding python modules and scripts from tesla\_utils [\#142](https://github.com/ethz-msrl/Tesla_core/pull/142) ([samlcharreyron](https://github.com/samlcharreyron))
- Add parameter to invert field in forward model [\#141](https://github.com/ethz-msrl/Tesla_core/pull/141) ([qboehler](https://github.com/qboehler))
- Automatic linting [\#120](https://github.com/ethz-msrl/Tesla_core/pull/120) ([samlcharreyron](https://github.com/samlcharreyron))

**Fixed bugs:**

- Rostest unit tests fail on Github Actions [\#140](https://github.com/ethz-msrl/Tesla_core/issues/140)
- Fix import of rviz\_utils module in publish\_rviz\_mesh\_marker\_node.py [\#164](https://github.com/ethz-msrl/Tesla_core/pull/164) ([qboehler](https://github.com/qboehler))
- Added tsc\_utils as a dependency to const\_curv\_rod [\#160](https://github.com/ethz-msrl/Tesla_core/pull/160) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed the seed of the randomness in the rotations module tests [\#144](https://github.com/ethz-msrl/Tesla_core/pull/144) ([samlcharreyron](https://github.com/samlcharreyron))
- Moved markdownlint config to where it can be correctly picked up by markdownlint [\#138](https://github.com/ethz-msrl/Tesla_core/pull/138) ([samlcharreyron](https://github.com/samlcharreyron))

**Closed issues:**

- GitHub actions' checks show up under wrong workflow [\#137](https://github.com/ethz-msrl/Tesla_core/issues/137)

**Merged pull requests:**

- Added back hadolint job [\#162](https://github.com/ethz-msrl/Tesla_core/pull/162) ([samlcharreyron](https://github.com/samlcharreyron))
- Added const\_curv\_sim package [\#161](https://github.com/ethz-msrl/Tesla_core/pull/161) ([samlcharreyron](https://github.com/samlcharreyron))
- Fix typo in const\_curv\_rod README [\#159](https://github.com/ethz-msrl/Tesla_core/pull/159) ([qboehler](https://github.com/qboehler))
- Add methods to convert numpy arrays from cartesian to/from spherical coordinates [\#158](https://github.com/ethz-msrl/Tesla_core/pull/158) ([dreyfusr](https://github.com/dreyfusr))
- Removed outdated python scripts from mag\_rviz [\#156](https://github.com/ethz-msrl/Tesla_core/pull/156) ([jstiefel](https://github.com/jstiefel))
- Add README to mag\_rviz package [\#155](https://github.com/ethz-msrl/Tesla_core/pull/155) ([qboehler](https://github.com/qboehler))
- Updated Readme.md with Disable TensorFlow tutorial [\#151](https://github.com/ethz-msrl/Tesla_core/pull/151) ([dreyfusr](https://github.com/dreyfusr))
- Add python installs for tsc\_utils [\#149](https://github.com/ethz-msrl/Tesla_core/pull/149) ([jstiefel](https://github.com/jstiefel))
- fixed exec dependencies on python packages [\#148](https://github.com/ethz-msrl/Tesla_core/pull/148) ([samlcharreyron](https://github.com/samlcharreyron))
- Added const\_curv\_rod package [\#146](https://github.com/ethz-msrl/Tesla_core/pull/146) ([samlcharreyron](https://github.com/samlcharreyron))
- Minor improvements and fixes in doc of tsc\_utils [\#145](https://github.com/ethz-msrl/Tesla_core/pull/145) ([qboehler](https://github.com/qboehler))
- Update README.md [\#143](https://github.com/ethz-msrl/Tesla_core/pull/143) ([qboehler](https://github.com/qboehler))
- Adding mag\_rviz [\#139](https://github.com/ethz-msrl/Tesla_core/pull/139) ([samlcharreyron](https://github.com/samlcharreyron))
- Fix README mag\_tensorflow [\#136](https://github.com/ethz-msrl/Tesla_core/pull/136) ([qboehler](https://github.com/qboehler))
- Fix typo in installation instructions [\#135](https://github.com/ethz-msrl/Tesla_core/pull/135) ([qboehler](https://github.com/qboehler))
- Adding back changes to remove external dependencies [\#126](https://github.com/ethz-msrl/Tesla_core/pull/126) ([samlcharreyron](https://github.com/samlcharreyron))

## [v3.0.0](https://github.com/ethz-msrl/Tesla_core/tree/v3.0.0) (2021-03-24)

[Full Changelog](https://github.com/ethz-msrl/Tesla_core/compare/v2.0.0...v3.0.0)

**Breaking changes:**

- Removed add\_swig\_python\_module from tsc\_utils [\#132](https://github.com/ethz-msrl/Tesla_core/pull/132) ([samlcharreyron](https://github.com/samlcharreyron))
- Rename saturation models [\#105](https://github.com/ethz-msrl/Tesla_core/pull/105) ([samlcharreyron](https://github.com/samlcharreyron))

**Implemented enhancements:**

- Specifying analytical jacobians for BackwardModelNLS from a YAML file [\#115](https://github.com/ethz-msrl/Tesla_core/issues/115)
- Add BackwardModelNLS to factory [\#110](https://github.com/ethz-msrl/Tesla_core/issues/110)
- Add option to specify analytical jacobians in BackwardModelNLS [\#108](https://github.com/ethz-msrl/Tesla_core/issues/108)
- Add support for GPUs in mag\_tensorflow [\#106](https://github.com/ethz-msrl/Tesla_core/issues/106)
- Add issolutionusable to BackwardModelNLS [\#101](https://github.com/ethz-msrl/Tesla_core/issues/101)
- Add more intermediate information in BackwardModelNLS [\#100](https://github.com/ethz-msrl/Tesla_core/issues/100)
- Mag\_manip: add linear models with saturation to factory functions [\#99](https://github.com/ethz-msrl/Tesla_core/issues/99)
- Added GPU support for mag\_tensorflow [\#124](https://github.com/ethz-msrl/Tesla_core/pull/124) ([samlcharreyron](https://github.com/samlcharreyron))
- Logging of BackwardModelNLS Iterations [\#123](https://github.com/ethz-msrl/Tesla_core/pull/123) ([samlcharreyron](https://github.com/samlcharreyron))
- Backward model detects problems computing currents and reports as a ROS error [\#121](https://github.com/ethz-msrl/Tesla_core/pull/121) ([samlcharreyron](https://github.com/samlcharreyron))
- Making clang-format linting automatic [\#119](https://github.com/ethz-msrl/Tesla_core/pull/119) ([samlcharreyron](https://github.com/samlcharreyron))
- Configuring BackwardModelNLS analytical Jacobians from config file [\#117](https://github.com/ethz-msrl/Tesla_core/pull/117) ([samlcharreyron](https://github.com/samlcharreyron))
- Added check on the maximum gradient distance to BackwardModelNLS [\#116](https://github.com/ethz-msrl/Tesla_core/pull/116) ([samlcharreyron](https://github.com/samlcharreyron))
- BackwardModelNLS speedups with analytical Jacobians and position caching [\#114](https://github.com/ethz-msrl/Tesla_core/pull/114) ([samlcharreyron](https://github.com/samlcharreyron))
- Add cmag mpem saturated model for backward\_cmag [\#112](https://github.com/ethz-msrl/Tesla_core/pull/112) ([qboehler](https://github.com/qboehler))
- Made BackwardModelNLS work with the BackwardModelFactory. [\#111](https://github.com/ethz-msrl/Tesla_core/pull/111) ([samlcharreyron](https://github.com/samlcharreyron))
- BackwardModelNLS has options to output success [\#109](https://github.com/ethz-msrl/Tesla_core/pull/109) ([samlcharreyron](https://github.com/samlcharreyron))
- Adding saturated models to common interface [\#104](https://github.com/ethz-msrl/Tesla_core/pull/104) ([samlcharreyron](https://github.com/samlcharreyron))
- Adding cmake option to disable python bindings for MPEM [\#95](https://github.com/ethz-msrl/Tesla_core/pull/95) ([samlcharreyron](https://github.com/samlcharreyron))
- Added unit test for mag\_manip.ForwardModelLinearSaturation [\#93](https://github.com/ethz-msrl/Tesla_core/pull/93) ([samlcharreyron](https://github.com/samlcharreyron))
- Update Navion parameters [\#91](https://github.com/ethz-msrl/Tesla_core/pull/91) ([qboehler](https://github.com/qboehler))
- Check power to limit max field [\#90](https://github.com/ethz-msrl/Tesla_core/pull/90) ([qboehler](https://github.com/qboehler))
- Print VFieldGridProperties to stream [\#87](https://github.com/ethz-msrl/Tesla_core/pull/87) ([samlcharreyron](https://github.com/samlcharreyron))
- Field computations at several positions in mag\_manip [\#83](https://github.com/ethz-msrl/Tesla_core/pull/83) ([samlcharreyron](https://github.com/samlcharreyron))

**Fixed bugs:**

- Passing a vector of shared\_ptr does not work with SWIG [\#103](https://github.com/ethz-msrl/Tesla_core/issues/103)
- Shadowed alignedGradient function in mag\_manip helpers [\#102](https://github.com/ethz-msrl/Tesla_core/issues/102)
- Swigged modules don't compile on Mac [\#98](https://github.com/ethz-msrl/Tesla_core/issues/98)
- Undefined reference to libbenchmark.dylib [\#97](https://github.com/ethz-msrl/Tesla_core/issues/97)
- Fixed bug with in ForwardModelSaturation::getCachedPosition [\#130](https://github.com/ethz-msrl/Tesla_core/pull/130) ([samlcharreyron](https://github.com/samlcharreyron))
- Fix shadowed alignedGradient warning [\#107](https://github.com/ethz-msrl/Tesla_core/pull/107) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed bug in benchmark where the wrong Tensor dimensions were set [\#96](https://github.com/ethz-msrl/Tesla_core/pull/96) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed invalid calibration for default MFG launch file [\#92](https://github.com/ethz-msrl/Tesla_core/pull/92) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixes to the CMag TensorFlow model [\#88](https://github.com/ethz-msrl/Tesla_core/pull/88) ([samlcharreyron](https://github.com/samlcharreyron))

**Closed issues:**

- Set bounds on currents in BackwardModelNLS [\#125](https://github.com/ethz-msrl/Tesla_core/issues/125)
- Tesla\_core's backward\_mfg launch file does not work [\#89](https://github.com/ethz-msrl/Tesla_core/issues/89)

**Merged pull requests:**

- Release v3.0.0 [\#133](https://github.com/ethz-msrl/Tesla_core/pull/133) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed jupyter docker on melodic [\#131](https://github.com/ethz-msrl/Tesla_core/pull/131) ([samlcharreyron](https://github.com/samlcharreyron))
- Testing action to automate docker build [\#129](https://github.com/ethz-msrl/Tesla_core/pull/129) ([samlcharreyron](https://github.com/samlcharreyron))
- Adding box constraints on currents in BackwardsModelNLS [\#127](https://github.com/ethz-msrl/Tesla_core/pull/127) ([samlcharreyron](https://github.com/samlcharreyron))
- Switched some external packages to system dependencies [\#118](https://github.com/ethz-msrl/Tesla_core/pull/118) ([samlcharreyron](https://github.com/samlcharreyron))

## [v2.0.0](https://github.com/ethz-msrl/Tesla_core/tree/v2.0.0) (2020-09-09)

[Full Changelog](https://github.com/ethz-msrl/Tesla_core/compare/1.0.0...v2.0.0)

**Implemented enhancements:**

- Improve gradients computation at edges of grid in Tricubic interpolation [\#61](https://github.com/ethz-msrl/Tesla_core/issues/61)
- Forward model to display dipole aligned gradient [\#55](https://github.com/ethz-msrl/Tesla_core/issues/55)
- Small additions to unit testing in MPEM [\#79](https://github.com/ethz-msrl/Tesla_core/pull/79) ([samlcharreyron](https://github.com/samlcharreyron))
- Docker Jupyter [\#78](https://github.com/ethz-msrl/Tesla_core/pull/78) ([samlcharreyron](https://github.com/samlcharreyron))
- Support for ROS Noetic [\#77](https://github.com/ethz-msrl/Tesla_core/pull/77) ([samlcharreyron](https://github.com/samlcharreyron))
- Switch for disabling TensorFlow [\#76](https://github.com/ethz-msrl/Tesla_core/pull/76) ([samlcharreyron](https://github.com/samlcharreyron))
- Tricubic Scalar Potential Interpolator [\#69](https://github.com/ethz-msrl/Tesla_core/pull/69) ([samlcharreyron](https://github.com/samlcharreyron))
- Better derivative calculation in tricubic interpolation [\#68](https://github.com/ethz-msrl/Tesla_core/pull/68) ([samlcharreyron](https://github.com/samlcharreyron))
- Dockerfile for noetic [\#67](https://github.com/ethz-msrl/Tesla_core/pull/67) ([samlcharreyron](https://github.com/samlcharreyron))
- Aligned Gradient Publisher [\#58](https://github.com/ethz-msrl/Tesla_core/pull/58) ([samlcharreyron](https://github.com/samlcharreyron))
- TensorFlow Based Forward Model [\#57](https://github.com/ethz-msrl/Tesla_core/pull/57) ([samlcharreyron](https://github.com/samlcharreyron))
- Turned on git LFS sync on the build\_and\_test job [\#56](https://github.com/ethz-msrl/Tesla_core/pull/56) ([samlcharreyron](https://github.com/samlcharreyron))
- Backward NLS Returns Success [\#54](https://github.com/ethz-msrl/Tesla_core/pull/54) ([samlcharreyron](https://github.com/samlcharreyron))
- Docker  [\#53](https://github.com/ethz-msrl/Tesla_core/pull/53) ([samlcharreyron](https://github.com/samlcharreyron))
- RBF Interpolation Based Models [\#52](https://github.com/ethz-msrl/Tesla_core/pull/52) ([samlcharreyron](https://github.com/samlcharreyron))
- Added changelog [\#50](https://github.com/ethz-msrl/Tesla_core/pull/50) ([samlcharreyron](https://github.com/samlcharreyron))
- Eigen Tensor Compare [\#49](https://github.com/ethz-msrl/Tesla_core/pull/49) ([samlcharreyron](https://github.com/samlcharreyron))
- Moving over Eigen\_matrix\_compare tests from Tesla [\#47](https://github.com/ethz-msrl/Tesla_core/pull/47) ([samlcharreyron](https://github.com/samlcharreyron))

**Fixed bugs:**

- Add swig as a dependency key [\#74](https://github.com/ethz-msrl/Tesla_core/issues/74)
- Coefficients in Tricubic forward model are incorrectly precomputed [\#59](https://github.com/ethz-msrl/Tesla_core/issues/59)
- Fixed paths in dockerfiles [\#86](https://github.com/ethz-msrl/Tesla_core/pull/86) ([samlcharreyron](https://github.com/samlcharreyron))
- Removing undefined member function declaration [\#85](https://github.com/ethz-msrl/Tesla_core/pull/85) ([samlcharreyron](https://github.com/samlcharreyron))
- Adding swig as a dependency in the package.xml [\#75](https://github.com/ethz-msrl/Tesla_core/pull/75) ([samlcharreyron](https://github.com/samlcharreyron))
- Update Navion\_2.yaml [\#70](https://github.com/ethz-msrl/Tesla_core/pull/70) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed bug with coefficients not being recomputed correctly in tricubic interpolation [\#60](https://github.com/ethz-msrl/Tesla_core/pull/60) ([samlcharreyron](https://github.com/samlcharreyron))
- Eigen\_matrix\_compare now checks if there are NaNs [\#51](https://github.com/ethz-msrl/Tesla_core/pull/51) ([samlcharreyron](https://github.com/samlcharreyron))
- Bumping up eigen\_catkin to eigen 3.3.7 [\#48](https://github.com/ethz-msrl/Tesla_core/pull/48) ([samlcharreyron](https://github.com/samlcharreyron))

**Closed issues:**

- Github lfs instructions [\#72](https://github.com/ethz-msrl/Tesla_core/issues/72)
- clang-format switch version to 6.0 [\#42](https://github.com/ethz-msrl/Tesla_core/issues/42)
- Freezing and last fixes for release [\#39](https://github.com/ethz-msrl/Tesla_core/issues/39)
- mag\_rviz package in Tesla\_core [\#14](https://github.com/ethz-msrl/Tesla_core/issues/14)

**Merged pull requests:**

- preparing for v2.0 [\#84](https://github.com/ethz-msrl/Tesla_core/pull/84) ([samlcharreyron](https://github.com/samlcharreyron))
- CMake scripts for git SHA retrieval [\#82](https://github.com/ethz-msrl/Tesla_core/pull/82) ([dvarx](https://github.com/dvarx))
- Parameter to remove warning "zero field" in forward model [\#81](https://github.com/ethz-msrl/Tesla_core/pull/81) ([qboehler](https://github.com/qboehler))
- rospkg dependency fix [\#80](https://github.com/ethz-msrl/Tesla_core/pull/80) ([jstiefel](https://github.com/jstiefel))
- Add Git LFS instructions to README [\#73](https://github.com/ethz-msrl/Tesla_core/pull/73) ([samlcharreyron](https://github.com/samlcharreyron))
- Add parameters files from mag\_manip to install space [\#71](https://github.com/ethz-msrl/Tesla_core/pull/71) ([qboehler](https://github.com/qboehler))
- Added job to build and test on Noetic [\#66](https://github.com/ethz-msrl/Tesla_core/pull/66) ([samlcharreyron](https://github.com/samlcharreyron))
- Unit tests for Python extensions [\#65](https://github.com/ethz-msrl/Tesla_core/pull/65) ([samlcharreyron](https://github.com/samlcharreyron))
- Create main-self-hosted [\#63](https://github.com/ethz-msrl/Tesla_core/pull/63) ([samlcharreyron](https://github.com/samlcharreyron))

## [1.0.0](https://github.com/ethz-msrl/Tesla_core/tree/1.0.0) (2020-05-07)

[Full Changelog](https://github.com/ethz-msrl/Tesla_core/compare/d4de96bea472d97088a83c70d9f53d527d5a95fa...1.0.0)

**Implemented enhancements:**

- Applying clang-format-6.0 [\#46](https://github.com/ethz-msrl/Tesla_core/pull/46) ([jstiefel](https://github.com/jstiefel))
- eMNS Parameter Interface [\#41](https://github.com/ethz-msrl/Tesla_core/pull/41) ([samlcharreyron](https://github.com/samlcharreyron))
- ran clang-format [\#38](https://github.com/ethz-msrl/Tesla_core/pull/38) ([samlcharreyron](https://github.com/samlcharreyron))
- Continuous integration [\#25](https://github.com/ethz-msrl/Tesla_core/pull/25) ([samlcharreyron](https://github.com/samlcharreyron))
- Feature/get max field in direction [\#22](https://github.com/ethz-msrl/Tesla_core/pull/22) ([samlcharreyron](https://github.com/samlcharreyron))
- Improved exception handling in mag\_manip  [\#19](https://github.com/ethz-msrl/Tesla_core/pull/19) ([samlcharreyron](https://github.com/samlcharreyron))

**Fixed bugs:**

- BackwardModelNLS failing in mag\_manip on melodic [\#30](https://github.com/ethz-msrl/Tesla_core/issues/30)
- Snapcraft build overview [\#9](https://github.com/ethz-msrl/Tesla_core/issues/9)
- Package yaml\_cpp\_catkin necessary? [\#8](https://github.com/ethz-msrl/Tesla_core/issues/8)
- fixed typo in the cmake option to build the mag\_manip python extensions [\#45](https://github.com/ethz-msrl/Tesla_core/pull/45) ([samlcharreyron](https://github.com/samlcharreyron))
- SWIG python install targets [\#37](https://github.com/ethz-msrl/Tesla_core/pull/37) ([samlcharreyron](https://github.com/samlcharreyron))
- Install python executable in tsc\_utils [\#32](https://github.com/ethz-msrl/Tesla_core/pull/32) ([jstiefel](https://github.com/jstiefel))
- Removing auto return type for eigen expressions [\#28](https://github.com/ethz-msrl/Tesla_core/pull/28) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixing mag\_manip unit tests [\#27](https://github.com/ethz-msrl/Tesla_core/pull/27) ([samlcharreyron](https://github.com/samlcharreyron))
- Fix for Fatal error on loading calibration file [\#24](https://github.com/ethz-msrl/Tesla_core/pull/24) ([samlcharreyron](https://github.com/samlcharreyron))
- Remove uptr from mag manip [\#21](https://github.com/ethz-msrl/Tesla_core/pull/21) ([samlcharreyron](https://github.com/samlcharreyron))
- Fixed YAML loading in MPEM [\#18](https://github.com/ethz-msrl/Tesla_core/pull/18) ([samlcharreyron](https://github.com/samlcharreyron))

**Closed issues:**

- Circular dependency between tsc\_utils and mag\_manip caused by ComputeMaxField node [\#34](https://github.com/ethz-msrl/Tesla_core/issues/34)
- Python bindings not properly exported [\#33](https://github.com/ethz-msrl/Tesla_core/issues/33)
- mag\_manip unit tests failing [\#26](https://github.com/ethz-msrl/Tesla_core/issues/26)
- Fatal error on loading calibration file [\#23](https://github.com/ethz-msrl/Tesla_core/issues/23)
- add footnote to wstool documentation [\#15](https://github.com/ethz-msrl/Tesla_core/issues/15)
- Unable to find calibration file with MPEM [\#5](https://github.com/ethz-msrl/Tesla_core/issues/5)
- Migrating to Tesla\_core and Navion [\#2](https://github.com/ethz-msrl/Tesla_core/issues/2)

**Merged pull requests:**

- Link to current branch for testing [\#43](https://github.com/ethz-msrl/Tesla_core/pull/43) ([jstiefel](https://github.com/jstiefel))
- Minor improvements to the CI [\#40](https://github.com/ethz-msrl/Tesla_core/pull/40) ([jstiefel](https://github.com/jstiefel))
- Integration Testing Actions [\#36](https://github.com/ethz-msrl/Tesla_core/pull/36) ([jstiefel](https://github.com/jstiefel))
- Switching linear solver type in BackwardModelNLS [\#31](https://github.com/ethz-msrl/Tesla_core/pull/31) ([samlcharreyron](https://github.com/samlcharreyron))
- switch back to ASL benchmark version [\#29](https://github.com/ethz-msrl/Tesla_core/pull/29) ([jstiefel](https://github.com/jstiefel))
- adding vim swp files to gitignore [\#20](https://github.com/ethz-msrl/Tesla_core/pull/20) ([samlcharreyron](https://github.com/samlcharreyron))
- Broken MPEM build on OSX [\#17](https://github.com/ethz-msrl/Tesla_core/pull/17) ([samlcharreyron](https://github.com/samlcharreyron))
- Add doc ssh key [\#16](https://github.com/ethz-msrl/Tesla_core/pull/16) ([qboehler](https://github.com/qboehler))
- Fix pose transformation for actual field publication in forward model [\#13](https://github.com/ethz-msrl/Tesla_core/pull/13) ([qboehler](https://github.com/qboehler))
- Switching back to SSH as initially proposed [\#12](https://github.com/ethz-msrl/Tesla_core/pull/12) ([jstiefel](https://github.com/jstiefel))
- Feature/get max field script [\#11](https://github.com/ethz-msrl/Tesla_core/pull/11) ([qboehler](https://github.com/qboehler))
- Switching dependency versions [\#10](https://github.com/ethz-msrl/Tesla_core/pull/10) ([jstiefel](https://github.com/jstiefel))
- removed --merge-devel instructions [\#7](https://github.com/ethz-msrl/Tesla_core/pull/7) ([jstiefel](https://github.com/jstiefel))
- Switch to old eigen\_catkin wrapper [\#6](https://github.com/ethz-msrl/Tesla_core/pull/6) ([jstiefel](https://github.com/jstiefel))
- Added mag\_calculator, mag\_manip, tsc\_utils, mpem [\#4](https://github.com/ethz-msrl/Tesla_core/pull/4) ([jstiefel](https://github.com/jstiefel))
- Navion message dependencies [\#1](https://github.com/ethz-msrl/Tesla_core/pull/1) ([jstiefel](https://github.com/jstiefel))



\* *This Changelog was automatically generated by [github_changelog_generator](https://github.com/github-changelog-generator/github-changelog-generator)*
