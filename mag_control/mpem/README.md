# Multipole Electromagnet Model

This models magnetic steering systems using a number of magnetic multipole
sources representing the electromagnets and ferromagnetic cores. The model
is analytical and thus provides analytical expressions for the magnetic fields and
gradients in the workspace. Additionally these expressions obey Maxwell's
equations for quasistatic magnetic fields.

For more information on the model, see [1].

## C++ API

Both the functionality for calibrating the model from data and using the model
are contained in [the ElectromagnetCalibration class](include/mpem/electromagnet_calibration.h).

## Python API

A python extension module is automatically generated using SWIG.
To use, import the package:

```python
import mpem
cal = mpem.ElectromagnetCalibration('calibration.yaml')
```

## Fitting a MPEM model

The MPEM model is fit using nonlinear least squares and the LM algorithm. To fit the model, you need both magnetic field
data and initial values of the model parameters.

### Model Initial Parameters

Initial parameterization of the different eMNS configurations is found in the `cal_ic_files` folder.

### Magnetic Field Calibration Data

In order to fit a MPEM model, you need to first collect data of magnetic field measurements and the associated
electromagnet currents. This can be performed using an array of Hall-effect sensors placed on a known grid, or by moving
a single sensor using a micropositioner. The data should be supplied in a comma-separated CSV file with the following
format. Each line corresponds to a single magnetic field vector measurement.

px,py,pz,i0,...,iN,bx,by,bz

The first three columns are the position at which the magnetic field is measured in meters. The following N columns are
the values of the electrical currents on the N electromagnets in Amps. The last three columns are the value of the
magnetic field measurement in Tesla.

**The first line of the data files is assumed to be a header and is ignored.**

### Verification Data

You must also supply a CSV data file of magnetic field measurements that are used to test the fitted model. If you do
not care about validating the model with a separate set of measurements, you can simply supply the same file for both
the fitting and the validation.

### Running the Model Fit

To fit the model, run

```bash
rosrun mpem dipole-model-fit <eMNS_name> <path to calibration data> <path to verification data> <path to initial parameters file> <output yaml filename>
```

Fitting the model can take several minutes and you should see the residual error decreasing as the number of iterations
increases. Once the tolerance has been reached, the final statistics are displayed and the fit model parameters are
saved to the output yaml file. You can then use this file for your MPEM forward and backward models.

## References

[1] A. J. Petruska, J. Edelmann, and B. J. Nelson,
“Model-Based Calibration for Magnetic Manipulation,”
IEEE Trans.  Magn., vol. 53, no. 7, pp. 1–1, Jul. 2017.
