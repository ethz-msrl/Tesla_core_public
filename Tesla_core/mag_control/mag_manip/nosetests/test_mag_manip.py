# coding: utf-8
# Tesla - A ROS-based framework for performing magnetic manipulation

# Software License Agreement (BSD License)

# 2022 ETH Zurich, ​MAVT, Multi Scale Robotics Lab (MSRL) , Prof Bradley J. Nelson
# All rights reserved.

# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * All advertising materials mentioning features or use of this software
#   must display the following acknowledgement:
#   “This product includes software developed by the Multi-Scale Robotics Lab,
#   ETH Zurich, Switzerland and its contributors.”

# * Neither the name of MSRL nor the names of its
#   contributors may be used to endorse or promote products
#   derived from this software without specific prior
#   written permission of MSRL.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from os import path

import numpy as np
import rospkg
from nose.tools import raises
from numpy.testing import assert_almost_equal


def test_import():
    from mag_manip import mag_manip

    model = mag_manip.ForwardModelMPEM()
    assert not model.isValid()


def test_forward_model_factory():
    from mag_manip.mag_manip import ForwardModelFactory

    r = rospkg.RosPack()
    f = ForwardModelFactory()
    pkg_path = r.get_path("mag_manip")
    cal_path = path.join(pkg_path, "test", "OctoMag_Calibration.yaml")
    model = f.create("mpem", cal_path)
    assert model.isValid()


def test_forward_model_mpem():
    from mag_manip.mag_manip import ForwardModelMPEM

    r = rospkg.RosPack()
    pkg_path = r.get_path("mag_manip")
    cal_path = path.join(pkg_path, "test", "OctoMag_Calibration.yaml")
    model = ForwardModelMPEM()
    model.setCalibrationFile(cal_path)
    assert model.isValid()
    position = np.zeros((3,), np.float)
    currents = np.ones((8,), np.float)
    field = model.computeFieldFromCurrents(position, currents)
    assert field.shape == (3, 1)


def test_backward_model_mpem_L2():
    from mag_manip.mag_manip import BackwardModelMPEML2, ForwardModelMPEM

    r = rospkg.RosPack()
    pkg_path = r.get_path("mag_manip")
    cal_path = path.join(pkg_path, "test", "OctoMag_Calibration.yaml")
    model = BackwardModelMPEML2()
    model.setCalibrationFile(cal_path)
    assert model.isValid()
    position = np.zeros((3,), np.float)
    field = np.array([30e-3, 0, 0], np.float)
    currents = model.computeCurrentsFromField(position, field)
    assert currents.shape == (8, 1)
    f_model = ForwardModelMPEM()
    f_model.setCalibrationFile(cal_path)
    field_ = f_model.computeFieldFromCurrents(position, currents)
    assert_almost_equal(field, np.squeeze(field_))


def test_forward_model_mpem_saturation():
    from mag_manip.mag_manip import (
        ForwardModelMPEM,
        ForwardModelSaturation,
        SaturationTanh,
    )

    r = rospkg.RosPack()
    pkg_path = r.get_path("mag_manip")
    cal_path = path.join(pkg_path, "test", "OctoMag_Calibration.yaml")
    model = ForwardModelSaturation()
    f_model = ForwardModelMPEM()
    model.setModel(f_model)

    model.setModelCalibrationFile(cal_path)
    sat_params = np.array([16.0, 1 / 16.0])
    sat_functions = [SaturationTanh(sat_params)] * model.getNumCoils()
    model.setSaturationFunctions(sat_functions)
    assert model.isValid()
    currents = 8 * np.ones((8, 1), np.float)
    position = np.zeros((3, 1), np.float)
    field_gradient = model.getModel().computeFieldGradient5FromCurrents(
        position, currents
    )
    field_gradient_sat = model.computeFieldGradient5FromCurrents(position, currents)

    assert np.linalg.norm(field_gradient) > np.linalg.norm(field_gradient_sat)


@raises(Exception)
def test_backward_model_mpem_L2_saturation_toohigh():
    from mag_manip.mag_manip import BackwardModelSaturation

    r = rospkg.RosPack()
    pkg_path = r.get_path("mag_manip")
    cal_path = path.join(pkg_path, "models", "cmag_bml2s_v1", "params.yaml")
    bmodel = BackwardModelSaturation()
    bmodel.setCalibrationFile(cal_path)
    field = np.array([-0.129076, -0.0131011, -0.083101])
    position = np.zeros((3, 1), np.float)
    currents_calc = bmodel.computeCurrentsFromField(position, field)
    assert np.linalg.norm(currents_calc) > 0


def test_backward_model_nls_pair():
    import numpy as np
    from mag_manip.mag_manip import BackwardModelNLS, ForwardModelMPEM

    position = np.zeros((3,), np.float)
    field = np.array([30e-3, 0, 0], np.float)

    r = rospkg.RosPack()
    pkg_path = r.get_path("mag_manip")
    fmodel = ForwardModelMPEM()
    cal_path = path.join(pkg_path, "test", "OctoMag_Calibration.yaml")
    fmodel.setCalibrationFile(cal_path)

    model = BackwardModelNLS()
    model.setForwardModel(fmodel)

    ret, currents = model.computeCurrentsFromFieldRet(position, field)

    assert ret

    gradient5 = np.zeros((5,), np.float)
    ret, currents = model.computeCurrentsFromFieldGradient5Ret(
        position, field, gradient5
    )
    assert ret

    dipole = np.array([1, 0, 0], np.float)
    gradient3 = np.array([0.5, 0, 0], np.float)
    ret, currents = model.computeCurrentsFromFieldDipoleGradient3Ret(
        position, field, dipole, gradient3
    )
    assert ret
