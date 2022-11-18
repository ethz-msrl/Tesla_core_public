// Tell swig the name of the module we're creating
%module mag_manip 

// Pull in the headers from Python itself and from our library
%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "mag_manip/rbf_3d_field_interpolator.h"
#include "mag_manip/backward_model.h"
#include "mag_manip/backward_model_factory.h"
#include "mag_manip/backward_model_saturation.h"
#include "mag_manip/backward_model_nls.h"
#include "mag_manip/backward_model_linear_rbf_L2.h"
#include "mag_manip/backward_model_linear_thinplatespline_L2.h"
#include "mag_manip/backward_model_linear_vfield_L2.h"
#include "mag_manip/backward_model_mpem_L2.h"
#include "mag_manip/forward_model_linear_vfield.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/forward_model.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/forward_model_linear_rbf.h"
#include "mag_manip/forward_model_linear_thinplatespline.h"
#include "mag_manip/forward_model_linear_vfield.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/forward_model_saturation.h"
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/dipole_interactions.h"
#include "mag_manip/types.h"
#include "mag_manip/helpers.h"
#include "mag_manip/saturation.h"
#include "mag_manip/saturation_function_factory.h"
#include "mag_manip/emns.h"
#include "mag_manip/currents_jacobian_functor.h"
#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#ifndef NO_TENSORFLOW
#include "mag_manip/forward_model_tensorflow.h"
#endif
%}

// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <exception.i>
%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>
%include <std_pair.i>

// eigen.i is found in ../swig/ and contains specific definitions to convert
// Eigen matrices into Numpy arrays.
%include <eigen.i>
%include <std_unique_ptr.i>
%include "mag_manip/types.h"

%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
//%template(Vector8d) Eigen::MatrixXd;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::VectorXf)
%eigen_typemaps(Eigen::MatrixXf)
%eigen_typemaps(Eigen::Vector3f)
%eigen_typemaps(Eigen::Matrix3f)
%eigen_typemaps(mag_manip::PositionVec)
%eigen_typemaps(mag_manip::FieldVec)
%eigen_typemaps(mag_manip::CurrentsVec)
%eigen_typemaps(mag_manip::ActuationMat)
// Even though Eigen::MatrixXd is just a typedef for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>, our templatedInverse function doesn't
// compile correctly unless we also declare typemaps for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>. Not totally sure why that is.
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 3, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 5, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 8, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 5, 1>)
%eigen_typemaps(Eigen::Matrix<double, 3, 1>)
%eigen_typemaps(Eigen::Matrix<double, 8, 1>)
%eigen_typemaps(Eigen::Matrix<double, 5, 3>)
%eigen_typemaps(Eigen::Matrix<double, 3, 5>)

%shared_ptr(mag_manip::SaturationFunction);
%shared_ptr(mag_manip::SaturationErf);
%shared_ptr(mag_manip::SaturationAtan);
%shared_ptr(mag_manip::SaturationTanh);
%shared_ptr(mag_manip::SaturationRational);
%template(vectorSaturationSharedPtr) std::vector<std::shared_ptr<mag_manip::SaturationFunction> >;
%template(vectorSaturationTanhSharedPtr) std::vector<std::shared_ptr<mag_manip::SaturationTanh> >;
%template(vectorSaturationErfSharedPtr) std::vector<std::shared_ptr<mag_manip::SaturationErf> >;
%template(vectorSaturationRationalSharedPtr) std::vector<std::shared_ptr<mag_manip::SaturationRational> >;
%template(vectorSaturationAtanSharedPtr) std::vector<std::shared_ptr<mag_manip::SaturationAtan> >;

%shared_ptr(mag_manip::RBF3DFieldInterpolator);
%shared_ptr(mag_manip::ThinPlateSplineInterpolator);
%shared_ptr(mag_manip::BackwardModel);
%shared_ptr(mag_manip::BackwardModelLinearL2);
%shared_ptr(mag_manip::BackwardModelLinearVFieldL2);
%shared_ptr(mag_manip::BackwardModelNLS);
%shared_ptr(mag_manip::BackwardModelSaturation);
%shared_ptr(mag_manip::BackwardModelLinearRBFL2);
%shared_ptr(mag_manip::BackwardModelLinearThinPlateSplineL2);
%shared_ptr(mag_manip::BackwardModelMPEML2);
%shared_ptr(mag_manip::ForwardModelLinear);
%shared_ptr(mag_manip::ForwardModelSaturation);
%shared_ptr(mag_manip::ForwardModelLinearSaturation);
%shared_ptr(mag_manip::ForwardModelMPEM);
%shared_ptr(mag_manip::ForwardModelLinearVField);
%shared_ptr(mag_manip::ForwardModelLinearRBF);
%shared_ptr(mag_manip::ForwardModelLinearThinPlateSpline);
#ifndef NO_TENSORFLOW
%shared_ptr(mag_manip::ForwardModelTensorFlow);
#endif
%shared_ptr(mag_manip::ForwardModel);
%shared_ptr(mag_manip::BackwardModel);
%shared_ptr(mag_manip::CurrentsJacobianFunctor);

//%template(RBF3DFieldInterpolatorVec) std::vector<std::shared_ptr<mag_manip::RBF3DFieldInterpolator> >;

// swig has a real problem with these 
%ignore mag_manip::ForwardModelLinearRBF::getInterpolants;
%ignore mag_manip::BackwardModelLinearRBFL2::getInterpolants;
%ignore mag_manip::ForwardModelLinearThinPlateSpline::getInterpolants;
%ignore mag_manip::BackwardModelLinearThinPlateSplineL2::getInterpolants;

// mag_manip raises exceptions that will crash a Python program
// We instead treat the exceptions as Python error messages
// There is some overhead with this though as it will wrap every function
// in a try/catch block
%exception {
    try {
        $action
    }
    catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
        SWIG_fail;
    }
}

// Tell swig to build bindings for everything in our library
%include "mag_manip/rbf_3d_field_interpolator.h"
%include "mag_manip/saturation_function.h"
%include "mag_manip/saturation_function_factory.h"
%include "mag_manip/saturation_tanh.h"
%include "mag_manip/saturation_atan.h"
%include "mag_manip/saturation_rational.h"
%include "mag_manip/saturation_erf.h"

%include "mag_manip/backward_model.h"
%include "mag_manip/backward_model_linear_L2.h"
%include "mag_manip/forward_model.h"
%include "mag_manip/forward_model_linear.h"
%include "mag_manip/backward_model_factory.h"
%include "mag_manip/forward_model_factory.h"
%include "mag_manip/forward_model_saturation.h"
%include "mag_manip/vfield_grid_properties.h"
%include "mag_manip/interpolate_regular.h"
%include "mag_manip/forward_model_linear_saturation.h"
%include "mag_manip/forward_model_linear_vfield.h"
%include "mag_manip/forward_model_mpem.h"
%include "mag_manip/forward_model_linear_rbf.h"
%include "mag_manip/forward_model_linear_thinplatespline.h"
%include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#ifndef NO_TENSORFLOW
%include "mag_manip/forward_model_tensorflow.h"
#endif
%include "mag_manip/backward_model_linear_vfield_L2.h" 
%include "mag_manip/backward_model_mpem_L2.h"
%include "mag_manip/backward_model_saturation.h"
%include "mag_manip/backward_model_linear_rbf_L2.h"
%include "mag_manip/backward_model_linear_thinplatespline_L2.h"
%include "mag_manip/currents_jacobian_functor.h"
%include "mag_manip/backward_model_nls.h"
%include "mag_manip/dipole_interactions.h"
%include "mag_manip/helpers.h"
%include "mag_manip/emns.h"

