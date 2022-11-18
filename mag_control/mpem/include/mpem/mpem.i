// Tell swig the name of the module we're creating
%module mpem

// Pull in the headers from Python itself and from our library
%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "mpem/electromagnet_calibration.h"
%}

// typemaps.i is a built-in swig interface that lets us map c++ types to other
// types in our language of choice. We'll use it to map Eigen matrices to
// Numpy arrays.
%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>

// eigen.i is found in ../swig/ and contains specific definitions to convert
// Eigen matrices into Numpy arrays.
%include <eigen.i>

%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
//%template(Vector8d) Eigen::MatrixXd;

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix3d)
// Even though Eigen::MatrixXd is just a typedef for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>, our templatedInverse function doesn't
// compile correctly unless we also declare typemaps for Eigen::Matrix<double,
// Eigen::Dynamic, Eigen::Dynamic>. Not totally sure why that is.
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 5, 1>)
%eigen_typemaps(Eigen::Matrix<double, 3, 1>)
%eigen_typemaps(Eigen::Matrix<double, 8, 1>)
%eigen_typemaps(Eigen::Matrix<double, 5, 3>)

// Tell swig to build bindings for everything in our library
%include "mpem/electromagnet_calibration.h"
