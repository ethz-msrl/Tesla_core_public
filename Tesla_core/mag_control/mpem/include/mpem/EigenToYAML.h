//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Software License Agreement (BSD License)
//
// ©2022 ETH Zurich, Andrew Petruska, Janis Edelmann, D-​MAVT; Multi-Scale Robotics Lab (MSRL) ;
// Prof Bradley J. Nelson All rights reserved.
//
// Redistribution and use of this software in source and binary forms,
// with or without modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above
//   copyright notice, this list of conditions and the
//   following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * All advertising materials mentioning features or use of this software
//   must display the following acknowledgement:
//   “This product includes software developed by the Multi-Scale Robotics Lab, ETH Zurich,
//   Switzerland and its contributors.”
//
// * Neither the name of MSRL nor the names of its
//   contributors may be used to endorse or promote products
//   derived from this software without specific prior
//   written permission of MSRL.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef EIGENTOYAML_H
#define EIGENTOYAML_H

/**
    This header file creates template specializations to work with the libyaml-cpp library for Eigen
matricies and std::vector<double> vectors.

**/

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace YAML {
template <>
struct convert<Eigen::VectorXd> {
  static Node encode(const Eigen::VectorXd& rhs) {
    Node node;
    for (unsigned int i = 0; i < rhs.size(); i++) {
      node.push_back(rhs(i));
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXd& rhs) {
    if (node.size() == 0 && node.IsSequence()) {
      // Empty Sequence
      rhs.resize(0, 1);
      return true;

    } else if (node.IsSequence() && node[0].size() == 1) {
      rhs.resize(node.size(), 1);

      for (unsigned int i = 0; i < node.size(); i++) rhs[i] = node[i][0].as<double>();

      return true;

    } else if (node.IsSequence() && node[0].size() == 0) {
      rhs.resize(node.size(), 1);

      for (unsigned int i = 0; i < node.size(); i++) rhs[i] = node[i].as<double>();

      return true;
    } else if (node.IsScalar()) {
      std::stringstream tmp;
      tmp << node;
      std::string nodeStr = tmp.str();

      if ((nodeStr.size() == 1 && nodeStr.c_str()[0] == '~')) {
        rhs.resize(0, 1);
      } else {
        rhs.resize(1, 1);
        rhs[0] = node.as<double>();
      }

      return true;
    }

    /*if(!node.IsSequence() || ( node.size() > 0 && node[0].size() != 1) )
    {
        return false;
    }

    if( node.size() == 0 )
    {
        rhs.resize(0,0);
        return true;
    }

    rhs.resize(node.size(),1);

    for( unsigned int i=0; i<node.size(); i++ )
        rhs[i] = node[i][0].as<double>();

    return true;*/
  }
};

template <>
struct convert<Eigen::Vector3d> {
  static Node encode(const Eigen::Vector3d& rhs) {
    Node node;
    for (unsigned int i = 0; i < 3; i++) {
      node.push_back(rhs(i));
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3d& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    if (node[0].size() == 1) {
      for (unsigned int i = 0; i < node.size(); i++) rhs[i] = node[i][0].as<double>();
    } else if (node[0].size() == 0) {
      for (unsigned int i = 0; i < node.size(); i++) rhs[i] = node[i].as<double>();
    } else {
      return false;
    }

    return true;
  }
};

template <>
struct convert<Eigen::MatrixXd> {
  static Node encode(const Eigen::MatrixXd& rhs) {
    Node node;
    for (unsigned int i = 0; i < rhs.rows(); i++) {
      /*Node tmpNode;
      for( unsigned int j=0; j<rhs.cols(); j++ )
          tmpNode.push_back( rhs(i,j) );

      node.push_back( tmpNode );
      */
      for (unsigned int j = 0; j < rhs.cols(); j++) node[i][j] = rhs(i, j);
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::MatrixXd& rhs) {
    if (!node.IsSequence()) {
      return false;
    }

    if (node.size() == 0) {
      // Empty Sequence
      rhs.resize(0, 0);
      return true;
    }

    rhs.resize(node.size(), node[0].size());

    for (unsigned int i = 0; i < node.size(); i++)
      for (unsigned int j = 0; j < node[0].size(); j++) rhs(i, j) = node[i][j].as<double>();

    return true;
  }
};

template <>
struct convert<Eigen::Matrix3d> {
  static Node encode(const Eigen::Matrix3d& rhs) {
    Node node;
    for (unsigned int i = 0; i < rhs.rows(); i++) {
      // Node tmpNode;
      for (unsigned int j = 0; j < rhs.cols(); j++)
        // tmpNode.push_back( rhs(i,j) );
        node[i][j] = rhs(i, j);

      // node.push_back( tmpNode );
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix3d& rhs) {
    if (!node.IsSequence()) {
      return false;
    }

    if (node.size() != 3 || node[0].size() != 3) {
      // Not a 3x3 matrix!
      return false;
    }

    rhs.resize(node.size(), node[0].size());

    for (unsigned int i = 0; i < node.size(); i++)
      for (unsigned int j = 0; j < node[0].size(); j++) rhs(i, j) = node[i][j].as<double>();

    return true;
  }
};

template <>
struct convert<std::vector<double> > {
  static Node encode(const std::vector<double>& rhs) {
    Node node;
    for (unsigned int i = 0; i < rhs.size(); i++) {
      node.push_back((rhs[i]));
    }
    return node;
  }

  static bool decode(const Node& node, std::vector<double>& rhs) {
    if (node.size() == 0 && node.IsSequence()) {
      // Empty Sequence
      rhs.resize(0);
      return true;

    } else if (node.IsSequence() && node[0].size() == 1) {
      for (unsigned int i = 0; i < node.size(); i++) rhs.push_back(node[i][0].as<double>());

      return true;

    } else if (node.IsSequence() && node[0].size() == 0) {
      for (unsigned int i = 0; i < node.size(); i++) rhs.push_back(node[i].as<double>());

      return true;
    } else if (node.IsScalar()) {
      std::stringstream tmp;
      tmp << node;
      std::string nodeStr = tmp.str();

      if ((nodeStr.size() == 1 && nodeStr.c_str()[0] == '~'))
        rhs.resize(0);
      else
        rhs.push_back(node.as<double>());

      return true;
    }

    // Otherwise
    return false;
  }
};
}  // namespace YAML

#endif  // EIGENTOYAML_H
