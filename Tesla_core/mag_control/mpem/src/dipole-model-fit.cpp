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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <mpem/electromagnet_calibration.h>

#include <cmath>

using namespace std;

void parseData(const std::string& fileName, std::vector<MagneticMeasurement>& calibrationData_out,
               bool flipData = false);
void combineData(std::vector<MagneticMeasurement>& calibrationData_out);

int main(int argc, char* argv[]) {
  /*
   *  IF YOU GET THIS ERROR:
   * terminate called after throwing an instance of 'YAML::BadFile'
   *  what():  yaml-cpp: error at line 0, column 0: bad file
   *
   * It either means your not pointing to the correct file location OR the file has inf in the
   * workspace fields
   *
   *
   */

  // std::string filePath = "../calFiles/";
  // std::string deviceName = "MINIMAG_E090094";
  // std::string calibrationDataFileName = filePath + "/MFG_CalData.csv";
  // std::string verificationFileName = filePath + "/MFG_Verification.csv";
  // std::string deviceInitialConditionFileName = filePath + "/MINIMAG_IC.yaml";

  std::string deviceName = argv[1];
  std::string calibrationDataFileName = argv[2];
  std::string verificationFileName = argv[3];
  std::string deviceInitialConditionFileName = argv[4];
  std::string calibrationOutFileName = argv[5];

  // load an initial guess for the source positions and headings
  ElectromagnetCalibration calibration(deviceInitialConditionFileName);

  std::vector<MagneticMeasurement> dataListCal, dataList, dataList_Ver;
  parseData(calibrationDataFileName, dataList, false);
  dataListCal = dataList;
  combineData(
      dataListCal);  // average field for measurments with the same position and current applied

  // turning on offset for CMag
  // calibration.useOffset(true);
  calibration.calibrate(deviceName, dataListCal, true, true,
                        ElectromagnetCalibration::HEADING_AND_POSITION, -1, -1, 1e-12, 5000);

  calibration.writeCalibration(calibrationOutFileName);

  cout << endl
       << endl
       << " -------------------- VERIFICATION COMPARISON -------------------- " << endl
       << endl;
  parseData(verificationFileName, dataList_Ver, false);

  calibration.printStats(dataList_Ver);

  return 0;
}

void parseData(const std::string& fileName, std::vector<MagneticMeasurement>& calibrationData_out,
               bool flipData) {
  calibrationData_out.clear();

  ifstream inputStream(fileName);

  string line;

  // skip the first line
  std::getline(inputStream, line);

  // getting number of currents
  if (inputStream.eof()) {
    throw runtime_error("Invalid file. File appears empty");
  }

  // number of currents is number of columns - 6 (3 for field and 3 for position)
  getline(inputStream, line);
  istringstream iss(line);

  int num_cols = 0;
  for (string s; getline(iss, line, ','); num_cols++) {
  }

  const int num_currents = num_cols - 6;

  MagneticMeasurement dataPoint;
  dataPoint.AppliedCurrentVector.setZero(num_currents, 1);

  // restart the file at the beginning
  inputStream = ifstream(fileName);

  // skip the first line
  std::getline(inputStream, line);

  while (!inputStream.eof()) {
    std::getline(inputStream, line);
    if (inputStream.eof()) {
      break;
    }

    stringstream inputLine(line);

    inputLine >> dataPoint.Position.x();
    inputLine.get();  // remove ,
    inputLine >> dataPoint.Position.y();
    inputLine.get();
    inputLine >> dataPoint.Position.z();
    inputLine.get();

    for (int i = 0; i < num_currents; i++) {
      double cur;
      inputLine >> cur;
      inputLine.get();
      dataPoint.AppliedCurrentVector(i) = cur;
    }

    // cout << dataPoint.AppliedCurrentVector.transpose() << endl;

    inputLine >> dataPoint.Field.x();
    inputLine.get();
    inputLine >> dataPoint.Field.y();
    inputLine.get();
    inputLine >> dataPoint.Field.z();

    if (flipData) {
      dataPoint.Field /= 1000.0;
      dataPoint.Position /= 1.0e6;

      // switch for coordinate system
      dataPoint.Field.z() *= -1;
      dataPoint.Field.x() *= -1;
      dataPoint.Position *= -1;
    }

    calibrationData_out.push_back(dataPoint);
  }

  return;
}

void combineData(std::vector<MagneticMeasurement>& calibrationData_out) {
  std::vector<MagneticMeasurement>::iterator dataIT = calibrationData_out.begin();
  std::vector<MagneticMeasurement>::iterator dataIT_Next = dataIT;
  dataIT_Next++;

  while (dataIT != calibrationData_out.end()) {
    dataIT_Next = dataIT;
    dataIT_Next++;
    double count = 1.0;
    while (dataIT_Next != calibrationData_out.end() && dataIT->Position == dataIT_Next->Position &&
           dataIT->AppliedCurrentVector == dataIT_Next->AppliedCurrentVector) {
      dataIT->Field += dataIT_Next->Field;
      count++;
      dataIT_Next++;
    }

    dataIT->Field /= count;
    dataIT++;
    dataIT = calibrationData_out.erase(dataIT, dataIT_Next);
  }
}
