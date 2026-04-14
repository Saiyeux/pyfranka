//
// Created by cyshi on 2021/2/26.
//

#ifndef FRANKA_PYBIND_FRANKA_LOG_H
#define FRANKA_PYBIND_FRANKA_LOG_H

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/exception.h>
#include <franka/robot.h>

void writeLogToFile(const std::vector<franka::Record>& log);
void test_path();

#endif //FRANKA_PYBIND_FRANKA_LOG_H
