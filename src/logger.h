/*
    KiloSim

    Saves the parameters and time series logs of the Kilobot simulator in HDF5
    files

    Created 2018-10 by Julia Ebert
*/

#ifndef __KILOSIM_LOGGER_H
#define __KILOSIM_LOGGER_H

#include "H5PacketTable.h"
#include "H5Cpp.h"
#include "robot.h"
#include <unordered_map>
#include <string>
#include <memory>
#include <set>
#include <vector>

namespace KiloSim
{
class Logger
{
public:
  // Function that collects outputs from a robot into a single vector (row)
  // A pointer to a function from Robot pointers to a vector of doubles
  typedef std::vector<double> (*aggregatorFunc)(std::vector<Robot *> &robots);
  // a typedef for our managed H5File pointer
  typedef std::shared_ptr<H5::H5File> H5FilePtr;
  typedef std::shared_ptr<H5::Group> H5GroupPtr;
  typedef std::shared_ptr<FL_PacketTable> H5PacketTablePtr;
  typedef std::unordered_map<std::string, double> Params;

protected:
  // HDF5 file where the data lives
  std::string fileID;
  // Trial number specifying group where the data lives
  int trialNum;
  // Names and functions of aggregators
  std::unordered_map<std::string, aggregatorFunc> aggregators;
  // TODO: Save pointer to open file, trial group, parameter group, and aggregator packet tables
  H5FilePtr h5fileP;
  // HDF5 group name for trial. e.g., /trial_0
  std::string trialGroupName;
  // HDF5 group name for trial parameters. e.g., /trial_0/params
  std::string paramsGroupName;
  H5GroupPtr paramsGroup;
  // HDF5 dataset name for time series (packet table)
  std::string timeDsetName;
  H5PacketTablePtr timeTable;

public:
  // Construct a Logger that logs within the given HDF5 file and group trialNum
  // File will be created if it doesn't exist
  Logger(std::string fileID, int trialNum);
  // Destructor: closes the file when it goes out of scope
  ~Logger();
  // Add an aggregator function that will be run on logState
  void addAggregator(std::string aggName, aggregatorFunc aggFunc);
  // Log the aggregators at the given time mapped over all the given robots
  void logState(double timeSec, std::vector<Robot *> &robots);
  // Log all of the given parameters
  void logParams(Params paramPairs);

protected:
  // Log a single parameter name and value
  void logParam(std::string name, double val);
  // Log data for this specific aggregator
  void logAggregator(std::string aggName, aggregatorFunc aggFunc, std::vector<Robot *> &robots);
  // Create or open an HDF5 file
  H5FilePtr createOrOpenFile(const std::string &fname);
  // Create or open a group in an HDF5 file
  H5GroupPtr createOrOpenGroup(H5FilePtr file, std::string &groupName);
};
} // namespace KiloSim

#endif