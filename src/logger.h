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
  std::string m_file_id;
  // Trial number specifying group where the data lives
  int trial_num;
  // Names and functions of aggregators
  std::unordered_map<std::string, aggregatorFunc> aggregators;
  // TODO: Save pointer to open file, trial group, parameter group, and aggregator packet tables
  H5FilePtr m_h5_file;
  // HDF5 group name for trial. e.g., /trial_0
  std::string m_trial_group_name;
  // HDF5 group name for trial parameters. e.g., /trial_0/params
  std::string m_params_group_name;
  H5GroupPtr m_params_group;
  // HDF5 dataset name for time series (packet table)
  std::string m_time_dset_name;
  H5PacketTablePtr m_time_table;

public:
  // Construct a Logger that logs within the given HDF5 file and group trial_num
  // File will be created if it doesn't exist
  Logger(std::string file_id, int trial_num);
  // Destructor: closes the file when it goes out of scope
  ~Logger();
  // Add an aggregator function that will be run on log_state
  void add_aggregator(std::string aggName, aggregatorFunc aggFunc);
  // Log the aggregators at the given time mapped over all the given robots
  void log_state(double timeSec, std::vector<Robot *> &robots);
  // Log all of the given parameters
  void log_params(Params paramPairs);

protected:
  // Log a single parameter name and value
  void log_param(std::string name, double val);
  // Log data for this specific aggregator
  void log_aggregator(std::string aggName, aggregatorFunc aggFunc, std::vector<Robot *> &robots);
  // Create or open an HDF5 file
  H5FilePtr create_or_open_file(const std::string &fname);
  // Create or open a group in an HDF5 file
  H5GroupPtr create_or_open_group(H5FilePtr file, std::string &groupName);
};
} // namespace KiloSim

#endif