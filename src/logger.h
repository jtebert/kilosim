/*
    KiloSim

    Saves the parameters and time series logs of the Kilobot simulator in HDF5
    files

    Created 2018-10 by Julia Ebert
*/

#ifndef __KILOSIM_LOGGER_H
#define __KILOSIM_LOGGER_H

#include "H5PacketTable.h"
#include "robot.h"
#include <unordered_map>
#include <string>

namespace KiloSim
{
class Logger
{
  public:
    // Function that collects outputs from a robot into a single vector (row)
    // A pointer to a function from Robot pointers to a vector of doubles
    typedef std::vector<double> (*aggregatorFunc)(Robot *);
    // a typedef for our managed H5File pointer
    typedef std::shared_ptr<H5::H5File> H5FilePtr;

  protected:
    // HDF5 file where the data lives
    std::string fileID;
    // Trial number specifying group where the data lives
    int trialNum;
    // Names and functions of aggregators
    std::unordered_map<std::string, aggregatorFunc> aggregators;

  public:
    // Construct a Logger that logs within the given HDF5 file and group trialNum
    // File will be created if it doesn't exist
    Logger(std::string fileID, int trialNum);
    // Destructor: closes the file when it goes out of scope
    ~Logger();
    // Add an aggregator function that will be run on logState
    void addAggregator(std::string aggName, aggregatorFunc aggFunc);
    // Log the aggregators at the given time mapped over all the given robots
    void logState(double timeSec, Robot *robots);
    // Log all of the given parameters
    void logParams(std::unordered_map paramPairs);

  protected:
    // Log a single parameter name and value
    logParam(H5::H5Group *paramsGroup, std::string name, val);
    // Log data for this specific aggregator
    logAggregator(std::string aggName, aggregatorFunc aggFunc, Robot *robots);
    // Create or open an HDF5 file
    H5FilePtr createOrOpen(const std::string &fname);
}
} // namespace KiloSim

#endif