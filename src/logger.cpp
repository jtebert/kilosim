/*
    KiloSim

    Created 2018-10 by Julia Ebert
*/

#include <typeinfo>
#include "logger.h"

namespace KiloSim
{
Logger::Logger(std::string fileID, int trialNum) : fileID(fileID),
                                                   trialNum(trialNum)
{
    // Create the HDF5 file if it doesn't already exist
    // TODO: Save this pointer?
    h5fileP = createOrOpen(fileID);
    // TODO: Create group for the trial. This should overwrite any existing group,
    // maybe with confirmation?
}

Logger::~Logger(void)
{
    std::cout << "TODO: Close the file when out of scope" << std::endl;
}

void Logger::addAggregator(std::string aggName, aggregatorFunc aggFunc)
{
    //std::pair<std::string, aggregatorFunc> agg(aggName, aggFunc);
    aggregators.insert({{aggName, aggFunc}});
    // TODO: Create a packet table and save it
}

void Logger::logState(double timeSec, std::vector<Robot> &robots)
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/
    for (std::pair<std::string, aggregatorFunc> agg : aggregators)
    {
        // Not sure about passing the pointer here?
        logAggregator(agg.first, agg.second, robots);
    }
}

void Logger::logParams(std::unordered_map<std::string, double> paramPairs)
{
    // Create the params group if it doesn't already exist (ugly)
    // https://stackoverflow.com/q/35668056
    H5::Group *paramsGroup;
    try
    {
        paramsGroup = new H5::Group(h5fileP->openGroup("/params"));
    }
    catch (H5::Exception &err)
    {
        paramsGroup = new H5::Group(h5fileP->createGroup("/params"));
    }
    for (std::pair<std::string, int> paramPair : paramPairs)
    {
        logParam(paramPair.first, paramPair.second);
    }
    // Close the group
    delete paramsGroup;
}

void Logger::logParam(std::string name, double val)
{
    // Create a dataset in the 'params' group with the given value
    // Example: https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5group.cpp
    H5::DataSpace *dataspace = new H5::DataSpace(H5S_SCALAR);
    // Get the matching parameter type to save
    /*PredType &valType;
    switch ((typeid(val)))
    {
    case typeid(int):
        valType = H5::PredType::NATIVE_INT;
        break;
    case typeid(float):
        valType = H5::PredType::NATIVE_FLOAT;
        break;
    case typeid(bool):
        valType = H5::PredType::NATIVE_H_BOOL;
        break;
    }*/
    std::string dsetName = "/params" + name;
    H5::DataSet *dataset = new H5::DataSet(h5fileP->createDataSet(dsetName.c_str(), H5::PredType::NATIVE_FLOAT, *dataspace, val));
    delete dataset;
    delete dataspace;
}

void Logger::logAggregator(std::string aggName, aggregatorFunc aggFunc, std::vector<Robot> &robots)
{
    // Call the aggregator function on the robots
    std::vector<double> aggVal = (*aggFunc)(robots);
    // TODO: append to the packet table (should be created by addAggregator)
}

Logger::H5FilePtr Logger::createOrOpen(const std::string &fname)
{
    // TODO: Not sure if I'm gonna keep this. Might only be used once and I don't understand the shared pointer thing
    // From: https://stackoverflow.com/a/13849946
    H5::Exception::dontPrint();
    H5::H5File *file = 0;
    try
    {
        file = new H5::H5File(fname.c_str(), H5F_ACC_RDWR);
    }
    catch (const H5::FileIException &)
    {
        file = new H5::H5File(fname.c_str(), H5F_ACC_TRUNC);
    }
    return H5FilePtr(file);
}
} // namespace KiloSim