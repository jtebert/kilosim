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
    H5FilePtr h5File = createOrOpen(fileID);
}

Logger::~Logger(void)
{
    std::cout << "TODO: Close the file when out of scope" << std::endl;
}

void Logger::addAggregator(std::string aggName, aggregatorFunc aggFunc)
{
    //std::pair<std::string, aggregatorFunc> agg(aggName, aggFunc);
    aggregators.insert({{aggName, s aggFunc}});
}

void Logger::logState(double timeSec, Robot *robots)
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/
    for (std::pair<std::string, aggregatorFunc> agg : aggregators)
    {
        // Not sure about passing the pointer here?
        logAggregator(agg.first, agg.second, robots);
    }
}

void Logger::logParams(std::unordered_map paramPairs)
{
    // Create the params group if it doesn't already exist (ugly)
    // https://stackoverflow.com/q/35668056
    H5::Group *paramsGroup;
    try
    {
        paramsGroup = new H5::Group(h5file->openGroup("params"));
    }
    catch (H5::Exception &err)
    {
        paramsGroup = new H5::Group(h5file->createGroup("params"));
    }
    for (std::pair<std::string, int> paramPair : paramPairs)
    {
        logParam(paramPair.first, paramPair.second);
    }
    // Close the group
    delete paramsGroup;
}

void Logger::logParam(H5::H5Group *paramsGroup, std::string name, val)
{
    // Create a dataset in the 'params' group with the given value
    // Example: https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5group.cpp
    H5::DataSpace dataspace(H5S_SCALAR);
    // Get the matching parameter type to save
    PredType &valType;
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
    }
    H5::DataSet *dataset = new H5::DataSet(paramsGroup->createDataSet(name), valType, *dataspace, val);
    delete dataset;
    delete dataspace;
}

void Logger::logAggregator(std::string aggName, aggregatorFunc aggFunc, Robot *robots)
{
    (*aggFunc)(robots); // Call the aggregator function on the robots
}

H5FilePtr Logger::H5FilePtr createOrOpen(const std::string &fname)
{
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
