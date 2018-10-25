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
    h5fileP = createOrOpenFile(fileID);
    // Create group for the trial. This should overwrite any existing group,
    // maybe with confirmation?
    trialGroupName = "trial_" + std::to_string(trialNum);
    paramsGroupName = trialGroupName + "/params";
    try
    {
        h5fileP->unlink(trialGroupName.c_str());
        std::cout << "WARNING: Overwrote trial data" << std::endl;
    }
    catch (H5::FileIException &)
    {
    }
    H5::Group *trialGroup = new H5::Group(h5fileP->createGroup(trialGroupName.c_str()));
    delete trialGroup;

    // Create the params group if it doesn't already exist
    paramsGroup = createOrOpenGroup(h5fileP, paramsGroupName);

    // Create a packet table dataset for the timeseries
    timeDsetName = trialGroupName + "/time";
    FL_PacketTable timeTable(h5fileP->getId(), (char *)timeDsetName.c_str(), H5T_NATIVE_FLOAT, 1);
    if (!timeTable.IsValid())
    {
        fprintf(stderr, "WARNING: Failed to create time series");
    }
    /* Append five packets to the packet table, one at a time */
    herr_t err;
    for (int x = 0; x < 5; x++)
    {
        float y = x;
        err = timeTable.AppendPacket(&y);
        if (err < 0)
            fprintf(stderr, "Error adding record.");
    }
}

Logger::~Logger(void)
{
    std::cout << "TODO: Close the file when out of scope?" << std::endl;
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
        // Not sure about passing the pointer/reference here?
        logAggregator(agg.first, agg.second, robots);
        // TODO: Create a packet table for each aggregator
    }
}

void Logger::logParams(Params paramPairs)
{
    for (std::pair<std::string, int> paramPair : paramPairs)
    {
        logParam(paramPair.first, paramPair.second);
    }
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
    std::cout << val << std::endl;
    std::string dsetName = paramsGroupName + "/" + name;

    H5::DataSet *dataset = new H5::DataSet(h5fileP->createDataSet(
        dsetName.c_str(),
        H5::PredType::NATIVE_DOUBLE,
        *dataspace,
        val));

    std::cout << "Created dataset" << std::endl;
    delete dataset;
    delete dataspace;
}

void Logger::logAggregator(std::string aggName, aggregatorFunc aggFunc, std::vector<Robot> &robots)
{
    // Call the aggregator function on the robots
    std::vector<double> aggVal = (*aggFunc)(robots);
    // TODO: append to the packet table (should be created by addAggregator)
}

Logger::H5FilePtr Logger::createOrOpenFile(const std::string &fname)
{
    // TODO: Not sure if I'm gonna keep this. Might only be used once and I don't understand the shared pointer thing
    // From: https://stackoverflow.com/a/13849946
    H5::Exception::dontPrint();
    H5::H5File *file;
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

Logger::H5GroupPtr Logger::createOrOpenGroup(H5FilePtr file, std::string &groupName)
{
    // https://stackoverflow.com/q/35668056
    H5::Exception::dontPrint();
    H5::Group *group;
    try
    {
        group = new H5::Group(file->openGroup(groupName));
    }
    catch (H5::Exception &err)
    {
        group = new H5::Group(file->createGroup(groupName));
    }
}

} // namespace KiloSim