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
    FL_PacketTable *timePacketTable = new FL_PacketTable(
        h5fileP->getId(), (char *)timeDsetName.c_str(), H5T_NATIVE_DOUBLE, 1);
    if (!timePacketTable->IsValid())
    {
        fprintf(stderr, "WARNING: Failed to create time series");
    }
    timeTable = H5PacketTablePtr(timePacketTable);
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

void Logger::logState(double timeSec, std::vector<Robot *> &robots)
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/

    // Add the current time to the time series
    printf("Logger::logState\n");
    herr_t err = timeTable->AppendPacket(&timeSec);
    if (err < 0)
        fprintf(stderr, "WARNING: Failed to append to time series");

    for (std::pair<std::string, aggregatorFunc> agg : aggregators)
    {
        // Not sure about passing the pointer / reference here ?
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
    // https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5tutr_crtgrpd.cpp

    std::string dsetName = paramsGroupName + "/" + name;

    // Save as array (saves something but truncates decimal values)
    // double data_arr[1];
    // data_arr[0] = val;
    // hsize_t dims[1];
    // dims[0] = 1;
    // H5::DataSpace *dataspace = new H5::DataSpace(1, dims);
    // H5::DataSet *dataset = new H5::DataSet(
    //     h5fileP->createDataSet(dsetName, H5::PredType::NATIVE_DOUBLE, *dataspace));
    // dataset->write(data_arr, H5::PredType::NATIVE_DOUBLE);

    // Save scalar...
    H5::DataSpace *dataspace = new H5::DataSpace();
    H5::DataSet *dataset = new H5::DataSet(h5fileP->createDataSet(dsetName, H5::PredType::NATIVE_DOUBLE, *dataspace));
    dataset->write(&val, H5::PredType::NATIVE_DOUBLE);

    delete dataset;
    delete dataspace;
}

void Logger::logAggregator(std::string aggName, aggregatorFunc aggFunc, std::vector<Robot *> &robots)
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