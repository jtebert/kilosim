/*
    KiloSim

    Created 2018-10 by Julia Ebert
*/

#include <typeinfo>
#include "logger.h"

namespace KiloSim
{

Logger::Logger(World *world, std::string file_id, int trial_num) : m_file_id(file_id),
                                                                   m_trial_num(trial_num),
                                                                   m_world(world)
{
    // Create the HDF5 file if it doesn't already exist
    m_h5_file = create_or_open_file(file_id);
    // Create group for the trial. This should overwrite any existing group,
    // maybe with confirmation?
    m_trial_group_name = "trial_" + std::to_string(trial_num);
    m_params_group_name = m_trial_group_name + "/params";
    try
    {
        m_h5_file->unlink(m_trial_group_name.c_str());
        std::cout << "WARNING: Overwrote trial data" << std::endl;
    }
    catch (H5::FileIException &)
    {
    }
    H5::Group *trialGroup = new H5::Group(m_h5_file->createGroup(m_trial_group_name.c_str()));
    delete trialGroup;

    // Create the params group if it doesn't already exist
    m_params_group = create_or_open_group(m_h5_file, m_params_group_name);

    // Create a packet table dataset for the timeseries
    m_time_dset_name = m_trial_group_name + "/time";
    FL_PacketTable *timePacketTable = new FL_PacketTable(
        m_h5_file->getId(), (char *)m_time_dset_name.c_str(), H5T_NATIVE_DOUBLE, 1);
    if (!timePacketTable->IsValid())
    {
        fprintf(stderr, "WARNING: Failed to create time series");
    }
    m_time_table = H5PacketTablePtr(timePacketTable);
}

Logger::~Logger(void)
{
    std::cout << "TODO: Close the file when out of scope?" << std::endl;
}

void Logger::add_aggregator(std::string agg_name, aggregatorFunc aggFunc)
{
    //std::pair<std::string, aggregatorFunc> agg(agg_name, aggFunc);
    aggregators.insert({{agg_name, aggFunc}});
    // TODO: Create a packet table and save it
}

void Logger::log_state()
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/

    // Add the current time to the time series
    double t = m_world->get_time();
    herr_t err = m_time_table->AppendPacket(&t);
    if (err < 0)
        fprintf(stderr, "WARNING: Failed to append to time series");

    for (std::pair<std::string, aggregatorFunc> agg : aggregators)
    {
        // Not sure about passing the pointer / reference here ?
        log_aggregator(agg.first, agg.second);
        // TODO: Create a packet table for each aggregator
    }
}

void Logger::log_params(Params paramPairs)
{
    for (std::pair<std::string, double> paramPair : paramPairs)
    {
        log_param(paramPair.first, paramPair.second);
    }
}

void Logger::log_param(std::string name, double val)
{
    // Create a dataset in the 'params' group with the given value
    // Example: https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5group.cpp
    // https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5tutr_crtgrpd.cpp

    std::string dsetName = m_params_group_name + "/" + name;

    // Save as array (saves something but truncates decimal values)
    // double data_arr[1];
    // data_arr[0] = val;
    // hsize_t dims[1];
    // dims[0] = 1;
    // H5::DataSpace *dataspace = new H5::DataSpace(1, dims);
    // H5::DataSet *dataset = new H5::DataSet(
    //     m_h5_file->createDataSet(dsetName, H5::PredType::NATIVE_DOUBLE, *dataspace));
    // dataset->write(data_arr, H5::PredType::NATIVE_DOUBLE);

    // Save scalar...
    H5::DataSpace *dataspace = new H5::DataSpace();
    H5::DataSet *dataset = new H5::DataSet(m_h5_file->createDataSet(dsetName, H5::PredType::NATIVE_DOUBLE, *dataspace));
    dataset->write(&val, H5::PredType::NATIVE_DOUBLE);

    delete dataset;
    delete dataspace;
}

void Logger::log_aggregator(std::string agg_name, aggregatorFunc aggFunc)
{
    // Call the aggregator function on the robots
    std::vector<double> aggVal = (*aggFunc)(m_world->get_robots());
    // TODO: append to the packet table (should be created by add_aggregator)
}

Logger::H5FilePtr Logger::create_or_open_file(const std::string &fname)
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

Logger::H5GroupPtr Logger::create_or_open_group(H5FilePtr file, std::string &groupName)
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