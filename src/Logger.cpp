/*
    KiloSim

    Created 2018-10 by Julia Ebert
*/

#include <typeinfo>
#include "Logger.h"

struct PostProcessingData
{
    char *datatype;
};
namespace KiloSim
{

Logger::Logger(World *world, std::string file_id, int trial_num, bool overwrite_trials)
    : m_file_id(file_id),
      m_trial_num(trial_num),
      m_world(world),
      m_overwrite_trials(overwrite_trials)
{
    // Create the HDF5 file if it doesn't already exist
    m_h5_file = create_or_open_file(file_id);
    // Create group for the trial. This should overwrite any existing group,
    // maybe with confirmation?
    m_trial_group_name = "trial_" + std::to_string(trial_num);
    m_params_group_name = m_trial_group_name + "/params";
    try
    {
        H5::Group *trialGroup = new H5::Group(m_h5_file->openGroup(m_trial_group_name.c_str()));
        if (m_overwrite_trials)
        {
            m_h5_file->unlink(m_trial_group_name.c_str());
            std::cout << "WARNING: Overwrote trial data" << std::endl;
        }
        else
        {
            printf("Conflicts with existing trial. Exiting to avoid data overwrite.\n");
            exit(EXIT_FAILURE);
        }
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
    FL_PacketTable *time_packet_table = new FL_PacketTable(
        m_h5_file->getId(), (char *)m_time_dset_name.c_str(), H5T_NATIVE_DOUBLE, 1);
    if (!time_packet_table->IsValid())
    {
        fprintf(stderr, "WARNING: Failed to create time series");
    }
    m_time_table = H5PacketTablePtr(time_packet_table);
}

Logger::~Logger(void)
{
    std::cout << "TODO: Close the file when out of scope?" << std::endl;
}

void Logger::add_aggregator(std::string agg_name, aggregatorFunc agg_func)
{
    m_aggregators.insert({{agg_name, agg_func}});

    // Do a test run of the aggregator to get the length of the output
    std::vector<double> test_output = (*agg_func)(m_world->get_robots());

    hsize_t out_len[1] = {test_output.size()};
    H5::ArrayType agg_type(H5::PredType::NATIVE_DOUBLE, 1, out_len);
    std::make_shared<H5::ArrayType>(agg_type);

    // Create a packet table and save it
    std::string agg_dset_name = m_trial_group_name + "/" + agg_name;
    FL_PacketTable *agg_packet_table = new FL_PacketTable(
        m_h5_file->getId(), (char *)agg_dset_name.c_str(), agg_type.getId(), 1);
    if (!agg_packet_table)
    {
        fprintf(stderr, "WARNING: Failed to create aggregator table");
    }
    m_aggregator_dsets.insert({{agg_name, H5PacketTablePtr(agg_packet_table)}});
}

void Logger::log_state()
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/

    // Add the current time to the time series
    double t = m_world->get_time();
    herr_t err = m_time_table->AppendPacket(&t);
    if (err < 0)
        fprintf(stderr, "WARNING: Failed to append to time series");

    for (std::pair<std::string, aggregatorFunc> agg : m_aggregators)
    {
        // Not sure about passing the pointer / reference here ?
        log_aggregator(agg.first, agg.second);
    }
}

void Logger::log_aggregator(std::string agg_name, aggregatorFunc agg_func)
{
    // Call the aggregator function on the robots
    std::vector<double> agg_val = (*agg_func)(m_world->get_robots());
    // TODO: append to the packet table (should be created by add_aggregator)
    herr_t err = m_aggregator_dsets.at(agg_name)->AppendPacket(agg_val.data());
    if (err < 0)
    {
        fprintf(stderr, "WARNING: Failled to append data to aggregator table");
    }
}

void Logger::log_params(Params param_pairs)
{
    for (std::pair<std::string, double> param_pair : param_pairs)
    {
        log_param(param_pair.first, param_pair.second);
    }
}

void Logger::log_config(ConfigParser *config)
{
    json j = config->get();
    for (auto &mol : j.get<json::object_t>())
    {
        log_param(mol.first, mol.second);
    }
}

void Logger::log_param(std::string name, json val)
{
    // Example: https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5group.cpp
    // https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5tutr_crtgrpd.cpp

    std::string dset_name = m_params_group_name + "/" + name;

    // Get the type of the parameter
    if (val.type() == json::value_t::object)
    {
        printf("WARNING: Cannot save param '%s' (currently no support for JSON 'object' type)\n", name.c_str());
    }
    else if (val.type() == json::value_t::array)
    {
        // TODO: Implement non-scalar parameters
        printf("WARNING: Cannot save param '%s' (currently no support for JSON 'array' type)\n", name.c_str());
    }
    else
    {
        H5::PredType val_type = h5_type(val);
        H5::DataSpace *dataspace = new H5::DataSpace();
        if (val_type == H5::PredType::C_S1)
        {
            std::string str_val = val.get<std::string>();
            hid_t strtype = H5Tcopy(H5T_C_S1);
            H5Tset_size(strtype, H5T_VARIABLE);
            H5::DataSet dataset = m_h5_file->createDataSet(dset_name, strtype, *dataspace);
            dataset.write(&str_val, strtype);
        }
        else
        {
            H5::DataSet *dataset = new H5::DataSet(m_h5_file->createDataSet(dset_name, val_type, *dataspace));
            // Save scalar...

            if (val_type == H5::PredType::NATIVE_HBOOL)
            {
                bool bool_val = val.get<bool>();
                dataset->write(&bool_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_INT)
            {
                int int_val = val.get<int>();
                dataset->write(&int_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_UINT)
            {
                uint uint_val = val.get<uint>();
                dataset->write(&uint_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_DOUBLE)
            {
                double double_val = val.get<double>();
                dataset->write(&double_val, val_type);
            }
        }
    }
}

H5::PredType Logger::h5_type(json j)
{
    return m_json_h5_types.at(j.type());
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

Logger::H5GroupPtr Logger::create_or_open_group(H5FilePtr file, std::string &group_name)
{
    // https://stackoverflow.com/q/35668056
    H5::Exception::dontPrint();
    H5::Group *group;
    try
    {
        group = new H5::Group(file->openGroup(group_name));
    }
    catch (H5::Exception &err)
    {
        group = new H5::Group(file->createGroup(group_name));
    }
}

} // namespace KiloSim
