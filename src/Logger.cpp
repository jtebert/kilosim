/*
    Kilosim

    Created 2018-10 by Julia Ebert
*/

#include <kilosim/Logger.h>

#include <typeinfo>

namespace Kilosim
{

Logger::Logger(World &world, std::string const file_id, int const trial_num,
               bool const overwrite_trials)
    : m_world(world),
      m_file_id(file_id),
      m_overwrite_trials(overwrite_trials)
{
    // Create the HDF5 file if it doesn't already exist
    m_h5_file = create_or_open_file(file_id);
    set_trial(trial_num);
}

Logger::~Logger(void)
{
    m_h5_file->close();
}

void Logger::set_trial(uint const trial_num)
{
    m_trial_num = trial_num;
    // Create group for the trial
    m_trial_group_name = "trial_" + std::to_string(trial_num);
    m_params_group_name = m_trial_group_name + "/params";
    try
    {
        H5::Group *trialGroup = new H5::Group(
            m_h5_file->openGroup(m_trial_group_name.c_str()));
        (void)trialGroup; //Suppresses warning abut trialGroup variable not being used
        if (m_overwrite_trials)
        {
            m_h5_file->unlink(m_trial_group_name.c_str());
            fprintf(stderr, "WARNING: Overwrote trial data\n");
        }
        else
        {
            fprintf(stderr, "Conflicts with existing trial. Exiting to avoid data overwrite.\n");
            m_h5_file->close();
            exit(EXIT_FAILURE);
        }
    }
    catch (H5::FileIException &)
    {
    }
    H5::Group *trialGroup =
        new H5::Group(m_h5_file->createGroup(m_trial_group_name.c_str()));
    (void)trialGroup; // Suppresses warning abut trialGroup variable not being used

    // Create the params group if it doesn't already exist
    m_params_group = create_or_open_group(m_h5_file, m_params_group_name);

    // Create a packet table dataset for the timeseries
    m_time_dset_name = m_trial_group_name + "/time";
    FL_PacketTable *time_packet_table = new FL_PacketTable(
        m_h5_file->getId(),
        (char *)m_time_dset_name.c_str(),
        H5T_NATIVE_DOUBLE, 1);
    if (!time_packet_table->IsValid())
    {
        fprintf(stderr, "WARNING: Failed to create time series");
    }
    m_time_table = H5PacketTablePtr(time_packet_table);
}

uint Logger::get_trial() const
{
    return m_trial_num;
}

void Logger::add_aggregator(std::string const agg_name,
                            aggregatorFunc const agg_func)
{
    m_aggregators.insert({{agg_name, agg_func}});

    // Do a test run of the aggregator to get the length of the output
    const std::vector<double> test_output = (*agg_func)(m_world.get_robots());

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

void Logger::log_state() const
{
    // https://thispointer.com/how-to-iterate-over-an-unordered_map-in-c11/
    // Add the current time to the time series
    double t = m_world.get_time();
    herr_t err = m_time_table->AppendPacket(&t);
    if (err < 0)
        fprintf(stderr, "WARNING: Failed to append to time series");

    for (std::pair<std::string, aggregatorFunc> agg : m_aggregators)
    {
        // Not sure about passing the pointer / reference here ?
        log_aggregator(agg.first, agg.second);
    }
}

void Logger::log_aggregator(std::string const agg_name,
                            aggregatorFunc const agg_func) const
{
    // Call the aggregator function on the robots
    std::vector<double> agg_val = (*agg_func)(m_world.get_robots());
    // TODO: append to the packet table (should be created by add_aggregator)
    herr_t err = m_aggregator_dsets.at(agg_name)->AppendPacket(agg_val.data());
    if (err < 0)
    {
        fprintf(stderr, "WARNING: Failed to append data to aggregator table");
    }
}

void Logger::log_config(ConfigParser &config, const bool show_warnings)
{
    const json j = config.get();
    for (auto &mol : j.get<json::object_t>())
    {
        log_param(mol.first, mol.second, show_warnings);
    }
}

void Logger::log_param(const std::string name, const json val)
{
    log_param(name, val, true);
}

void Logger::log_param(const std::string name, const json val, const bool show_warnings)
{
    // Example: https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5group.cpp
    // https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/c++/examples/h5tutr_crtgrpd.cpp

    std::string dset_name = m_params_group_name + "/" + name;

    // Get the type of the parameter
    if (val.type() == json::value_t::object)
    {
        if (show_warnings)
            printf("WARNING: Cannot save param '%s' (currently no support for JSON 'object' type)\n",
                   name.c_str());
    }
    else if (val.type() == json::value_t::array)
    {
        // TODO: Implement non-scalar parameters
        if (show_warnings)
            printf("WARNING: Cannot save param '%s' (currently no support for JSON 'array' type)\n",
                   name.c_str());
    }
    else
    {
        H5::PredType val_type = h5_type(val);
        H5::DataSpace dataspace;
        if (val_type == H5::PredType::C_S1)
        {
            std::string str_val = val.get<std::string>();
            hid_t strtype = H5Tcopy(H5T_C_S1);
            H5Tset_size(strtype, H5T_VARIABLE);
            H5::DataSet dataset =
                m_h5_file->createDataSet(dset_name, strtype, dataspace);
            dataset.write(&str_val, strtype);
        }
        else
        {
            H5::DataSet dataset =
                m_h5_file->createDataSet(dset_name, val_type, dataspace);
            // Save scalar...

            if (val_type == H5::PredType::NATIVE_HBOOL)
            {
                bool bool_val = val.get<bool>();
                dataset.write(&bool_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_INT)
            {
                int int_val = val.get<int>();
                dataset.write(&int_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_UINT)
            {
                uint uint_val = val.get<uint>();
                dataset.write(&uint_val, val_type);
            }
            else if (val_type == H5::PredType::NATIVE_DOUBLE)
            {
                double double_val = val.get<double>();
                dataset.write(&double_val, val_type);
            }
        }
    }
}

void Logger::log_vector(const std::string vec_name, const std::vector<double> vec_val)
{
    std::string dset_name = m_trial_group_name + "/" + vec_name;
    hsize_t out_len[1] = {vec_val.size()};
    // H5::ArrayType agg_type(H5::PredType::NATIVE_DOUBLE, 1, out_len);
    // std::make_shared<H5::ArrayType>(agg_type);

    H5::DataType datatype(H5::PredType::NATIVE_DOUBLE);
    H5::DataSpace dataspace(1, out_len);
    H5::DataSet dataset = m_h5_file->createDataSet(dset_name, datatype, dataspace);
    dataset.write(vec_val.data(), datatype);
    // H5::DataSet *dataset = new H5::DataSet(
    //     m_h5_file->createDataSet(vec_name, H5::PredType::NATIVE_DOUBLE, *dataspace));
    // dataset->write(vec_val.data(), H5::PredType::NATIVE_DOUBLE);
}

H5::PredType Logger::h5_type(const json j) const
{
    return m_json_h5_types.at(j.type());
}

Logger::H5FilePtr Logger::create_or_open_file(const std::string &fname)
{
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

Logger::H5GroupPtr Logger::create_or_open_group(H5FilePtr file,
                                                const std::string &group_name)
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
    return H5GroupPtr(group);
}

} // namespace Kilosim
