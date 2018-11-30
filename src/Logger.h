/*
    KiloSim

    Saves the parameters and time series logs of the Kilobot simulator in HDF5
    files

    Created 2018-10 by Julia Ebert
*/

#ifndef __KILOSIM_LOGGER_H
#define __KILOSIM_LOGGER_H

#include <H5PacketTable.h>
#include <H5Cpp.h>
#include "Robot.h"
#include "KiloSim.h"
#include "ConfigParser.h"
#include "../include/json.hpp"
#include <unordered_map>
#include <string>
#include <memory>
#include <vector>

using json = nlohmann::json;

namespace KiloSim
{
/*!
 * A Logger is used to save [HDF5](https://portal.hdfgroup.org/display/support)
 * files containing parameters and continuous state information for multiple
 * simulation trials.
 *
 * State information is logged through aggregator functions, which reduce the
 * state of the robots to a vector. This could be a single average value over
 * all the robots (e.g., mean observed ambient light) all the way to saving a
 * value for every robot (e.g., each robot's ambient light value as an
 * element). Each aggregator is saved as an array, where each row is the
 * output of the aggregator function. Whenever the state is logged, the
 * simulator time is also saved as a timeseries.
 *
 * The resulting file structure would look as follows:
 *
 * ```
 * logFile.h5
 * |__ trial_1  (group)
 * |   |__ time (dataset)  [1 x t]
 * |   |__ params (group)
 * |   |   |__ param1 (dataset)
 * |   |   |__ param2 (dataset)
 * |   |   |__ ...
 * |   |__ aggregator_1 (dataset)  [n x t]
 * |   |__ aggregator_2 (dataset)  [m x t]
 * |   |__ ...
 * |__ trial_2
 * |   |__ time (dataset)  [1 x t]
 * |   |__ params
 * |   |   |__ ...
 * |   |__ ...
 * ...
 * ```
 *
 * where `t` is the number of time steps when data was logged, and
 * `aggregator_1` and `aggregator_2` were specified by the user.
 *
 * Logging (I/O in general) is one of the *slowest* parts of the simulation. As
 * such, a high logging rate will significantly slow down your simulations. Try
 * something like logging every 5-10 seconds, if you can get away with it.
 *
 * If you delete an HDF5 file, it does NOT actually delete the data; it just
 * removes the reference to it. Therefore, rather than extensive use of the
 * overwrite flag, if you plan to completely redo a set of simulations, delete
 * the file and let it regenerate. (However, there are tools for removing this
 * pseudo-deleted data later.)
 *
 * **NOTE:** The Logger does **not** provide functionality for reading/viewing
 * log files once created. (It's kind of a pain in C++. I recommend using
 * [h5py](https://www.h5py.org/) instead.)
 */
class Logger
{
public:
  /*!
   * A function mapping Robots to values
   * This is used by #log_state to compute values to save from all of the
   * robots at a time step. The output will form a row in an H5::PacketTable. It
   * may be one value per robot (e.g., the current motor command) or all the way
   * to one combined value from all the robots (e.g., the mean light perceived).
   *
   * Functions with this signature are the inputs to #add_aggregator
   */
  typedef std::vector<double> (*aggregatorFunc)(std::vector<Robot *> &robots);

protected:
  //! Managed H5File pointer
  typedef std::shared_ptr<H5::H5File> H5FilePtr;
  //! Managed pointer to HDF5 Group
  typedef std::shared_ptr<H5::Group> H5GroupPtr;
  //! Managed pointer to HDF5 PacketTable
  typedef std::shared_ptr<FL_PacketTable> H5PacketTablePtr;
  typedef std::unordered_map<std::string, double> Params;
  //! Pointer to KiloSim World that this Logger tracks
  World *m_world;
  //! HDF5 file where the data lives
  std::string m_file_id;
  //! Whether or not to override existing data trial groups (this is done at the group level); defaults to false
  bool m_overwrite_trials;
  //! Trial number specifying group where the data lives
  uint m_trial_num;
  //! Names and functions of aggregators
  std::unordered_map<std::string, aggregatorFunc> m_aggregators;
  //! Names and HDF5 datasets (PacketTables) for every aggregator
  std::unordered_map<std::string, H5PacketTablePtr> m_aggregator_dsets;
  //! Opened HDF5 file where this Logger saves
  H5FilePtr m_h5_file;
  //! HDF5 group name for trial. e.g., /trial_0
  std::string m_trial_group_name;
  //! HDF5 group name for trial parameters. e.g., /trial_0/params
  std::string m_params_group_name;
  //! HDF5 group opened within the file for storing/saving parameters
  H5GroupPtr m_params_group;
  //! HDF5 dataset name for time series (packet table)
  std::string m_time_dset_name;
  //! HDF5 PacketTable used to track the time (in seconds) when logging state
  H5PacketTablePtr m_time_table;
  //! Conversion from JSON types to HDF5 types (NOTE: only works for atomic datatypes)
  std::unordered_map<json::value_t, H5::PredType> m_json_h5_types = {
      {json::value_t::boolean, H5::PredType::NATIVE_HBOOL},
      {json::value_t::number_integer, H5::PredType::NATIVE_INT},
      {json::value_t::number_unsigned, H5::PredType::NATIVE_UINT},
      {json::value_t::number_float, H5::PredType::NATIVE_DOUBLE},
      {json::value_t::string, H5::PredType::C_S1},
  };

public:
  /*!
   * Construct a Logger that logs within the given HDF5 file and group trial_num
   *
   * If the HDF5 file does not exist, it will be created.
   *
   * __WARNING:__ Data will be overwritten if a group already exists in the file
   * for the given trial number. You'll get a warning, but it will be too late.
   * (This should be fixed with an overwrite flag in a future version.)
   *
   * @param world The world from which state and parameters will be logged
   * @param file_id Name and location of the HDF5 file to log stuff in
   * @param trial_num Number of the trial to save the data. Data will be saved
   * in a group named "trial_#", where # is trial_num.
   */
  Logger(World *world, std::string file_id, int trial_num, bool overwrite_trials = false);
  //! Destructor: closes the file when it goes out of scope
  ~Logger();

  /*!
   * Set which trial is being logged.
   * This allows you to create a single logger and use it for many trials,
   * without having to re-initialize and re-add your aggregators. You DO have
   * to manually re-log parameters for each trial, if you want to do that.
   *
   * The same overwrite_trials flag from initialization is still in effect
   *
   * @param trial_num New trial you want to log.
   */
  void set_trial(uint trial_num);

  /*!
   * Get/check which trial the Logger is currently set to log data/parameters
   * for.
   * @return Current trial. Change with set_trial()
   */
  uint get_trial();

  /*!
   * Add an aggregator function that will be run on log_state when #log_state is
   * called, all aggregator functions will be called on the robots in the World.
   * The output is saved as a row in a dataset named by #agg_name
   *
   * @param agg_name Name of the dataset in with to store the output of the
   * agg_func. This exists within the trial_# group.
   * @param agg_func Aggregator that saves values from the Robots in the World.
   * Each output is saved as a row in the dataset.
   */
  void add_aggregator(std::string agg_name, aggregatorFunc agg_func);

  /*!
   * Log the aggregators at the given time mapped over all the given robots in
   * the World. Every time this is called, the current time (in seconds) is
   * added to the time series, and a row is appended to every aggregator array.
   */
  void log_state();

  /*!
   * Log all of the values in the configuration as params in the HDF5 file/trial
   *
   * **NOTE:** This only supports atomic datatypes (bool, int, uint, float,
   * string). Any non-atomic data (arrays and objects) will be skipped (with a
   * warning displayed in the terminal).
   *
   * @param config Loaded configuration for this experiment/trial
   */
  void log_config(ConfigParser *config);

protected:
  //! Log data for this specific aggregator
  void log_aggregator(std::string agg_name, aggregatorFunc agg_func);
  //! Log a single parameter name and value (supports atomic datatypes only)
  void log_param(std::string name, json val);
  //! Get the H5 data type (for saving) from the JSON
  H5::PredType h5_type(json j);
  //! Create or open an HDF5 file
  H5FilePtr create_or_open_file(const std::string &fname);
  //! Create or open a group in an HDF5 file
  H5GroupPtr create_or_open_group(H5FilePtr file, std::string &group_name);
};

/*! \example example_logger.cpp
 * Example minimal usage of a Logger
 */
} // namespace KiloSim

#endif
