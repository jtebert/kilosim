/*
    KiloSim

    Created 2018-11 by Julia Ebert
*/

#ifndef __KILOSIM_CONFIGPARSER_H
#define __KILOSIM_CONFIGPARSER_H

#include <string>
#include <fstream>
#include "../include/json.hpp"

// for convenience
using json = nlohmann::json;

namespace KiloSim
{
/*!
 * A ConfigParser is used to parse and process JSON configuration files for
 * user-provided simulation management. It also provides an option to directly
 * save all parameters to the Logger HDF5 file (for consolidation).
 */
class ConfigParser
{
protected:
  //! Name/location of the file configuration comes from
  std::string m_config_file;
  //! Internal JSON representation of the config retrieved from file
  json m_config;

public:
  /*!
   * Create a parser to handle the values in the given JSON file
  * This will automatically load the contents into the parser
   * @param config_file Name/location of JSON file for config.
   */
  ConfigParser(std::string config_file);
  /*!
   * Get a value from the configuration by name.
   *
   * This can be automatically cast to the native type by defining the type on
   * the returned value.
   * For example, `int my_val = config.get("key_to_int_val");` will cast the
   * output to an integer. If you use `auto`, I think you'll get back the `json`
   * type.
   * Alternatively, you can directly pass the output to a function taking the
   * relevant type without needing to explicitly specify the type.
   *
   * @param val_name Name/key to get the value for
   * @returns Wrapped output value. Use .type_name() to get the type
   */
  json get(std::string val_name);
  /*!
   * Get the whole JSON object from the parser
   */
  json get();
};
} // namespace KiloSim

#endif