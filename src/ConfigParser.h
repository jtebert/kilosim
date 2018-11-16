/*
    KiloSim

    Created 2018-11 by Julia Ebert
*/

#ifndef __KILOSIM_CONFIGPARSER_H
#define __KILOSIM_CONFIGPARSER_H

#include <string>

namespace KiloSim
{
/*!
 * A ConfigParser is used to parse and process JSON configuration files for
 * user-provided simulation management. It also provides an option to directly
 * save all parameters to the logger HDF5 file (for consolidation).
 */
class ConfigParser
{
  public:
    ConfigParser(std::string config_file);
};
} // namespace KiloSim

#endif