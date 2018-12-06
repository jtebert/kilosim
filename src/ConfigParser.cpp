/*
    KiloSim

    Created 2018-11 by Julia Ebert
*/

#include "ConfigParser.h"

namespace KiloSim
{
ConfigParser::ConfigParser(std::string config_file)
    : m_config_file(config_file)
{
    // Read from the config file into a JSON object
    std::ifstream in_json(config_file);
    try
    {
        in_json >> m_config;
    }
    catch (nlohmann::detail::parse_error)
    {
        std::cout << "ERROR: Invalid or nonexistent config file: " << config_file << std::endl;
        exit(EXIT_FAILURE);
    }
}

json ConfigParser::get(std::string val_name)
{
    return m_config[val_name];
}
json ConfigParser::get()
{
    return m_config;
}

} // namespace KiloSim