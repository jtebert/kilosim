/*
    Kilosim

    Created 2018-11 by Julia Ebert
*/

#include "ConfigParser.h"

namespace Kilosim
{
ConfigParser::ConfigParser(const std::string config_file)
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
        std::cerr << "ERROR: Invalid or nonexistent config file: "
                  << config_file << std::endl;
        exit(EXIT_FAILURE);
    }
}

json ConfigParser::get(const std::string val_name) const
{
    try
    {
        return m_config.at(val_name);
    }
    catch (nlohmann::detail::out_of_range)
    {
        std::cerr << "[ConfigParser.get()] ERROR: Invalid or nonexistent key: " << val_name << std::endl;
        exit(EXIT_FAILURE);
    }
}
json ConfigParser::get() const
{
    return m_config;
}

} // namespace Kilosim