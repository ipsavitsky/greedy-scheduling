#include "OPTS/options.hpp"
#include "toml.hpp"

#include <boost/describe/enum_from_string.hpp>

namespace opts {

/**
 * @brief Parse base config.
 *
 * @param data toml tag to start parsing from.
 * @retval base_config Generalised configuration parameters.
 */
base_config parse_base_config(toml::value &data) {
    base_config res;
    std::string criteria = toml::find<std::string>(data, "criteria");
    boost::describe::enum_from_string(criteria.c_str(), res.criteria);
    res.BF_bound = toml::find<double>(data, "BF_bound");
    res.CR_bound = toml::find<double>(data, "CR_bound");
    res.CR2_bound = toml::find<double>(data, "CR2_bound");
    std::string input_class = toml::find<std::string>(data, "inp_class");
    boost::describe::enum_from_string(input_class.c_str(), res._class);
    return res;
}

greedy_config parse_greedy_config(std::string filename) {
    toml::value all_data = toml::parse(filename);
    toml::value &base_section = toml::find(all_data, "general");
    base_config base = parse_base_config(base_section);
    greedy_config res(base);
    toml::value &greedy_section = toml::find(all_data, "greedy");
    std::string gc2_scheme =
        toml::find<std::string>(greedy_section, "GC2_scheme");
    boost::describe::enum_from_string(gc2_scheme.c_str(), res.scheme);
    res.threshold = toml::find<double>(greedy_section, "threshold");
    res.cr_con = toml::find<bool>(greedy_section, "cr_con");
    return res;
}

} // namespace opts