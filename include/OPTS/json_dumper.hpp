#ifndef JSON_DUMPER_HPP_
#define JSON_DUMPER_HPP_

#include "output_class.hpp"

namespace opts {

/**
 * @brief Dump the result data to a json file
 *
 * @param filename Filename to dump to
 * @param out_data Algorithm output data
 * @param exec_time Execution time of the algorithm
 */
void dump_to_json(const std::string &filename, const Output_data &out_data,
                  int64_t exec_time);

} // namespace opts

#endif