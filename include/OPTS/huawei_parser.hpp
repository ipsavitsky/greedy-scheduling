#ifndef HUAWEI_PARSER_HPP
#define HUAWEI_PARSER_HPP

#include "OPTS/options.hpp"
#include "OPTS/greedy/schedule_data.hpp"

#include <string>

namespace opts {

/**
 * @brief Upload all input data from a file
 *
 * The format of the file is the following: \n
 * `{task_num} {proc_num} {edge_num}` \n
 * `{task execution time matrix, proc_num x task_num}` \n
 * `{processors connectivity matrix, proc_num x proc_num}` \n
 * `{Pairs of edges are separated by a space, edge_num x 2}` \n
 *
 * Where task_num - number of tasks in th task graph, proc_num - number of processors
 * for scheduling, edge_num - number of edges in the task graph. For
 * data class 2, strings `{task execution time matrix, proc_num x task_num}`
 * are replaced with one number - execution time for every task.
 * 
 * @throws std::runtime_error Thrown if a non-file is passed.
 * @param path Path to file
 * @param inp_class Class of input data.
 * @retval greedy::ScheduleData Generalised input data.
 */
greedy::ScheduleData input_schedule_regular(std::string path,
                                  opts::input_class inp_class);

} // namespace opts

#endif