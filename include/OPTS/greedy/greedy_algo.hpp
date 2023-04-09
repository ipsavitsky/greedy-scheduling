#ifndef GREEDY_ALGO_HPP_
#define GREEDY_ALGO_HPP_

#include "OPTS/greedy/schedule.hpp"
#include "OPTS/greedy/time_schedule.hpp"

namespace opts {
namespace greedy {

/**
 * @brief This is the Greedy_GC1 algorithm.
 * 
 * @param schedule All specialized greedy algorithm input data. 
 * @param conf Greedy algorithm configuration.
 * @retval TimeSchedule Constructed schedule.
 */
TimeSchedule construct_time_schedule(Schedule &schedule,
                                     opts::greedy_config conf);

/**
 * @brief This is the Greedy_EDF algorithm.
 * 
 * @param sched All specialized greedy algorithm input data. 
 * @param conf Greedy algorithm configuration.
 * @throws std::runtime_error If ann error occured during inserting a task. 
 * @retval TimeSchedule Constructed schedule.
 */
TimeSchedule greedy_EDF_heuristic(Schedule &sched, opts::greedy_config conf);

} // namespace greedy
} // namespace opts

#endif
