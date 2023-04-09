#ifndef OUTPUT_CLASS_HPP_
#define OUTPUT_CLASS_HPP_

#include "options.hpp"
#include "greedy/schedule_data.hpp"
#include <list>
#include <string>

namespace opts {

/**
 * @brief Class for representing constructed schedule.
 *
 * Contains start times of tasks, CR, CR2 and BF for schedule, number of nodes
 * etc.
 *
 */
class Output_data {
  public:
    /**
     * @brief The value of CR criterion.
     *
     */
    double CR;

    /**
     * @brief The value of CR2 criterion.
     *
     */
    double CR2;

    /**
     * @brief The value of BF criterion.
     *
     */
    double BF;

    /**
     * @brief Criterion that was used to construct the schedule.
     *
     */
    opts::extra_criteria criteria;

    /**
     * @brief Number of tasks in schedule.
     *
     */
    std::size_t nodes;

    /**
     * @brief Execution time of the constructed schedule.
     *
     */
    std::size_t time;

    /**
     * @brief Execution time of the initial constructed schedule for simulated
     * annealing algorithms.
     *
     */
    std::size_t start_time{0};

    /**
     * @brief Array of schedule durations received from Greedy_EDF in complex
     * algorithm.
     *
     */
    std::vector<std::size_t> start_times;

    /**
     * @brief Class represents tasks in constructed schedule.
     *
     */
    struct PlacedTask {
        greedy::ScheduleData::Task
            task_no;        ///< number of task in order that they
                            ///< are numerated in the graph.
        std::size_t start;  ///< task execution start time
        std::size_t finish; ///< task execution end time
    };

    /**
     * @brief A list of all tasks in order on a single processor.
     *
     */
    using proc_info = std::list<PlacedTask>;

    /**
     * @brief Vector where index represents the number of processor and values
     * are arrays of placed tasks.
     *
     */
    std::vector<proc_info> proc_array;
};

} // namespace opts

#endif