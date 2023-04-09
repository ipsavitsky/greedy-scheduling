#include <fstream>
#include <list>
#include <memory>
#include <vector>

#include "OPTS/graph_part.hpp"
#include "OPTS/logger_config.hpp"

namespace opts {

/**
 * @brief Calculate `BF` values on a partitioning
 *
 * Since `BF` is a property of task-to-processor assignment, it can be
 * calculated on a partitioning.
 *
 * @param partitioning Partitioning vector.
 * @param s Generalised input data
 * @retval double Partition `BF` value
 */
double calculate_partitioned_BF(const std::vector<std::size_t> &partitioning,
                                const greedy::ScheduleData &s) {
    BOOST_LOG_NAMED_SCOPE("calculate_BF");
    std::vector<std::size_t> am_of_tasks_on_proc(s.proc_num);
    std::fill(am_of_tasks_on_proc.begin(), am_of_tasks_on_proc.end(), 0);
    std::for_each(partitioning.begin(), partitioning.end(),
                  [&am_of_tasks_on_proc](const auto &elem) {
                      am_of_tasks_on_proc[elem] += 1;
                  });
    auto max_load = *std::max_element(am_of_tasks_on_proc.begin(),
                                      am_of_tasks_on_proc.end());
    double BF = 100 * ((max_load * s.proc_num / (double)s.task_num) - 1);
    return std::ceil(BF);
}

/**
 * @brief Calculate `CR` values on a partitioning
 *
 * Since `CR` is a property of task-to-processor assignment, it can be
 * calculated on a partitioning.
 *
 * @param partitioning Partitioning vector.
 * @param s Generalised input data
 * @retval double Partition `CR` value
 */
double calculate_partitioned_CR(const std::vector<std::size_t> &partitioning,
                                const greedy::ScheduleData &s) {
    BOOST_LOG_NAMED_SCOPE("calculate_CR")
    auto [cur, end] = boost::edges(s.graph);
    std::size_t cut_edges = 0;
    for (; cur != end; ++cur) {
        if (partitioning[cur->m_source] != partitioning[cur->m_target]) {
            ++cut_edges;
        }
    }
    return cut_edges / (double)s.edges;
}

CSR adjcy2CSR(const greedy::ScheduleData::Graph &graph) {
    BOOST_LOG_NAMED_SCOPE("adjcy2CSR");
    CSR res;
    std::size_t vtxs = boost::num_vertices(graph);
    res.xadj.resize(vtxs + 1);
    for (std::size_t i = 0; i < vtxs; ++i) {
        res.xadj[i] = res.adjcy.size();
        auto ei_in = boost::in_edges(i, graph);
        for (; ei_in.first != ei_in.second; ++ei_in.first) {
            auto from = ei_in.first->m_source;
            res.adjcy.push_back(from);
        }
        auto ei_out = boost::out_edges(i, graph);
        for (; ei_out.first != ei_out.second; ++ei_out.first) {
            auto to = ei_out.first->m_target;
            res.adjcy.push_back(to);
        }
    }
    res.xadj.back() = res.adjcy.size();
    LOG_DEBUG << "num_vertices: " << boost::num_vertices(graph)
              << "; num_edges: " << boost::num_edges(graph) << "; len(xadj)"
              << res.xadj.size() << "; len(adjcy): " << res.adjcy.size();
    return res;
}

std::tuple<std::vector<std::size_t>, uint64_t>
part_graph(CSR &csr, std::size_t num_parts, std::uint64_t ufactor,
           std::vector<std::size_t> first_row) {
    static unsigned long seed = time(nullptr);
    BOOST_LOG_NAMED_SCOPE("part_graph");
    idx_t nvexes = csr.xadj.size() - 1;
    idx_t ncon = 1;
    idx_t nparts = num_parts;
    idx_t edgecut;
    std::unique_ptr<idx_t[]> part(new idx_t[nvexes]);
    LOG_DEBUG << "nvexes: " << nvexes << "; ncon: " << ncon
              << "; nparts: " << nparts;
    idx_t m_options[METIS_NOPTIONS];
    METIS_SetDefaultOptions(m_options);
    m_options[METIS_OPTION_UFACTOR] = ufactor;
    m_options[METIS_OPTION_SEED] = seed;
    std::vector<idx_t> vweights(first_row.begin(), first_row.end());
    if (nparts > 8) {
        METIS_PartGraphRecursive(
            &nvexes, &ncon, csr.xadj.data(), csr.adjcy.data(), vweights.data(),
            NULL, NULL, &nparts, NULL, NULL, m_options, &edgecut, part.get());
    } else {
        METIS_PartGraphKway(&nvexes, &ncon, csr.xadj.data(), csr.adjcy.data(),
                            vweights.data(), NULL, NULL, &nparts, NULL, NULL,
                            m_options, &edgecut, part.get());
    }

    std::vector<std::size_t> result(part.get(), part.get() + nvexes);
    seed++;
    srand(seed);
    return std::make_tuple(result, edgecut);
}

/**
 * @brief Structure that represents partitioning on a processor.
 *
 */
struct parted_proc {
    std::size_t load; ///< Sum of durations of all tasks on a processor. Does
                      ///< not take possible gaps into account
    std::list<greedy::ScheduleData::Task>
        parted_tasks; ///< List of tasks allocated on a processor.
};

/**
 * @brief Convert a "deep" partition to a flat partition.
 *
 * Converts partitioning from a vector of parted_proc each representing a
 * processor to a partitioning where `partitioning[num_task]` is the assignment
 * to a processor. `num_task` is the number of the task.
 *
 * @param deep_partition A "deep" partition, where each processor contains all
 * distributed tasks.
 * @param am_of_tasks The number of tasks.
 * @retval std::vector<std::size_t> "Flat" partition.
 */
std::vector<std::size_t>
flatten_partition(const std::vector<parted_proc> &deep_partition,
                  std::size_t am_of_tasks) {
    BOOST_LOG_NAMED_SCOPE("flatten_partition");
    std::vector<std::size_t> new_partitioning(am_of_tasks);
    for (auto proc_it = deep_partition.begin(); proc_it != deep_partition.end();
         ++proc_it) {
        for (auto task : proc_it->parted_tasks) {
            // LOG_INFO << "task: " << task
            //          << std::distance(deep_partition.begin(), proc_it);
            new_partitioning[task] =
                std::distance(deep_partition.begin(), proc_it);
        }
    }
    return new_partitioning;
}

std::vector<std::size_t>
local_partition_optimization(const std::vector<std::size_t> &partition,
                             const greedy::ScheduleData &data,
                             const opts::base_config &conf) {
    BOOST_LOG_NAMED_SCOPE("local_partition_optimization");
    std::vector<parted_proc> proc_loads(data.proc_num);
    for (int i = 0; i < partition.size(); ++i) {
        auto proc_assign = partition[i];
        proc_loads[proc_assign].load += data.get_task_time(proc_assign, i);
        proc_loads[proc_assign].parted_tasks.push_back(i);
    }

    // LOG_DEBUG << "got proc_loads";

    while (true) {
        auto max_load_it =
            std::max_element(proc_loads.begin(), proc_loads.end(),
                             [](const parted_proc &f, const parted_proc &s) {
                                 return f.load < s.load;
                             });

        auto max_load_proc = std::distance(proc_loads.begin(), max_load_it);
        max_load_it->parted_tasks.sort(
            [&data, max_load_proc](const greedy::ScheduleData::Task &f,
                                   const greedy::ScheduleData::Task &s) {
                return data.get_task_time(max_load_proc, f) >
                       data.get_task_time(max_load_proc, s);
            });

        auto cur_dis_task = max_load_it->parted_tasks.begin();
        for (; cur_dis_task != max_load_it->parted_tasks.end();
             ++cur_dis_task) {
            std::optional<decltype(proc_loads)::iterator> transfer_proc =
                std::nullopt;
            for (auto cur_proc = proc_loads.begin();
                 cur_proc != proc_loads.end(); ++cur_proc) {
                if (cur_proc == max_load_it) {
                    continue;
                }
                auto new_load_from =
                    max_load_it->load -
                    data.get_task_time(max_load_proc, *cur_dis_task);

                auto cur_proc_num = std::distance(proc_loads.begin(), cur_proc);
                auto new_load_to =
                    cur_proc->load +
                    data.get_task_time(cur_proc_num, *cur_dis_task);

                if (std::max(new_load_from, new_load_to) <
                    std::max(max_load_it->load, cur_proc->load)) {
                    if (transfer_proc) {
                        auto dist = std::distance(proc_loads.begin(),
                                                  transfer_proc.value());
                        if (data.get_task_time(dist, *cur_dis_task) >
                            data.get_task_time(cur_proc_num, *cur_dis_task)) {
                            transfer_proc = cur_proc;
                        }
                    } else if (!transfer_proc) {
                        transfer_proc = cur_proc;
                    }
                }
            }
            if (transfer_proc) {
                // LOG_INFO << "transfering!!";
                max_load_it->load -=
                    data.get_task_time(max_load_proc, *cur_dis_task);
                auto task_in_question = *cur_dis_task;
                cur_dis_task = max_load_it->parted_tasks.erase(cur_dis_task);
                auto dist =
                    std::distance(proc_loads.begin(), transfer_proc.value());
                transfer_proc.value()->load +=
                    data.get_task_time(dist, task_in_question);
                transfer_proc.value()->parted_tasks.push_back(task_in_question);
                auto prelim_part = flatten_partition(proc_loads, data.task_num);
                if (((conf.criteria == extra_criteria::CR) &&
                     (calculate_partitioned_CR(prelim_part, data) >=
                      conf.CR_bound)) ||
                    ((conf.criteria == extra_criteria::BF) &&
                     (calculate_partitioned_BF(prelim_part, data) >=
                      conf.BF_bound))) {
                    // LOG_INFO << "mistake! transfering back!";
                    // LOG_INFO << max_load_proc << " " << dist;
                    // LOG_INFO << "moving " << task_in_question;
                    max_load_it->load +=
                        data.get_task_time(max_load_proc, task_in_question);
                    max_load_it->parted_tasks.push_front(task_in_question);
                    transfer_proc.value()->load -=
                        data.get_task_time(dist, task_in_question);
                    std::erase_if(
                        transfer_proc.value()->parted_tasks,
                        [task_in_question](const greedy::ScheduleData::Task &cur) {
                            return cur == task_in_question;
                        });
                    // transfer_proc.value()->parted_tasks.remove_if(cur_dis_task);
                    if (cur_dis_task == max_load_it->parted_tasks.end()) {
                        break;
                    }
                    continue;
                }
                break;
            }
        }
        if (cur_dis_task == max_load_it->parted_tasks.end()) {
            break;
        }
    }
    return flatten_partition(proc_loads, data.task_num);
}

} // namespace opts
