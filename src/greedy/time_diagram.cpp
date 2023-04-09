#include "OPTS/greedy/time_diagram.hpp"
#include "OPTS/logger_config.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>

#include <boost/iterator/counting_iterator.hpp>
#include <boost/range/combine.hpp>

namespace opts {
namespace greedy {

TimeDiagram::TimeDiagram(const ScheduleData &schedule, std::size_t proc_num)
    : sched(schedule) {
    proc_array.resize(proc_num);
}

int TimeDiagram::get_time() const {
    BOOST_LOG_NAMED_SCOPE("get_time");
    LOG_TRACE << "calculating time";
    LOG_TRACE << "array size: " << proc_array.size();
    std::vector<std::size_t> proc_times;
    for (const proc_info &a : proc_array) {
        if (!a.empty()) {
            proc_times.push_back(a.back().finish);
        }
    }
    return *std::max_element(proc_times.begin(), proc_times.end());
}

std::size_t TimeDiagram::find_place(greedy::ScheduleData::Task task,
                                     greedy::ScheduleData::Proc proc) {
    BOOST_LOG_NAMED_SCOPE("find_place");
    std::vector<std::size_t> times;
    for (auto it = sched.get_in_edges(task); it.first != it.second;
         ++it.first) {
        auto from = boost::source(*it.first, sched.get_graph());
        int trans_time = sched.tran_times(fast_mapping[from], proc);
        auto &proc_data = proc_array[fast_mapping[from]];
        std::size_t stop_time =
            std::find_if(proc_data.begin(), proc_data.end(),
                         [from](Output_data::PlacedTask &cur) {
                             return cur.task_no == from;
                         })
                ->finish;
        times.push_back(trans_time + stop_time);
    }

    std::size_t first_available_dependencies = 0;

    if (!times.empty()) {
        first_available_dependencies =
            *std::max_element(times.begin(), times.end());
    }

    LOG_TRACE << "first_available_dependencies: "
              << first_available_dependencies;

    auto &proc_to_put = proc_array[proc];
    auto first_to_check =
        std::find_if(proc_to_put.begin(), proc_to_put.end(),
                     [first_available_dependencies](auto n) {
                         return n.finish > first_available_dependencies;
                     });
    auto time_to_exec = sched.get_task_time(proc, task);
    if (first_to_check == proc_to_put.end()) {
        LOG_TRACE << "putting task at the end";
        if (!proc_to_put.empty()) {
            LOG_TRACE << "last task at " << proc_to_put.back().finish
                      << " depen: " << first_available_dependencies;
            return std::max(proc_to_put.back().finish,
                            first_available_dependencies);
        } else {
            return first_available_dependencies;
        }
    }

    auto cur = first_to_check;
    for (; cur != std::prev(proc_to_put.end()); ++cur) {
        if (std::next(cur)->start - cur->finish >= time_to_exec) {
            LOG_TRACE << "found hole!";
            return cur->finish;
        }
    }
    LOG_TRACE << "no hole found :(";
    return cur->finish;
}

TimeDiagram::precalc_info
TimeDiagram::test_add_task(greedy::ScheduleData::Task task,
                            greedy::ScheduleData::Proc proc) {
    BOOST_LOG_NAMED_SCOPE("test_add_task");
    auto expected_start = find_place(task, proc);
    add_task_internal(task, proc, expected_start);
    auto res_CR = calculate_CR();
    auto res_CR2 = calculate_CR2();
    auto res_BF = calculate_BF();
    remove_task(task);
    return {.starting_place = expected_start,
            .resulting_CR = res_CR,
            .resulting_CR2 = res_CR2,
            .resulting_BF = res_BF,
            .expected_finish =
                expected_start + sched.get_task_time(proc, task)};
}

void TimeDiagram::add_task_internal(greedy::ScheduleData::Task task,
                                     greedy::ScheduleData::Proc proc,
                                     std::size_t starting_place) {
    BOOST_LOG_NAMED_SCOPE("add_task_internal");
    Output_data::PlacedTask placed_task{
        task, starting_place, starting_place + sched.get_task_time(proc, task)};
    LOG_TRACE << "adding: " << task << " from time: " << starting_place
              << " to time: " << placed_task.finish << " on process " << proc;
    auto &proc_to_add_to = proc_array[proc];
    auto place_to_add =
        std::find_if(proc_to_add_to.rbegin(), proc_to_add_to.rend(),
                     [placed_task](Output_data::PlacedTask cur) {
                         return cur.finish <= placed_task.start;
                     });
    LOG_TRACE << "inserting after task " << place_to_add->task_no;
    proc_to_add_to.insert(place_to_add.base(), placed_task);
    fast_mapping[task] = proc;
    for (auto it = sched.get_in_edges(task); it.first != it.second;
         ++it.first) {
        auto from = boost::source(*it.first, sched.get_graph());
        if (fast_mapping[from] != proc) {
            ++amount_of_transitions;
            if (!sched.is_direct_connection(fast_mapping[from], proc))
                ++amount_of_indirect_transitions;
        }
    }
}

void TimeDiagram::add_task(greedy::ScheduleData::Task task,
                            greedy::ScheduleData::Proc proc) {
    BOOST_LOG_NAMED_SCOPE("add_task");
    auto x = find_place(task, proc);
    LOG_TRACE << "adding: " << task << " on time: " << x << " on process "
              << proc;
    add_task_internal(task, proc, x);
}

void TimeDiagram::remove_task(greedy::ScheduleData::Task task) {
    BOOST_LOG_NAMED_SCOPE("remove_task");
    auto &proc = proc_array.at(fast_mapping[task]);
    std::erase_if(proc, [task](const Output_data::PlacedTask &t) {
        return t.task_no == task;
    });
    for (auto it = sched.get_in_edges(task); it.first != it.second;
         ++it.first) {
        auto from = boost::source(*it.first, sched.get_graph());
        if (fast_mapping[from] != fast_mapping[task]) {
            --amount_of_transitions;
            if (!sched.is_direct_connection(fast_mapping[from],
                                            fast_mapping[task]))
                --amount_of_indirect_transitions;
        }
    }
    fast_mapping.erase(task);
}

greedy::ScheduleData::Proc TimeDiagram::GC2(greedy::ScheduleData::Task task) {
    std::vector<std::pair<greedy::ScheduleData::Proc, std::size_t>> times;
    std::transform(boost::counting_iterator<std::size_t>(0),
                   boost::counting_iterator<std::size_t>(proc_array.size()),
                   std::back_inserter(times), [&](std::size_t i) {
                       return std::make_pair(
                           i, test_add_task(task, i).expected_finish);
                   });
    greedy::ScheduleData::Proc best_proc =
        std::min_element(
            times.begin(), times.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; })
            ->first;
    return best_proc;
}

greedy::ScheduleData::Proc TimeDiagram::GC2_CR_simple(greedy::ScheduleData::Task task,
                                                   double C1, double C2,
                                                   double C3) {
    std::vector<precalc_info> precalc;
    std::transform(boost::counting_iterator<std::size_t>(0),
                   boost::counting_iterator<std::size_t>(proc_array.size()),
                   std::back_inserter(precalc),
                   [&](std::size_t i) { return test_add_task(task, i); });
    std::size_t max_time =
        std::max_element(precalc.begin(), precalc.end(),
                         [](const precalc_info &a, const precalc_info &b) {
                             return a.expected_finish < b.expected_finish;
                         })
            ->expected_finish;
    std::vector<double> normalized_times;
    std::transform(precalc.begin(), precalc.end(),
                   std::back_inserter(normalized_times),
                   [max_time](const precalc_info &cur_elem) {
                       return cur_elem.expected_finish / (double)max_time;
                   });
    std::vector<std::pair<greedy::ScheduleData::Proc, double>> times;
    auto tied_it = boost::combine(precalc, normalized_times);
    std::transform(
        tied_it.begin(), tied_it.end(),
        boost::counting_iterator<std::size_t>(0), std::back_inserter(times),
        [C1, C2, C3](const auto &vals, std::size_t num) {
            precalc_info inf;
            double time;
            boost::tie(inf, time) = vals;
            return std::make_pair(num, C1 * time + C2 * inf.resulting_CR +
                                           C3 * inf.resulting_CR2);
        });
    return std::min_element(
               times.begin(), times.end(),
               [](const auto &a, const auto &b) { return a.second < b.second; })
        ->first;
}

greedy::ScheduleData::Proc TimeDiagram::GC2_BF_simple(greedy::ScheduleData::Task task,
                                                   double C1, double C2) {
    std::vector<precalc_info> precalc;
    std::transform(boost::counting_iterator<std::size_t>(0),
                   boost::counting_iterator<std::size_t>(proc_array.size()),
                   std::back_inserter(precalc),
                   [&](std::size_t i) { return test_add_task(task, i); });
    std::size_t max_time =
        std::max_element(precalc.begin(), precalc.end(),
                         [](const precalc_info &a, const precalc_info &b) {
                             return a.expected_finish < b.expected_finish;
                         })
            ->expected_finish;
    std::vector<double> normalized_times;
    std::transform(precalc.begin(), precalc.end(),
                   std::back_inserter(normalized_times),
                   [max_time](const precalc_info &cur_elem) {
                       return cur_elem.expected_finish / (double)max_time;
                   });
    std::vector<std::pair<greedy::ScheduleData::Proc, double>> times;
    auto tied_it = boost::combine(precalc, normalized_times);
    std::transform(
        tied_it.begin(), tied_it.end(),
        boost::counting_iterator<std::size_t>(0), std::back_inserter(times),
        [C1, C2](const auto &vals, std::size_t num) {
            precalc_info inf;
            double time;
            boost::tie(inf, time) = vals;
            return std::make_pair(num, C1 * time + C2 * inf.resulting_BF);
        });
    return std::min_element(
               times.begin(), times.end(),
               [](const auto &a, const auto &b) { return a.second < b.second; })
        ->first;
}

greedy::ScheduleData::Proc TimeDiagram::GC2_BF_access(greedy::ScheduleData::Task task,
                                                   double n) {
    BOOST_LOG_NAMED_SCOPE("GC2_BF_access");
    using precalc_table = std::pair<greedy::ScheduleData::Proc, precalc_info>;
    std::vector<precalc_table> task_table;
    std::transform(boost::counting_iterator<std::size_t>(0),
                   boost::counting_iterator<std::size_t>(proc_array.size()),
                   std::back_inserter(task_table), [&](std::size_t i) {
                       return std::make_pair(i, test_add_task(task, i));
                   });

    auto best_bf = std::min_element(
        task_table.begin(), task_table.end(), [](const auto &a, const auto &b) {
            return a.second.resulting_BF < b.second.resulting_BF;
        });

    if (best_bf->second.resulting_BF > 10) {
        return best_bf->first;
    }

    std::sort(task_table.begin(), task_table.end(),
              [](const auto &a, const auto &b) {
                  return a.second.expected_finish < b.second.expected_finish;
              });

    unsigned int task_thresh = std::ceil(proc_array.size() * n);

    std::vector<precalc_table> best_by_gc(task_table.begin(),
                                          task_table.begin() + task_thresh);

    std::sort(best_by_gc.begin(), best_by_gc.end(),
              [](const auto &a, const auto &b) {
                  return a.second.resulting_BF < b.second.resulting_BF;
              });

    task_thresh = std::ceil(best_by_gc.size() * n);
    return std::min_element(
               best_by_gc.begin(), best_by_gc.begin() + task_thresh,
               [](const auto &a, const auto &b) {
                   return a.second.resulting_BF < b.second.resulting_BF;
               })
        ->first;
}

greedy::ScheduleData::Proc TimeDiagram::GC2_CR_access(greedy::ScheduleData::Task task,
                                                   double n) {
    BOOST_LOG_NAMED_SCOPE("GC2_CR_access");
    using precalc_table = std::pair<std::size_t, precalc_info>;
    std::vector<precalc_table> task_table;
    std::transform(boost::counting_iterator<std::size_t>(0),
                   boost::counting_iterator<std::size_t>(proc_array.size()),
                   std::back_inserter(task_table), [&](std::size_t i) {
                       return std::make_pair(i, test_add_task(task, i));
                   });

    std::sort(task_table.begin(), task_table.end(),
              [](const auto &a, const auto &b) {
                  return a.second.expected_finish < b.second.expected_finish;
              });

    unsigned int task_thresh = std::ceil(proc_array.size() * n);
    LOG_TRACE << "task_thresh: " << task_thresh;
    std::vector<precalc_table> best_by_gc(task_table.begin(),
                                          task_table.begin() + task_thresh);

    std::sort(best_by_gc.begin(), best_by_gc.end(),
              [](const auto &a, const auto &b) {
                  return a.second.resulting_CR < b.second.resulting_CR;
              });

    task_thresh = std::ceil(best_by_gc.size() * n);
    LOG_TRACE << "task_thresh: " << task_thresh;

    std::vector<precalc_table> best_by_cr(best_by_gc.begin(),
                                          best_by_gc.begin() + task_thresh);

    std::sort(best_by_cr.begin(), best_by_cr.end(),
              [](const auto &a, const auto &b) {
                  return a.second.resulting_CR2 < b.second.resulting_CR2;
              });

    task_thresh = std::ceil(best_by_cr.size() * n);
    LOG_TRACE << "task_thresh: " << task_thresh;

    return std::min_element(
               best_by_cr.begin(), best_by_cr.begin() + task_thresh,
               [](const auto &a, const auto &b) {
                   return a.second.resulting_CR < b.second.resulting_CR;
               })
        ->first;
}

double TimeDiagram::calculate_BF() const {
    BOOST_LOG_NAMED_SCOPE("calculate_BF");
    auto max_tasks =
        std::max_element(proc_array.begin(), proc_array.end(),
                         [](const proc_info &a, const proc_info &b) {
                             return a.size() < b.size();
                         })
            ->size();
    size_t amount_of_tasks =
        std::accumulate(proc_array.begin(), proc_array.end(), 0,
                        [](std::size_t a, const proc_info &b) {
                            return std::move(a) + b.size();
                        });
    double BF =
        100 * ((max_tasks * proc_array.size() / (double)amount_of_tasks) - 1);
    return std::ceil(BF);
}

double TimeDiagram::calculate_CR() const {
    BOOST_LOG_NAMED_SCOPE("calculate_CR");
    LOG_TRACE << amount_of_transitions << " "
              << boost::num_edges(sched.get_graph());
    return amount_of_transitions / (double)boost::num_edges(sched.get_graph());
}

double TimeDiagram::calculate_CR2() const {
    BOOST_LOG_NAMED_SCOPE("calculate_CR2");
    return amount_of_indirect_transitions /
           (double)boost::num_edges(sched.get_graph());
}

void TimeDiagram::print_schedule() const {
    BOOST_LOG_NAMED_SCOPE("print_schedule");
    LOG_TRACE << "printing schedule";
    std::stringstream ss;
    for (auto proc : proc_array) {
        for (auto task : proc) {
            ss << '(' << task.task_no << ")[" << task.start << ','
               << task.finish << "]; ";
        }
        ss << std::endl;
    }
    LOG_TRACE << ss.str();
}

double TimeDiagram::calculate_downtime() const {
    BOOST_LOG_NAMED_SCOPE("calculate_downtime");
    double relative_downtime = 0;
    for (auto proc : proc_array) {
        size_t proc_downtime = 0;
        size_t prev_time = 0;
        size_t task_time = 0;
        if (!proc.empty()) {
            for (auto task : proc) {
                proc_downtime += task.start - prev_time;
                task_time += task.finish - task.start;
                prev_time = task.finish;
            }
            relative_downtime += proc_downtime / (double)task_time;
        }
    }
    return relative_downtime / proc_array.size();
}

std::vector<TimeDiagram::proc_info> &TimeDiagram::get_schedule() {
    return proc_array;
}

Output_data TimeDiagram::extract_data(const opts::greedy_config &conf) const {
    Output_data res;
    res.BF = calculate_BF();
    res.CR = calculate_CR();
    res.CR2 = calculate_CR2();
    res.criteria = conf.criteria;
    res.nodes = std::accumulate(
        proc_array.begin(), proc_array.end(), 0,
        [](std::size_t cum, const std::list<Output_data::PlacedTask> &cur) {
            return cum + cur.size();
        });
    res.time = get_time();
    res.proc_array = std::vector<proc_info>(this->proc_array);
    return res;
}

} // namespace greedy
} // namespace opts