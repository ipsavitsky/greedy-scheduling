#include "OPTS/greedy/schedule.hpp"
#include "OPTS/logger_config.hpp"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <iostream>

namespace opts {
namespace greedy {

Schedule::Schedule(std::vector<Edge> &edge_vec,
                   boost::numeric::ublas::matrix<std::size_t> task_times,
                   boost::numeric::ublas::matrix<int> tran_times) {
    BOOST_LOG_NAMED_SCOPE("Schedule");
    this->task_num = task_times.size2();
    this->proc_num = tran_times.size1();
    graph = Graph(edge_vec.begin(), edge_vec.end(), task_num);
    edges = boost::num_edges(graph);
    this->task_times = task_times;
    init_transmition_matrices(tran_times);
}

void Schedule::print_graph() const {
    BOOST_LOG_NAMED_SCOPE("print_graph");
    LOG_TRACE << "printing graph";
    auto fictiveness = get(&VertexData::is_fictive, graph);
    auto sh_path = get(&VertexData::shortest_path_length, graph);
    for (Schedule::Task curr_task = 0; curr_task < task_num; ++curr_task) {
        LOG_TRACE << std::boolalpha << "from: " << curr_task
                  << " (f:" << fictiveness[curr_task]
                  << "; sh_path:" << sh_path[curr_task] << ')'
                  << std::noboolalpha << std::endl;
        for (auto edges = boost::out_edges(curr_task, graph);
             edges.first != edges.second; ++edges.first) {
            Schedule::Task child_task = boost::target(*edges.first, graph);
            LOG_TRACE << "to: " << child_task << std::endl;
        }
    }
}

std::set<Schedule::Task> Schedule::get_top_vertices() {
    BOOST_LOG_NAMED_SCOPE("get_top_vertices");
    std::set<Schedule::Task> top_vertices;
    for (Schedule::Task task = 0; task < task_num; ++task) {
        bool has_real_parents = false;
        for (auto edges = boost::in_edges(task, graph);
             edges.first != edges.second; ++edges.first) {
            auto src = boost::source(*edges.first, graph);
            if (graph[src].is_existent == true) {
                has_real_parents = true;
                break;
            }
        }
        if (!has_real_parents && graph[task].is_existent == true)
            top_vertices.insert(task);
    }
    return top_vertices;
}

void Schedule::create_fictive_node(std::set<Schedule::Task> &D) {
    BOOST_LOG_NAMED_SCOPE("create_fictive_node");
    auto new_vert = add_vertex({0, 0, true, true}, graph);
    std::for_each(D.begin(), D.end(), [&](Schedule::Task task) {
        LOG_TRACE << "Adding edge from " << new_vert << " to " << task;
        add_edge(new_vert, task, {0}, graph);
    });
    ++task_num;
}

void Schedule::set_up_critical_paths() {
    BOOST_LOG_NAMED_SCOPE("set_up_critical_paths");
    Schedule::Task fictive_node = -1;

    for (Schedule::Task curr_task = 0; curr_task < task_num; ++curr_task) {
        auto out_edges = boost::out_edges(curr_task, graph);
        size_t min_time = std::numeric_limits<size_t>::max();
        if (graph[curr_task].is_fictive) {
            min_time = 0;
            fictive_node = curr_task;
        } else {
            for (std::size_t i = 0; i < proc_num; ++i) {
                if (task_times(i, curr_task) < min_time) {
                    min_time = task_times(i, curr_task);
                }
            }
        }
        LOG_TRACE << "min time on " << curr_task << " is " << min_time;
        std::for_each(out_edges.first, out_edges.second,
                      [&](auto edge) { graph[edge].min_time = min_time; });
    }
    LOG_DEBUG << "fictive node is " << fictive_node;
    boost::dijkstra_shortest_paths(
        graph, fictive_node,
        distance_map(get(&VertexData::shortest_path_length, graph))
            .weight_map(get(&EdgeData::min_time, graph)));
}

void Schedule::hard_remove_fictive_vertices() {
    BOOST_LOG_NAMED_SCOPE("hard_remove_fictive_vertices");
    for (Schedule::Task task = 0; task < task_num; ++task) {
        if (graph[task].is_fictive) {
            LOG_DEBUG << "removing fictive node " << task;
            boost::clear_vertex(task, graph);
            boost::remove_vertex(task, graph);
            --task_num;
        }
    }
}

void Schedule::remove_vertex(const Schedule::Task &task) {
    BOOST_LOG_NAMED_SCOPE("remove_vertex");
    graph[task].is_existent = false;
}

Schedule::Task Schedule::GC1(std::set<Schedule::Task> &D) {
    BOOST_LOG_NAMED_SCOPE("GC1");
    return *std::max_element(D.begin(), D.end(),
                             [&](Schedule::Task task1, Schedule::Task task2) {
                                 return boost::out_degree(task1, graph) <
                                        boost::out_degree(task2, graph);
                             });
}

Schedule::Task Schedule::GC1_for_CR_con(std::set<Schedule::Task> &D) {
    BOOST_LOG_NAMED_SCOPE("GC1_for_CR_con");
    return *std::max_element(D.begin(), D.end(),
                             [&](Schedule::Task task1, Schedule::Task task2) {
                                 return boost::in_degree(task1, graph) <
                                        boost::in_degree(task2, graph);
                             });
}

Schedule::Graph &Schedule::get_graph() {
    BOOST_LOG_NAMED_SCOPE("get_graph");
    return graph;
}

const Schedule::Graph &Schedule::get_graph() const {
    BOOST_LOG_NAMED_SCOPE("get_graph");
    return graph;
}

std::vector<std::size_t> Schedule::get_proc_times_vector() {
    boost::numeric::ublas::matrix_row<decltype(task_times)> first_row(
        task_times, 0);
    std::vector<std::size_t> ret(first_row.begin(), first_row.end());
    return ret;
}

std::set<Schedule::Task>
Schedule::progress_top_vertices(std::set<Schedule::Task> &D,
                                Schedule::Task task_to_progress) {
    D.erase(task_to_progress);
    for (auto [from_it, to_it] = boost::out_edges(task_to_progress, graph);
         from_it != to_it; ++from_it) {
        auto task = boost::target(*from_it, graph);
        bool has_real_parents = false;
        for (auto edges = boost::in_edges(task, graph);
             edges.first != edges.second; ++edges.first) {
            auto src = boost::source(*edges.first, graph);
            if (graph[src].is_existent == true) {
                has_real_parents = true;
                break;
            }
        }
        if (!has_real_parents && graph[task].is_existent == true)
            D.insert(task);
    }
    return D;
}

int Schedule::get_task_time(size_t proc, size_t task) const {
    if (task_times.size1() == 1) {
        return task_times(0, 0);
    }
    return task_times(proc, task);
}

bool Schedule::is_direct_connection(const Proc &proc1,
                                    const Proc &proc2) const {
    BOOST_LOG_NAMED_SCOPE("is_direct_connection");
    return long_transmition(proc1, proc2) == -1;
}

void Schedule::init_transmition_matrices(
    boost::numeric::ublas::matrix<int> tran) {
    BOOST_LOG_NAMED_SCOPE("init_transmition_matrices");
    tran_times = tran;
    long_transmition.resize(tran.size1(), tran.size2(), false);
    for (std::size_t i = 0; i < tran.size1(); ++i) {
        for (std::size_t j = 0; j < tran.size2(); ++j) {
            if (tran(i, j) == -1) {
                for (std::size_t k = 0; k < tran.size1(); ++k) {
                    if (tran(i, k) != -1 && tran(k, j) != -1 &&
                        (tran_times(i, j) == -1 ||
                         tran(i, k) + tran(k, j) < tran_times(i, j))) {
                        tran_times(i, j) = tran(i, k) + tran(k, j);
                        long_transmition(i, j) = k;
                    }
                }
            } else {
                long_transmition(i, j) = -1;
            }
        }
    }
}

std::pair<boost::graph_traits<Schedule::Graph>::in_edge_iterator,
          boost::graph_traits<Schedule::Graph>::in_edge_iterator>
Schedule::get_in_edges(const Task &task) const {
    BOOST_LOG_NAMED_SCOPE("get_in_edges");
    return boost::in_edges(task, graph);
}

std::size_t Schedule::get_out_degree(const Task &task) const {
    BOOST_LOG_NAMED_SCOPE("get_out_degree");
    return boost::out_degree(task, graph);
}

std::size_t Schedule::get_in_degree(const Task &task) const {
    BOOST_LOG_NAMED_SCOPE("get_in_degree");
    return boost::in_degree(task, graph);
}

} // namespace greedy
} // namespace opts