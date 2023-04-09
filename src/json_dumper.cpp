#include <boost/describe/enum_to_string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

#include "OPTS/json_dumper.hpp"
#include "nlohmann/json.hpp"

namespace opts {

void dump_to_json(const std::string &filename, const Output_data &out_data,
                  int64_t exec_time) {
    nlohmann::json root;
    nlohmann::json procs_node;
    auto diagram = out_data.proc_array;
    for (std::size_t i = 0; i < diagram.size(); ++i) {
        nlohmann::json proc_node;
        for (auto &task : diagram[i]) {
            nlohmann::json task_node;
            task_node["task_no"] = task.task_no;
            task_node["task_start"] = task.start;
            task_node["task_dur"] = task.finish - task.start;
            proc_node.push_back(task_node);
        }
        procs_node[boost::lexical_cast<std::string, std::size_t>(i)] =
            proc_node;
    }
    root["procs"] = procs_node;
    root["time"] = out_data.time;
    root["start_time"] = out_data.start_time;
    root["start_times"] = out_data.start_times;
    root["criteria"] = boost::describe::enum_to_string(out_data.criteria, "");
    root["CR"] = out_data.CR;
    root["BF"] = out_data.BF;
    root["CR2"] = out_data.CR2;
    root["nodes"] = std::accumulate(
        diagram.begin(), diagram.end(), 0,
        [](std::size_t sum, const std::list<Output_data::PlacedTask> &cur) {
            return sum + cur.size();
        });
    // root["nodes"] = sched.get_graph().m_vertices.size();
    root["algo_time"] = exec_time;
    std::ofstream outfile;
    outfile.open(filename);
    outfile << root;
    outfile.close();
}
} // namespace opts