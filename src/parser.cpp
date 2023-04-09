#include "OPTS/huawei_parser.hpp"
#include "OPTS/logger_config.hpp"
#include "OPTS/options.hpp"

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <filesystem>
#include <fstream>

namespace opts {

greedy::ScheduleData input_schedule_regular(std::string path,
                                        opts::input_class inp_class) {
    std::filesystem::path p(path);
    if (!std::filesystem::is_regular_file(p)) {
        throw std::runtime_error(
            "Not a regular file (opening a schedule file)");
    }
    LOG_DEBUG << "path: " << p;

    std::ifstream input(p);

    int task_num, proc_num, edge_num;
    input >> task_num >> proc_num >> edge_num;
    boost::numeric::ublas::matrix<std::size_t> task_time;
    if (inp_class == opts::input_class::class_2) {
        std::size_t time;
        input >> time;
        task_time =
            boost::numeric::ublas::scalar_matrix<std::size_t>(1, task_num) *
            time;
    } else {
        task_time =
            boost::numeric::ublas::matrix<std::size_t>(proc_num, task_num); // C
        for (std::size_t i = 0; i < task_time.size1(); ++i) {
            for (std::size_t j = 0; j < task_time.size2(); ++j) {
                input >> task_time(i, j);
            }
        }
    }
    boost::numeric::ublas::matrix<int> tran_time(proc_num, proc_num); // D
    for (std::size_t i = 0; i < tran_time.size1(); ++i) {
        for (std::size_t j = 0; j < tran_time.size2(); ++j) {
            input >> tran_time(i, j);
        }
    }
    std::vector<greedy::ScheduleData::Edge> edges;
    for (int i = 0; i < edge_num; ++i) {
        int a, b;
        input >> a >> b;
        edges.push_back({a, b});
    }
    return greedy::ScheduleData(edges, task_time, tran_time);
}

} // namespace opts