#ifndef GRAPH_PART_HPP_
#define GRAPH_PART_HPP_

#include "OPTS/greedy/schedule_data.hpp"
#include "OPTS/greedy/time_diagram.hpp"
#include "metis.h"

namespace opts {
/**
 * @brief Compressed sparse row representation of the graph
 *
 * It is used in METIS partitioning.
 *
 */
struct CSR {
    std::vector<idx_t> xadj;  ///< Adjacency indices component of the CSR
    std::vector<idx_t> adjcy; ///< Adjacency component of the CSR
};

/**
 * @deprecated
 *
 * @brief Load partitioning from an external file.
 *
 * @param filename Input file name
 * @param num_proc Number of processors in a partition
 * @retval std::vector<std::size_t> Partition vector.
 */
std::vector<std::size_t> get_partition(std::string filename,
                                       std::size_t num_proc);

/**
 * @brief Convert an greedy::ScheduleData::Graph to a CSR understandable by
 * partitioning algorithms
 *
 * @param graph Graph to be converted
 * @retval CSR Generated CSR.
 */
CSR adjcy2CSR(const greedy::ScheduleData::Graph &graph);

/**
 * @brief Graph partitioning algorithm.
 *
 * This function utilises METIS to make a weighted graph partition.
 *
 * @param csr CSR representation of a graph.
 * @param num_parts Number of parts for the graph to be partitioned into
 * @param ufactor Unbalance factor of METIS algorithm
 * @param first_row A vector of processor weights. Usually these are task
 * execution times. If unweighted partition is needed, this is a vector of 1
 * @retval std::vector<std::size_t> partition vector
 * @retval uint64 `edgecut` value of the generated partition. This can be used
 * to calculate CR
 */
std::tuple<std::vector<std::size_t>, uint64_t>
part_graph(CSR &csr, std::size_t num_parts, std::uint64_t ufactor,
           std::vector<std::size_t> first_row);

/**
 * @brief Local partition optimization algorithm
 *
 * For heterogeneous processors this routine optimizes partition by execution
 * time.
 *
 * @param partition Unoptimized partition
 * @param data Input data object
 * @param conf Algorithm configuration object
 * @retval std::vector<std::size_t> Balanced partition
 */
std::vector<std::size_t>
local_partition_optimization(const std::vector<std::size_t> &partition,
                             const greedy::ScheduleData &data,
                             const opts::base_config &conf);

} // namespace opts
#endif