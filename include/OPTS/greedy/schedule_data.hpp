#ifndef SCHEDULE_HPP
#define SCHEDULE_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <set>
#include <vector>

namespace opts {
namespace greedy {

/**
 * @brief Specialized greedy algorithm input data class.
 *
 * Contains a task graph, \f$C\f$ and \f$D\f$ matrices, which are task execution
 * time and processor communication matrices.
 */
class ScheduleData {
  public:
    /**
     * @brief Internal type used to represent data in a node
     */
    struct VertexData {
        /**
         * @deprecated
         * @brief Shortest path from a fictive root to this node.
         *
         */
        int shortest_path_length;

        /**
         * @brief Inverse deadline of a task. If deadline is not yet set, the
         * deadline is 1.
         *
         */
        int deadline = 1;

        /**
         * @deprecated
         * @brief Indicates if this node is fictive
         *
         */
        bool is_fictive = false;

        /**
         * @brief Indicates if this node is present in the graph.
         *
         */
        bool is_existent = true;
    };

    /**
     * @brief Internal type used to represent data in an edge
     *
     */
    struct EdgeData {
        /**
         * @deprecated
         * @brief Maximum execution time of the parent node. Used in Dijksta's
         * algorithm.
         *
         */
        int min_time = 0;
    };

    /**
     * @brief Type used to represent a processor
     */
    using Proc = std::size_t;

    /**
     * @brief Edge type used to represent a task dependency. Only contains a
     * dependency.
     */
    using Edge = std::pair<Proc, Proc>;

    /**
     * @brief Boost::Graph graph type shortened
     *
     */
    using Graph =
        boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                              VertexData, EdgeData>;

    /**
     * @brief Type representing a node descriptor.
     *
     */
    using Task = Graph::vertex_descriptor;

    /**
     * @brief Number of tasks.
     *
     */
    std::size_t task_num = 0;

    /**
     * @brief Number of processors.
     *
     */
    std::size_t proc_num = 0;

    /**
     * @brief Number of edges.
     *
     */
    std::size_t edges = 0;

    boost::numeric::ublas::matrix<std::size_t>
        task_times; ///< \f$ C \f$ matrix (size: proc_num x task_num)
    boost::numeric::ublas::matrix<int>
        tran_times; ///< \f$ D \f$ matrix (size: proc_num x proc_num)
    boost::numeric::ublas::matrix<int>
        long_transmition; ///< A matrix that shows if processors are connected
                          ///< directly or indirectly.

    /**
     * @brief A specialized graph with all necessary data to run greedy
     * algorithm.
     *
     */
    Graph graph;

    /**
     * @brief Construct a new ScheduleData object
     *
     * @param edge_vec Vector of all edges in a graph
     * @param task_times `task_time` matrix (\f$ C \f$)
     * @param tran_times `tran_time` matrix (\f$ D \f$)
     */
    ScheduleData(std::vector<Edge> &edge_vec,
             boost::numeric::ublas::matrix<std::size_t> task_times,
             boost::numeric::ublas::matrix<int> tran_times);

    /**
     * @brief Assign one input data object to another.
     *
     * @param other The other input data object.
     * @retval ScheduleData& Other input data object after the assgnment.
     */
    ScheduleData &operator=(const ScheduleData &other) = default;

    /**
     * @brief Print graph to stdout.
     *
     */
    void print_graph() const;

    /**
     * @deprecated
     * @brief Get the whole underlying graph.
     *
     * @retval Graph& Internal graph
     */
    Graph &get_graph();

    /**
     * @deprecated
     * @brief Get the whole underlying graph.
     *
     * @retval "const Graph&" Internal graph
     */
    const Graph &get_graph() const;

    /**
     * @brief Remove all nodes marked fictive from graph.
     *
     * This removes the node completely in order not to get errors with node
     * counting and dependecies.
     *
     */
    void hard_remove_fictive_vertices();

    /**
     * @brief Remove a node from the graph
     *
     * This only sets the node as non-existent, so it only affects setting up
     * critical paths and node progression.
     *
     * @param task Task node to remove.
     *
     */
    void remove_vertex(const Task &task);

    /**
     * @brief Get all nodes that have no parent nodes, which means that they
     * are ready to be added to the schedule.
     *
     * @retval std::set<Task> All ready-to-be-added tasks
     */
    std::set<Task> get_top_vertices();

    /**
     * @deprecated
     * @brief Create a fictive node.
     *
     * This method creates a fictive node that has the duration of 0 and has an
     * edge to each node from std::set D
     *
     * @param D Nodes that have to be child nodes of a fictive node
     */
    void create_fictive_node(std::set<Task> &D);

    /**
     * @deprecated
     * @brief Calculate critical paths
     *
     *  1. For each edge set its property to minimum execution time of the
     * source task on any processor
     *  2. Run Dijkstra's algorithm on the graph to calculate critical paths
     *
     * @todo change `std::numeric_limits<int>::max()` to something more
     * reasonable
     * @todo change calculating minimum to matrix-aware code
     */
    void set_up_critical_paths();

    /**
     * @brief Calculate GC1
     *
     * Greedy criteria 1 chooses the task from \f$ D \f$ that has the highest
     * `out_degree`
     *
     * @param D Set of tasks that are ready to be added to the schedule
     * @retval ScheduleData::Task Chosen next task.
     */
    Task GC1(std::set<Task> &D);

    /**
     * @deprecated
     * @brief Calculate modified GC1 for CR control
     *
     * Greedy criteria 1 chooses the task from \f$ D \f$ that has the lowest
     * `out_degree`
     *
     * @param D Set of tasks that are ready to be added to the schedule
     * @retval ScheduleData::Task Chosen next task.
     */
    Task GC1_for_CR_con(std::set<Task> &D);

    /**
     * @brief This method progresses the top nodes set if a task is added.
     *
     * This method:
     *  1. Removes the just added task from \f$ D \f$
     *  2. For each child node of \f$ D \f$ checks whether its children are
     * ready to be added to the schedule and, if they are, adds them to \f$ D
     * \f$
     *
     * @param D Old set of available nodes
     * @param task_to_progress Task that has just been removed from the graph.
     * @retval std::set<Task> Updated set of available nodes.
     */
    std::set<Task> progress_top_vertices(std::set<Task> &D,
                                         Task task_to_progress);

    /**
     * @brief Get a vector of task times on the first processor.
     *
     * This method only makes sense when the processors are
     * homogeneous.
     *
     * @retval std::vector<std::size_t> A vector of times in the first
     * processor. Can be used as weights.
     */
    std::vector<std::size_t> get_proc_times_vector();

    /**
     * @brief Get the task time on a given processor
     *
     * @param proc Processor number.
     * @param task Task number.
     * @retval int Task time.
     */
    int get_task_time(size_t proc, size_t task) const;

    /**
     * @brief Get all edges that have the specified task as destination.
     *
     * @param task Task to get edges for
     * @retval
     * std::pair<boost::graph_traits<...>> `begin()` and `end()` iterators on
     * incoming edges
     */
    std::pair<boost::graph_traits<Graph>::in_edge_iterator,
              boost::graph_traits<Graph>::in_edge_iterator>
    get_in_edges(const Task &task) const;

    /**
     * @brief Get the number of child tasks
     *
     * @param task Parent task
     * @return size_t The number of outgoing edges of a node.
     */
    std::size_t get_out_degree(const Task &task) const;

    /**
     * @brief Get the number of parent tasks
     *
     * @param task Child task
     * @return size_t The number of incoming edges into a node.
     */
    std::size_t get_in_degree(const Task &task) const;

    /**
     * @brief Initialize communication delay matrices. Fill in connections
     * through third processors.
     *
     * @param tran Communication delay matrix `D`
     */
    void init_transmition_matrices(boost::numeric::ublas::matrix<int> tran);

    /**
     * @brief Check if the connection between two processors is direct
     *
     * @param proc1 Source processor
     * @param proc2 Destination processor
     * @retval true if directly connected
     * @retval false if connected through third processor
     */
    bool is_direct_connection(const Proc &proc1, const Proc &proc2) const;
};

} // namespace greedy
} // namespace opts

#endif // SCHEDULE_HPP