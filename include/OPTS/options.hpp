#ifndef OPTIONS_HPP_
#define OPTIONS_HPP_

#include <string>
#include <utility>

#include <boost/describe/enum.hpp>

namespace opts {

/**
 * @brief Criterion used in schedule construction.
 *
 */
enum class extra_criteria {
    NO, ///< no additional constraints
    CR, ///< constraint on interprocessor communication
    BF  ///< constraint on load balance between processors
};

BOOST_DESCRIBE_ENUM(extra_criteria, NO, CR, BF)

/**
 * @brief Type of input data.
 *
 */
enum class input_class {
    class_general, ///< processor matrix is full and homogenous, task time
                   ///< can be different for different processors
    class_1, ///< processor matrix is full and homogenous, task execution time
             ///< is same for all processors
    class_2 ///< processor matrix is either full and not homogenous, or not full
            ///< and homogenous, task time is same for all processors
};

BOOST_DESCRIBE_ENUM(input_class, class_general, class_1, class_2)

/**
 * @brief Structure that contains configuration parameters common for all
 * algorithms.
 *
 */
struct base_config {
    base_config() = default;
    base_config(const base_config &other) = default;

    /**
     * @brief Criterion used in schedule construction.
     *
     */
    extra_criteria criteria;

    /**
     * @brief Max value for BF criterion.
     *
     * Used only if criteria is set to BF. Otherwise, this parameter is ignored.
     *
     */
    double BF_bound;

    /**
     * @brief Max value for CR criterion.
     *
     * Used only if criteria is set to CR. Otherwise, this parameter is ignored.
     *
     */
    double CR_bound;

    /**
     * @brief Max value for CR2 criterion.
     *
     * Used only if criteria is set to CR. Otherwise, this parameter is ignored.
     *
     */
    double CR2_bound;

    /**
     * @brief Type of input data.
     *
     */
    input_class _class;
};

/**
 * @brief GC2 scheme of choosing processor for a task
 *
 */
enum class GC2_scheme {
    access, ///< tolrance scheme of sort -> cut -> sort -> take best
    simple  ///< choose processor according to weighted sum
};

BOOST_DESCRIBE_ENUM(GC2_scheme, access, simple)

/**
 * @brief Structure that contains configuration parameters common for all
 * algorithms and specific for greedy algorithm.
 *
 */
struct greedy_config : public base_config {
    greedy_config(const base_config &base) : base_config(base) {}

    GC2_scheme scheme; ///< GC2 scheme chosen
    double threshold;  ///< Threshold for tolerance scheme
    /**
     * @deprecated
     * @brief CR control toggle
     *
     */
    bool cr_con;
};

/**
 * @brief Parse parameters needed to run greedy algorithm from configuration
 * file.
 *
 * @param filename Name of the configuration file
 * @retval greedy_config Structure that contains parameters needed to run greedy
 * algorithm
 */
greedy_config parse_greedy_config(std::string filename);

} // namespace opts

#endif