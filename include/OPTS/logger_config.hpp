#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <boost/log/expressions.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup.hpp>

/**
 * @brief A macro to log with "debug" severity
 *
 */
#define LOG_DEBUG BOOST_LOG_SEV(opts::log, boost::log::trivial::debug)
/**
 * @brief A macro to log with "info" severity
 *
 */
#define LOG_INFO BOOST_LOG_SEV(opts::log, boost::log::trivial::info)
/**
 * @brief A macro to log with "warning" severity
 *
 */
#define LOG_WARNING BOOST_LOG_SEV(opts::log, boost::log::trivial::warning)
/**
 * @brief A macro to log with "error" severity
 *
 */
#define LOG_ERROR BOOST_LOG_SEV(opts::log, boost::log::trivial::error)

/**
 * @brief A macro to log with "trace" severity
 *
 */
#define LOG_TRACE BOOST_LOG_SEV(opts::log, boost::log::trivial::trace)
namespace opts {

/**
 * @brief Static logger object
 * 
 */
extern boost::log::sources::severity_logger<boost::log::trivial::severity_level>
    log;

/**
 * @brief Initialize the logger. Takes in namespace parameters
 * `verbose` and `debug`
 *
 * @param level logging level
 */
void init(boost::log::trivial::severity_level level);
} // namespace opts

#endif // LOGGER_HPP