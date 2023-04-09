#include "OPTS/logger_config.hpp"

namespace opts {

boost::log::sources::severity_logger<boost::log::trivial::severity_level> log;

void init(boost::log::trivial::severity_level level) {
    boost::log::add_common_attributes();
    opts::log.add_attribute("Scope", boost::log::attributes::named_scope());

    boost::log::core::get()->set_filter(boost::log::trivial::severity >= level);
    auto fmtTimeStamp =
        boost::log::expressions::format_date_time<boost::posix_time::ptime>(
            "TimeStamp", "%Y-%m-%d %H:%M:%S");
    auto fmtSeverity =
        boost::log::expressions::attr<boost::log::trivial::severity_level>(
            "Severity");

    auto fmtContext = boost::log::expressions::format_named_scope(
        "Scope", boost::log::keywords::format = "%n");

    boost::log::formatter logFmt =
        boost::log::expressions::format("[%1%] [%2%] [%3%] %4%") %
        fmtTimeStamp % fmtSeverity % fmtContext %
        boost::log::expressions::smessage;

    auto console_sink = boost::log::add_console_log(std::clog);
    auto file_sink = boost::log::add_file_log(
        boost::log::keywords::target = "logs/",
        boost::log::keywords::file_name = "greedy_%N.log",
        boost::log::keywords::auto_flush = true);
    console_sink->set_formatter(logFmt);
    file_sink->set_formatter(logFmt);
}

} // namespace opts