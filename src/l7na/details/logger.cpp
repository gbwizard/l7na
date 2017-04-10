#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/console.hpp>

#include "l7na/logger.h"

#ifndef L7NA_DISABLE_LOGS

namespace common {

namespace logger = boost::log;

std::string DefaultLogFormat() {
    static const std::string kLogFormat = "%LineID% (%ProcessID%:%ThreadID%) [%Severity%] : %Message%";
    return kLogFormat;
}

void InitLogger(const logger::trivial::severity_level& level, const std::string& format, const std::string& filename)
{
    logger::register_simple_formatter_factory< logger::trivial::severity_level, char >("Severity");

    logger::add_console_log(
        std::cout,
        logger::keywords::format = format,
        logger::keywords::filter = logger::trivial::severity >= level
    );

    if (! filename.empty()) {
        logger::add_file_log(
            logger::keywords::file_name = filename,
            logger::keywords::format = format,
            logger::keywords::filter = logger::trivial::severity >= level,
            logger::keywords::open_mode = (std::ios::out | std::ios::app)
        );
    }

    logger::add_common_attributes();
}

} // namespaces

#endif
