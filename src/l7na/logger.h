#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include <boost/log/trivial.hpp>

#define LOG(messages, level)\
    {\
        try {\
            BOOST_LOG_TRIVIAL(level) << messages;\
        } catch(const std::exception& e) {\
            std::cerr << "[Error logging] " << e.what() << '\t' << messages << std::endl; \
        }\
    }

#define LOG_TRACE(messages) LOG(messages, trace)
#define LOG_DEBUG(messages) LOG(messages, debug)
#define LOG_INFO(messages) LOG(messages, info)
#define LOG_WARN(messages) LOG(messages, warning)
#define LOG_ERROR(messages) LOG(messages, error)
#define LOG_FATAL(messages) LOG(messages, fatal)

namespace common {

std::string DefaultLogFormat();

void InitLogger(const boost::log::trivial::severity_level& level, const std::string& format, const std::string& filename = std::string());

//! \brief Оператор ввода severity_level. Благодаря ему в командной строке вместо цифр нужно явно задавать уровень логирования.
template<typename TChar, typename TTraits>
inline std::basic_istream<TChar, TTraits>& operator>>(std::basic_istream<TChar, TTraits>& stream, boost::log::trivial::severity_level& lvl)
{
    static const std::unordered_map<std::string, boost::log::trivial::severity_level> kStr2severity = {
        {"trace",   boost::log::trivial::trace},
        {"debug",   boost::log::trivial::debug},
        {"info",    boost::log::trivial::info},
        {"warning", boost::log::trivial::warning},
        {"error",   boost::log::trivial::error},
        {"fatal",   boost::log::trivial::fatal}
    };

    std::string s;
    stream >> s;

    auto it = kStr2severity.find(s);
    if (it == kStr2severity.end()) {
        stream.setstate(std::ios::failbit);
    } else {
        lvl = it->second;
    }

    return stream;
}

} // namespaces



