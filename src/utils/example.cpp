#include <iostream>
#include <chrono>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/memory_order.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/once.hpp>

#include "l7na/drives.h"
#include "l7na/configfile.h"
#include "l7na/logger.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace blog = boost::log;

struct Command {
    Drives::Axis axis;
    double pos;
    double vel;
    bool idle;

    Command()
        : axis(Drives::AZIMUTH_AXIS)
        , pos(0.0)
        , vel(0.0)
        , idle(false)
    {}

    void clear() {
        axis = Drives::AZIMUTH_AXIS;
        pos = 0.0;
        vel = 0.0;
        idle = false;
    }
};

bool parse_args(const std::string& cmd_str, Command& result) {

    typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
    boost::char_separator<char> sep(" ");

    Tokenizer tokens(cmd_str, sep);
    std::vector<std::string> cmd_vec;
    for (Tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
        cmd_vec.push_back(*tok_iter);
    }

    if (cmd_vec[0] == "a") {
        result.axis = Drives::AZIMUTH_AXIS;
        if (cmd_vec.size() < 2) {
            std::cerr << "Invalid input for command 'a'" << std::endl;
            return false;
        }

        if (cmd_vec[1] == "v") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'a v'" << std::endl;
                return false;
            }

            result.vel = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'a p'" << std::endl;
                return false;
            }

            result.pos = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "i") {
            result.idle = true;
        }
    } else if (cmd_vec[0] == "e") {
        result.axis = Drives::ELEVATION_AXIS;
        if (cmd_vec.size() < 2) {
            std::cerr << "Invalid input for command 'e'" << std::endl;
            return false;
        }

        if (cmd_vec[1] == "v") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'e v'" << std::endl;
                return false;
            }

            result.vel = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'e p'" << std::endl;
                return false;
            }

            result.pos = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "i") {
            result.idle = true;
        }
    } else {
        std::cerr << "Invalid input" << std::endl;
        return false;
    }


    return true;
}

void print_status(const Drives::SystemStatus& status, std::ostream& os) {
    static bool header_printed = false;
    if (! header_printed) {
        os << "DateTime|AxisA";
        os << "|StateA|StatusWordA|ControlWordA|ModeA|CurPosA|TgtPosA|DmdPosA";
        os << "|CurVelA|TgtVelA|DmdVelA|CurTrqA|CurTempA";
        os << "|AxisE";
        os << "|StateE|StatusWordE|ControlWordE|ModeE|CurPosE|TgtPosE|DmdPosE";
        os << "|CurVelE|TgtVelE|DmdVelE|CurTrqE|CurTempE";
        os << std::endl;
        header_printed = true;
    }
    static uint64_t i = 0;
    // os << i;
    os << "|" << boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time());
    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
         os << "|" << axis << "|" << status.axes[axis].state << "|" << std::hex << "0x" << status.axes[axis].statusword << "|" << status.axes[axis].ctrlword
                  << std::dec << "|" << status.axes[axis].mode
                  << "|" << status.axes[axis].cur_pos  << "|" << status.axes[axis].tgt_pos << "|" << status.axes[axis].dmd_pos
                  << "|" << status.axes[axis].cur_vel  << "|" << status.axes[axis].tgt_vel << "|" << status.axes[axis].dmd_vel
                  << "|" << status.axes[axis].cur_torq << "|" << status.axes[axis].cur_temperature;
    }
    os << std::endl;
    ++i;
}

void print_status_cerr(const Drives::SystemStatus& status) {
    std::cerr << "System > state: " << status.state << " dcsync: " << status.dcsync << std::endl;
//    std::cerr << "    prevapptime               : " << status.prev_apptime << std::hex << " = 0x" << status.prev_apptime << std::dec << std::endl;
    std::cerr << "    apptime                   : " << status.apptime << std::hex << " = 0x" << status.apptime << std::dec << std::endl;
    std::cerr << "    reftime                   : " << status.reftime << std::hex << " = 0x" << status.reftime << std::dec << std::endl;
//    std::cerr << "    cycle_latency_ns          : " << status.latency_ns << std::endl;
//    std::cerr << "    cycle_latency_min_ns      : " << status.latency_min_ns << std::endl;
//    std::cerr << "    cycle_latency_max_ns      : " << status.latency_max_ns << std::endl;
//    std::cerr << "    cycle_period_ns           : " << status.period_ns << std::endl;
//    std::cerr << "    cycle_period_min_ns       : " << status.period_min_ns << std::endl;
//    std::cerr << "    cycle_period_max_ns       : " << status.period_max_ns << std::endl;
//    std::cerr << "    cycle_exec_ns             : " << status.exec_ns << std::endl;
//    std::cerr << "    cycle_exec_min_ns         : " << status.exec_min_ns << std::endl;
//    std::cerr << "    cycle_exec_max_ns         : " << status.exec_max_ns << std::endl;

    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cerr << "Axis " << axis << " > state: " << status.axes[axis].state << " statusword: " << std::hex << "0x" << status.axes[axis].statusword << " ctrlword: 0x" << status.axes[axis].ctrlword
                  << std::dec << " mode: " << status.axes[axis].mode
                  << std::endl << "\t"
                  << " cur_pos_deg: " << status.axes[axis].cur_pos_deg << " tgt_pos_deg: " << status.axes[axis].tgt_pos_deg << " dmd_pos_deg: " << status.axes[axis].dmd_pos_deg
                  << " cur_vel_deg: " << status.axes[axis].cur_vel_deg << " tgt_vel_deg: " << status.axes[axis].tgt_vel_deg << " dmd_vel_deg: " << status.axes[axis].dmd_vel_deg
                  << std::endl << "\t"
                  << " cur_pos_raw: " << status.axes[axis].cur_pos << " tgt_pos_raw: " << status.axes[axis].tgt_pos << " dmd_pos_raw: " << status.axes[axis].dmd_pos
                  << " cur_vel_raw: " << status.axes[axis].cur_vel << " tgt_vel_raw: " << status.axes[axis].tgt_vel << " dmd_vel_raw: " << status.axes[axis].dmd_vel
                  << std::endl << "\t"
                  << " cur_trq: " << status.axes[axis].cur_torq << " cur_tmp: " << status.axes[axis].cur_temperature << std::endl;
    }
}

void print_info(const Drives::SystemInfo& info) {
    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cerr << "Axis " << axis << " > dev_name: " << info.axes[axis].dev_name << " encoder_resolution: " << info.axes[axis].encoder_resolution
                  << " hw_version: " << info.axes[axis].hw_version << " sw_version: " << info.axes[axis].sw_version << std::endl;
    }
}

void print_available_commands() {
    const char kLevelIndent[] = "    ";
    std::cerr << "Available commands:" << std::endl;
    std::cerr << kLevelIndent << "h, help           - print this message" << std::endl;
    std::cerr << kLevelIndent << "q                 - quit" << std::endl;
    std::cerr << kLevelIndent << "s                 - print system status" << std::endl;
    std::cerr << kLevelIndent << "i                 - print system info" << std::endl;
    std::cerr << kLevelIndent << "a|e v <vely>      - set (a)zimuth or (e)levation drive to 'scan' mode with <vel> velocity [pulses/sec]" << std::endl;
    std::cerr << kLevelIndent << "a|e p <pos>       - set (a)zimuth or (e)levation drive to 'point' mode with <pos> position [pulses]" << std::endl;
}

struct StatReader {
    StatReader(const  boost::atomic<Drives::SystemStatus>& status, const fs::path& outfilepath, uint32_t lograte_us)
        : stop_(false)
        , status_(status)
        , outfilepath_(outfilepath)
        , lograte_us_(lograte_us)
    {}

    void CycleRead() {
        if (outfilepath_.empty()) {
            return;
        }
        std::ofstream ofs(outfilepath_.string(), std::ofstream::out | std::ofstream::app);
        if (! ofs.good()) {
            std::cerr << "Failed to open log file" << std::endl;
            return;
        }
        while (! stop_) {
            print_status(status_.load(boost::memory_order_acquire), ofs);
            boost::this_thread::sleep_for(boost::chrono::microseconds(lograte_us_));
        }
        ofs.close();
    }

    volatile bool stop_;
    const boost::atomic<Drives::SystemStatus>& status_;
    const fs::path outfilepath_;
    const uint32_t lograte_us_;
};

int main(int argc, char* argv[]) {
    blog::trivial::severity_level loglevel;
    fs::path cfg_file_path, log_file_path;
    uint32_t log_rate_us;

    po::options_description options("options");
    options.add_options()
        ("help,h", "display this message")
        ("loglevel,l", po::value<boost::log::trivial::severity_level>(&loglevel)->default_value(boost::log::trivial::warning), "global loglevel (trace, debug, info, warning, error or fatal)")
        ("config,c", po::value<fs::path>(&cfg_file_path)->required(), "path to config file")
        ("logfile,f", po::value<fs::path>(&log_file_path), "path to output log file. If specified engine real time data will be written to this file")
        ("lograte,r", po::value<uint32_t>(&log_rate_us), "period in microseconds (us) between samples written to log file. Ignored without 'logfile' option")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, options), vm);
        po::notify(vm);
    } catch(const po::error& ex) {
        std::cerr << "Failed to parse command line options: " << ex.what() << std::endl;
        std::cerr << options << std::endl;
        return EXIT_FAILURE;
    }

    if (vm.count("help")) {
        std::cerr << options << std::endl;
        return EXIT_FAILURE;
    }

    const char* kLogFormat = "%LineID% %TimeStamp% (%ProcessID%:%ThreadID%) [%Severity%] : %Message%";
    common::InitLogger(loglevel, kLogFormat);

    Config::Storage config;
    try {
        config.ReadFile(cfg_file_path.string());
    } catch(const Config::Exception& ex) {
        std::cerr << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    Drives::Control control(config);
    control.SetPositionPulseOffset(Drives::AZIMUTH_AXIS, 0);
    control.SetPositionPulseOffset(Drives::ELEVATION_AXIS, 85000);

    std::cerr << "Please, specify your commands here:" << std::endl;

    const boost::atomic<Drives::SystemStatus>& sys_status = control.GetStatusRef();

    StatReader statreader(sys_status, log_file_path, log_rate_us);
    boost::thread statthread(boost::bind(&StatReader::CycleRead, &statreader));

    std::string cmd_str;
    const Drives::SystemInfo& sys_info = control.GetSystemInfo();
    while (true) {
        std::cerr << "> ";
        std::getline(std::cin, cmd_str);

        if (cmd_str == "q") {
            break;
        } else if (cmd_str == "h" || cmd_str == "help") {
            print_available_commands();
            continue;
        } else if (cmd_str == "s") {
            print_status_cerr(sys_status.load(boost::memory_order_acquire));
            continue;
        } else if (cmd_str == "i") {
            print_info(sys_info);
            continue;
        } else if (cmd_str.empty()) {
            continue;
        }

        Command cmd;
        if (! parse_args(cmd_str, cmd)) {
            continue;
        }

        // Команда будет передана, только если флаг соответствующий двигателю будет установлен.
        if (cmd.idle) {
            control.SetModeIdle(cmd.axis);
        } else {
            control.SetModeRun(cmd.axis, cmd.pos, cmd.vel);
        }

        std::cerr << "Command axis: " << cmd.axis << " pos: " << cmd.pos << " vel: " << cmd.vel << " idle: " << cmd.idle << std::endl;
    }

    statreader.stop_ = true;
    statthread.join();

    return 0;
}
