#include <iostream>
#include <chrono>
#include <fstream>
#include <atomic>

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
    bool disable;
    bool idle;
    bool enable;
    bool stop;
    bool reset;
    bool update_params;
    bool gopoint;
    bool govel;
    Drives::AxisParams params;

    Command()
        : axis(Drives::AZIMUTH_AXIS)
        , pos(0.0)
        , vel(0.0)
        , disable(false)
        , idle(false)
        , enable(false)
        , stop(false)
        , reset(false)
        , update_params(false)
        , gopoint(false)
        , govel(false)
        , params()
    {}

    void clear() {
        axis = Drives::AZIMUTH_AXIS;
        pos = 0.0;
        vel = 0.0;
        disable = false;
        idle = false;
        enable = false;
        stop = false;
        reset = false;
        update_params = false;
        gopoint = false;
        govel = false;
        params.clear();
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

            result.govel = true;
            result.vel = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'a p'" << std::endl;
                return false;
            }

            result.gopoint = true;
            result.pos = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "u") {
            if (cmd_vec.size() < 4 || cmd_vec.size() % 2 != 0) {
                std::cerr << "Invalid input for command 'a u'" << std::endl;
                return false;
            }

            uint16_t idx;
            int64_t val;
            size_t arg_idx = 2;
            while (arg_idx < cmd_vec.size()) {
                try {
                    idx = std::stoi(cmd_vec[arg_idx++], NULL, 16);
                    val = std::stoi(cmd_vec[arg_idx++]);
                } catch (const std::exception& ex) {
                    std::cerr << "Can't convert input for 'a u':" << ex.what() << std::endl;
                    return false;
                }
                result.params.push_back({idx, val});
            }

            result.update_params = true;
        } else if (cmd_vec[1] == "i") {
            result.idle = true;
        } else if (cmd_vec[1] == "r") {
            result.reset = true;
        } else if (cmd_vec[1] == "e") {
            result.enable = true;
        } else if (cmd_vec[1] == "d") {
            result.disable = true;
        } else if (cmd_vec[1] == "s") {
            result.stop = true;
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

            result.govel = true;
            result.vel = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'e p'" << std::endl;
                return false;
            }

            result.gopoint = true;
            result.pos = std::atof(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "u") {
            if (cmd_vec.size() < 4 || cmd_vec.size() % 2 != 0) {
                std::cerr << "Invalid input for command 'e u'" << std::endl;
                return false;
            }

            uint16_t idx;
            int64_t val;
            size_t arg_idx = 2;
            while (arg_idx < cmd_vec.size()) {
                try {
                    idx = std::stoi(cmd_vec[arg_idx++], NULL, 16);
                    val = std::stoi(cmd_vec[arg_idx++]);
                } catch (const std::exception& ex) {
                    std::cerr << "Can't convert input for 'e u':" << ex.what() << std::endl;
                    return false;
                }
                result.params.push_back({idx, val});
            }

            result.update_params = true;
        } else if (cmd_vec[1] == "i") {
            result.idle = true;
        } else if (cmd_vec[1] == "r") {
            result.reset = true;
        } else if (cmd_vec[1] == "e") {
            result.enable = true;
        } else if (cmd_vec[1] == "d") {
            result.disable = true;
        } else if (cmd_vec[1] == "s") {
            result.stop = true;
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
    os << boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time());
    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
         os << "|" << axis << "|" << status.axes[axis].state << "|" << std::hex << "0x" << status.axes[axis].statusword << "|" << status.axes[axis].ctrlword
                  << std::dec << "|" << status.axes[axis].mode
                  << "|" << status.axes[axis].cur_pos_deg  << "|" << status.axes[axis].tgt_pos_deg << "|" << status.axes[axis].dmd_pos_deg
                  << "|" << status.axes[axis].cur_vel_deg  << "|" << status.axes[axis].tgt_vel_deg << "|" << status.axes[axis].dmd_vel_deg
                  << "|" << status.axes[axis].cur_torq << "|" << status.axes[axis].cur_temperature0;
    }
    os << std::endl;
    ++i;
}

void print_status_cerr(const Drives::SystemStatus& status, const Drives::CycleTimeInfo& timing_info) {
    std::cerr << "System > state: " << Drives::GetSystemStateName(status.state) << " dcsync: " << status.dcsync
              << std::endl << "\t"
              << "apptime                   : " << status.apptime << std::hex << " = 0x" << status.apptime << std::dec
              << std::endl << "\t"
              << "reftime                   : " << status.reftime << std::hex << " = 0x" << status.reftime << std::dec
              << std::endl << "\t"
              << "period_ms (min/max)       : " << std::setw(4) << (timing_info.period_min_ns / 1e6) << ":" << std::setw(4) << (timing_info.period_max_ns / 1e6)
              << std::endl << "\t"
              << "latency_ms (min/max)      : " << std::setw(4) << (timing_info.latency_min_ns / 1e6) << ":" << std::setw(4) << (timing_info.latency_max_ns / 1e6)
              << std::endl << "\t"
              << "exec_ms (min/max)         : " << std::setw(4) << (timing_info.exec_min_ns / 1e6) << ":" << std::setw(4) << (timing_info.exec_max_ns / 1e6)
              << std::endl;

    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cerr << "Axis " << axis << " > state: " << Drives::GetAxisStateName(status.axes[axis].state) << " statusword: " << std::hex << "0x" << status.axes[axis].statusword << " ctrlword: 0x" << status.axes[axis].ctrlword
                  << std::dec << " mode: " << status.axes[axis].mode
                  << std::endl << "\t"
                  << " cur/dmd/tgt_pos_deg     = " << status.axes[axis].cur_pos_deg << "/" << status.axes[axis].dmd_pos_deg << "/" << status.axes[axis].tgt_pos_deg
                  << std::endl << "\t"
                  << " cur/dmd_vel_deg         = " << status.axes[axis].cur_vel_deg << "/" << status.axes[axis].dmd_vel_deg
                  << std::endl << "\t"
                  << " abs/cur/dmd/tgt_pos = " << status.axes[axis].cur_pos_abs << "/"  << status.axes[axis].cur_pos << "/" << status.axes[axis].dmd_pos << "/" << status.axes[axis].tgt_pos
                  << std::endl << "\t"
                  << " cur/dmd_vel_raw         = " << status.axes[axis].cur_vel << "/" << status.axes[axis].dmd_vel
                  << std::endl << "\t"
                  << " cur_trq: " << status.axes[axis].cur_torq << " cur_tmp: " << status.axes[axis].cur_temperature0
                  << std::endl << "\t"
                  << " params_mode: " << (status.axes[axis].params_mode == Drives::PARAMS_MODE_AUTOMATIC ? "auto" : "manual")
                  << " move_mode: " << status.axes[axis].move_mode
                  << std::endl;
    }
}

void print_info(const Drives::SystemInfo& info) {
    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cerr << "Axis " << axis << " > dev_name: " << info.axes[axis].dev_name << " encoder_pulses_per_rev: " << info.axes[axis].encoder_pulses_per_rev
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
    std::cerr << kLevelIndent << "a|e v <vel>       - set (a)zimuth or (e)levation drive to 'scan' mode with <vel> velocity [pulses/sec]" << std::endl;
    std::cerr << kLevelIndent << "a|e p <pos>       - set (a)zimuth or (e)levation drive to 'point' mode with <pos> position [pulses]" << std::endl;
    std::cerr << kLevelIndent << "a|e r             - reset (a)zimuth or (e)levation drive fault state" << std::endl;
    std::cerr << kLevelIndent << "a|e e             - make (a)zimuth or (e)levation drive go to (e)nabled state" << std::endl;
    std::cerr << kLevelIndent << "a|e i             - make (a)zimuth or (e)levation drive go to (i)dle state" << std::endl;
    std::cerr << kLevelIndent << "a|e s             - make (a)zimuth or (e)levation drive go to (s)top state" << std::endl;
    std::cerr << kLevelIndent << "a|e d             - make (a)zimuth or (e)levation drive go to (d)isabled state" << std::endl;
    std::cerr << kLevelIndent << "a|e u <idx> <val> - set (a)zimuth or (e)levation drive parameter" << std::endl;
}

struct StatReader {
    StatReader(const std::atomic<Drives::SystemStatus>& status, const fs::path& outfilepath, uint32_t lograte_us)
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
            print_status(status_.load(std::memory_order_acquire), ofs);
            boost::this_thread::sleep_for(boost::chrono::microseconds(lograte_us_));
        }
        ofs.close();
    }

    volatile bool stop_;
    const std::atomic<Drives::SystemStatus>& status_;
    const fs::path outfilepath_;
    const uint32_t lograte_us_;
};

int main(int argc, char* argv[]) {
    blog::trivial::severity_level loglevel;
    fs::path cfg_file_path, log_file_path;
    uint32_t log_rate_us;
    int32_t pos_abs_offset_azim, pos_abs_offset_elev;

    po::options_description options("options");
    options.add_options()
        ("help,h", "display this message")
        ("azim_off", po::value<decltype(pos_abs_offset_azim)>(&pos_abs_offset_azim)->default_value(0), "Offset for absolute azimuth position [pulses]")
        ("elev_off", po::value<decltype(pos_abs_offset_elev)>(&pos_abs_offset_elev)->default_value(0), "Offset for elevation azimuth position [pulses]")
        ("loglevel,l", po::value<decltype(loglevel)>(&loglevel)->default_value(boost::log::trivial::warning), "global loglevel (trace, debug, info, warning, error or fatal)")
        ("config,c", po::value<decltype(cfg_file_path)>(&cfg_file_path)->required(), "path to config file")
        ("logfile,f", po::value<decltype(log_file_path)>(&log_file_path), "path to output log file. If specified engine real time data will be written to this file")
        ("lograte,r", po::value<decltype(log_rate_us)>(&log_rate_us), "period in microseconds (us) between samples written to log file. Ignored without 'logfile' option")
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
    control.SetPosAbsPulseOffset(Drives::AZIMUTH_AXIS, pos_abs_offset_azim);
    control.SetPosAbsPulseOffset(Drives::ELEVATION_AXIS, pos_abs_offset_elev);

    const std::atomic<Drives::SystemStatus>& sys_status = control.GetStatusRef();
    const std::atomic<Drives::CycleTimeInfo>& timing_info = control.GetCycleTimeInfoRef();

    std::cerr << "Waiting for system initialization..." << std::endl;
    while (! control.IsOperational()) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }
    std::cerr << "System is ready" << std::endl;
    std::cerr << "Please, specify your commands here:" << std::endl;

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
            print_status_cerr(sys_status.load(std::memory_order_acquire), timing_info.load(std::memory_order_acquire));
            control.ResetCycleTimeInfo();
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
            control.Idle(cmd.axis);
        } else if (cmd.stop) {
            control.Stop(cmd.axis);
        } else if (cmd.enable) {
            control.Enable(cmd.axis);
        } else if (cmd.disable) {
            control.Disable(cmd.axis);
        } else if (cmd.reset) {
            control.ResetFault(cmd.axis);
        } else if (cmd.update_params) {
            control.SetAxisParams(cmd.axis, cmd.params);
        } else if (cmd.gopoint) {
            control.RunToPoint(cmd.axis, cmd.pos);
        } else if (cmd.govel) {
            control.RunAtVelocity(cmd.axis, cmd.vel);
        }

        std::cerr << "Command axis: " << cmd.axis
                  << " pos: " << cmd.pos
                  << " vel: " << cmd.vel
                  << " gopoint: " << cmd.gopoint
                  << " govel: " << cmd.govel
                  << " idle: " << cmd.idle
                  << " stop: " << cmd.stop
                  << " disable: " << cmd.disable
                  << " enable: " << cmd.enable
                  << " idle: " << cmd.idle
                  << " reset: " << cmd.reset
                  << " update_params: " << cmd.update_params
                  << std::endl;
    }

    statreader.stop_ = true;
    statthread.join();

    return 0;
}
