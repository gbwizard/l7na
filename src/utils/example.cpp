#include <iostream>
#include <thread>
#include <chrono>

#include <boost/tokenizer.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

#include "l7na/drives.h"
#include "l7na/details/logger.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace blog = boost::log;

struct Command {
    Drives::Axis axis = Drives::AZIMUTH_AXIS;
    int32_t pos = 0;
    int32_t vel = 0;
    bool idle = false;

    void clear() {
        axis = Drives::AZIMUTH_AXIS;
        pos = 0;
        vel = 0;
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

            result.vel = std::atoi(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'a p'" << std::endl;
                return false;
            }

            result.pos = std::atoi(cmd_vec[2].c_str());
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

            result.vel = std::atoi(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "p") {
            if (cmd_vec.size() < 3) {
                std::cerr << "Invalid input for command 'e p'" << std::endl;
                return false;
            }

            result.pos = std::atoi(cmd_vec[2].c_str());
        } else if (cmd_vec[1] == "i") {
            result.idle = true;
        }
    } else {
        std::cerr << "Invalid input" << std::endl;
        return false;
    }


    return true;
}

void print_status(const Drives::SystemStatus& status) {
    std::cout << "System > state: " << status.state << std::endl;

    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cout << "Axis " << axis << " > state: " << status.axes[axis].state << " statusword: " << std::hex << "0x" << status.axes[axis].statusword << " ctrlword: 0x" << status.axes[axis].ctrlword
                  << std::dec << " mode: " << status.axes[axis].mode
                  << " cur_pos: " << status.axes[axis].cur_pos << " tgt_pos: " << status.axes[axis].tgt_pos << " dmd_pos: " << status.axes[axis].dmd_pos
                  << " cur_vel: " << status.axes[axis].cur_vel << " tgt_vel: " << status.axes[axis].tgt_vel << " dmd_vel: " << status.axes[axis].dmd_vel
                  << " cur_torq: " << status.axes[axis].cur_torq << " cur_temp: " << status.axes[axis].cur_temperature << std::endl;
    }
}

void print_info(const Drives::SystemInfo& info) {
    for (int32_t axis = Drives::AXIS_MIN; axis < Drives::AXIS_COUNT; ++axis) {
        std::cout << "Axis " << axis << " > dev_name: " << info.axes[axis].dev_name << " encoder_resolution: " << info.axes[axis].encoder_resolution
                  << " hw_version: " << info.axes[axis].hw_version << " sw_version: " << info.axes[axis].sw_version << std::endl;
    }
}

void print_available_commands() {
    const char kLevelIndent[] = "    ";
    std::cout << "Available commands:" << std::endl;
    std::cout << kLevelIndent << "h, help           - print this message" << std::endl;
    std::cout << kLevelIndent << "q                 - quit" << std::endl;
    std::cout << kLevelIndent << "s                 - print system status" << std::endl;
    std::cout << kLevelIndent << "i                 - print system info" << std::endl;
    std::cout << kLevelIndent << "a|e v <vely>      - set (a)zimuth or (e)levation drive to 'scan' mode with <vel> velocity [pulses/sec]" << std::endl;
    std::cout << kLevelIndent << "a|e p <pos>       - set (a)zimuth or (e)levation drive to 'point' mode with <pos> position [pulses]" << std::endl;
}

int main(int argc, char* argv[]) {
    blog::trivial::severity_level loglevel;
    fs::path cfg_file_path;

    po::options_description options("options");
    options.add_options()
        ("help,h", "display this message")
        ("loglevel,l", po::value<boost::log::trivial::severity_level>(&loglevel)->default_value(boost::log::trivial::warning), "global loglevel (trace, debug, info, warning, error or fatal)")
        ("config,c", po::value<fs::path>(&cfg_file_path), "path to config file. If not specified default values for all parameters are used")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, options), vm);
        po::notify(vm);
    } catch(const po::error& ex) {
        std::cerr << "Failed to parse command line options: " << ex.what() << std::endl;
        std::cout << options << std::endl;
        return EXIT_FAILURE;
    }

    if (vm.count("help")) {
        std::cerr << options << std::endl;
        return EXIT_FAILURE;
    }

    const char* kLogFormat = "%LineID% %TimeStamp% (%ProcessID%:%ThreadID%) [%Severity%] : %Message%";
    common::InitLogger(loglevel, kLogFormat);

    Drives::Control control(cfg_file_path.string());

    std::cout << "Please, specify your commands here:" << std::endl;

    std::string cmd_str;
    const std::atomic<Drives::SystemStatus>& sys_status = control.GetStatus();
    const Drives::SystemInfo& sys_info = control.GetSystemInfo();
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, cmd_str);

        if (cmd_str == "q") {
            break;
        } else if (cmd_str == "h" || cmd_str == "help") {
            print_available_commands();
            continue;
        } else if (cmd_str == "s") {
            print_status(sys_status.load(std::memory_order_acquire));
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

        std::cout << "Command axis: " << cmd.axis << " pos: " << cmd.pos << " vel: " << cmd.vel << " idle: " << cmd.idle << std::endl;
    }

	return 0;
}
