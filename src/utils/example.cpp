#include <iostream>
#include <thread>
#include <chrono>

#include <boost/tokenizer.hpp>

#include "l7na/drives.h"
#include "l7na/details/logger.h"

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

    std::cout << "Axis 0 > state: " << status.elevation.state << " statusword: " << std::hex << "0x" << status.elevation.statusword << "0x" << status.elevation.ctrlword
              << std::dec << " mode: " << status.elevation.mode
              << " cur_pos: " << status.elevation.cur_pos << " tgt_pos: " << status.elevation.tgt_pos << " dmd_pos: " << status.elevation.dmd_pos
              << " cur_vel: " << status.elevation.cur_vel << " tgt_vel: " << status.elevation.tgt_vel << " dmd_vel: " << status.elevation.dmd_vel
              << " cur_torq: " << status.elevation.cur_torq << std::endl;

    std::cout << "Axis 1 > state: "  << status.azimuth.state << " statusword: " << std::hex << "0x" << status.azimuth.statusword << "0x" << status.azimuth.ctrlword
              << std::dec << " mode: " << status.azimuth.mode
              << " cur_pos: " << status.azimuth.cur_pos << " tgt_pos: " << status.azimuth.tgt_pos << " dmd_pos: " << status.azimuth.dmd_pos
              << " cur_vel: " << status.azimuth.cur_vel << " tgt_vel: " << status.azimuth.tgt_vel << " dmd_vel: " << status.azimuth.dmd_vel
              << " cur_torq: " << status.azimuth.cur_torq << std::endl;
}

int main(int argc, char* argv[]) {
    common::InitLogger(boost::log::trivial::error, "%LineID% %TimeStamp% (%ProcessID%:%ThreadID%) [%Severity%] : <%Channel%> %Message%");

    Drives::Control control("");

    std::cout << "Please, specify your commands here:" << std::endl;

    std::string cmd_str;
    const std::atomic<Drives::SystemStatus>& sys_status = control.GetStatus();
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, cmd_str);

        if (cmd_str == "q") {
            break;
        } else if (cmd_str == "s") {
            print_status(sys_status.load(std::memory_order_relaxed));
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
