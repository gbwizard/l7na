#include <iostream>
#include <thread>
#include <chrono>

#include <boost/tokenizer.hpp>

#include "l7na/drives.h"

int main(int argc, char* argv[]) {
    Drives::Control control("");

    std::cout << "Please, specify your commands here:" << std::endl;

    std::string cmd_str;
    typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
    boost::char_separator<char> sep(" ");

    struct Command {
        int32_t az_pos = 0;
        int32_t az_vel = 0;
        int32_t el_pos = 0;
        int32_t el_vel = 0;
        bool az_idle = false;
        bool el_idle = false;

        void clear() {
            az_pos = 0;
            az_vel = 0;
            el_pos = 0;
            el_vel = 0;
            az_idle = false;
            el_idle = false;
        }
    };


    while (true) {
        std::vector<std::string> cmd_vec;
        Command cmd;

        std::cout << "> ";
        std::getline(std::cin, cmd_str);

        if (cmd_str == "q") {
            break;
        }

        Tokenizer tokens(cmd_str, sep);
        for (Tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
            cmd_vec.push_back(*tok_iter);
        }

        for (uint32_t i = 0; i < cmd_vec.size();) {
            if (cmd_vec[i] == "a") {
                if (++i >= cmd_vec.size()) {
                    std::cerr << "Invalid input for command 'a'" << std::endl;
                    break;
                }

                if (cmd_vec[i] == "v") {
                    if (i + 1 >= cmd_vec.size()) {
                        std::cerr << "Invalid input for command 'a v'" << std::endl;
                        break;
                    }

                    cmd.az_vel = std::atoi(cmd_vec[++i].c_str());
                } else if (cmd_vec[i] == "p") {
                    if (i + 1 >= cmd_vec.size()) {
                        std::cerr << "Invalid input for command 'a p'" << std::endl;
                        break;
                    }

                    cmd.az_pos = std::atoi(cmd_vec[++i].c_str());
                } else if (cmd_vec[i] == "i") {
                    cmd.az_idle = true;
                }
            } else if (cmd_vec[i] == "e") {
                if (++i >= cmd_vec.size()) {
                    std::cerr << "Invalid input for command 'e'" << std::endl;
                    break;
                }

                if (cmd_vec[i] == "v") {
                    if (i + 1 >= cmd_vec.size()) {
                        std::cerr << "Invalid input for command 'e v'" << std::endl;
                        break;
                    }

                    cmd.el_vel = std::atoi(cmd_vec[++i].c_str());
                } else if (cmd_vec[i] == "p") {
                    if (i + 1 >= cmd_vec.size()) {
                        std::cerr << "Invalid input for command 'e p'" << std::endl;
                        break;
                    }

                    cmd.el_pos = std::atoi(cmd_vec[++i].c_str());
                } else if (cmd_vec[i] == "i") {
                    cmd.el_idle = true;
                }
            } else {
                std::cerr << "Invalid input" << std::endl;
                break;
            }

            ++i;
        }

        std::cout << "Azimuth pos: " << cmd.az_pos << " vel: " << cmd.az_vel << " idle: " << cmd.az_idle
                  << ". Elevation pos: " << cmd.el_pos << " vel: " << cmd.el_vel << " idle: " << cmd.el_idle << std::endl;
    }

	return 0;
}
