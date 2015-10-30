#include <thread>
#include <chrono>
#include "l7na/drives.h"

int main(int argc, char* argv[]) {
    Drives::Control control("");
    std::this_thread::sleep_for(std::chrono::seconds(20));
	return 0;
}
