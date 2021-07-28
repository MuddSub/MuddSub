// #include "drivers/DepthSensorPublisher.hh"
#include <iostream>
#include <fstream>

int main() {
    std::string str;
    std::fstream f;
    f.open("/dev/ttyACM0");
    while (f >> str) {
        std::cout << str;
    }
}