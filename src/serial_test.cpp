#include <iostream>
#include <memory>
#include <vector>
#include "serial_driver/serial_port.hpp"

int main() {
    std::cout << "Starting serial port test..." << std::endl;

    auto owned_ctx = std::make_unique<drivers::common::IoContext>(1);

    drivers::serial_driver::SerialPortConfig port_config(
        57600, // Baud rate
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE
    );

    std::unique_ptr<drivers::serial_driver::SerialPort> ser_port;

    try {
        std::cout << "Attempting to create SerialPort object..." << std::endl;
        ser_port = std::make_unique<drivers::serial_driver::SerialPort>(*owned_ctx, "/dev/ttyMotorLeft", port_config);

        std::cout << "Attempting to open port..." << std::endl;
        ser_port->open();

    } catch (const std::exception & e) {
        std::cerr << "[FATAL] An exception occurred: " << e.what() << std::endl;
        return -1;
    }

    if (ser_port && ser_port->is_open()) {
        std::cout << "[SUCCESS] Port /dev/ttyMotorLeft opened successfully." << std::endl;
        ser_port->close();
    } else {
        std::cerr << "[FAILURE] Port did not open, but no exception was thrown." << std::endl;
        return -1;
    }

    return 0;
}