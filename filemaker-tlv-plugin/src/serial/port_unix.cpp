#include "serial_port.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <system_error>
#include <string.h>

class SerialPort::Impl {
    int fd = -1;
    Config config;

public:
    Impl(const Config& cfg) : config(cfg) {}

    void open() {
        fd = ::open(config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            throw std::system_error(
                errno,
                std::system_category(),
                "Failed to open serial port");
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            close();
            throw std::system_error(
                errno,
                std::system_category(),
                "Failed to get serial port attributes");
        }

        // Set baud rate
        speed_t speed;
        switch(config.baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // Configure port settings
        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8 data bits
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_oflag &= ~OPOST; // Raw output

        // Set timeout (convert ms to deciseconds)
        tty.c_cc[VTIME] = config.timeout_ms / 100;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close();
            throw std::system_error(
                errno,
                std::system_category(),
                "Failed to set serial port attributes");
        }
    }

    void close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    bool is_open() const { return fd >= 0; }

    size_t write(const std::vector<uint8_t>& data) {
        ssize_t bytesWritten = ::write(fd, data.data(), data.size());
        if (bytesWritten < 0) {
            throw std::system_error(
                errno,
                std::system_category(),
                "Serial port write failed");
        }
        return static_cast<size_t>(bytesWritten);
    }

    std::vector<uint8_t> read() {
        std::vector<uint8_t> buffer(4096);
        ssize_t bytesRead = ::read(fd, buffer.data(), buffer.size());
        
        if (bytesRead < 0) {
            throw std::system_error(
                errno,
                std::system_category(),
                "Serial port read failed");
        }
        
        buffer.resize(bytesRead);
        return buffer;
    }

    ~Impl() { close(); }
};

// SerialPort wrapper methods (same as Windows implementation)
SerialPort::SerialPort(const Config& config) : pimpl(std::make_unique<Impl>(config)) {}
SerialPort::~SerialPort() = default;
void SerialPort::open() { pimpl->open(); }
void SerialPort::close() { pimpl->close(); }
bool SerialPort::is_open() const { return pimpl->is_open(); }
size_t SerialPort::write(const std::vector<uint8_t>& data) { return pimpl->write(data); }
std::vector<uint8_t> SerialPort::read() { return pimpl->read(); }