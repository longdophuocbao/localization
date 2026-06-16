#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class SerialPort {
public:
    SerialPort() : fd_(-1) {}
    ~SerialPort() { close_port(); }

    bool open_port(const std::string& port, int baudrate) {
        close_port();
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) {
            return false;
        }

        // Clear O_NDELAY flag to make read blocking/non-blocking via termios
        fcntl(fd_, F_SETFL, 0);

        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            close_port();
            return false;
        }

        // Set baud rate
        speed_t speed;
        switch (baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200; break;
        }
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // Local line, enable receiver
        options.c_cflag |= (CLOCAL | CREAD);

        // Raw input (no echoing, no canonical processing)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        // Raw output
        options.c_oflag &= ~OPOST;

        // Disable software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Fully non-blocking read settings: return immediately with whatever is available
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            close_port();
            return false;
        }

        return true;
    }

    void close_port() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const {
        return fd_ >= 0;
    }

    ssize_t read_data(uint8_t* buffer, size_t size) {
        if (fd_ < 0) return -1;
        return read(fd_, buffer, size);
    }

    ssize_t write_data(const uint8_t* buffer, size_t size) {
        if (fd_ < 0) return -1;
        return write(fd_, buffer, size);
    }

private:
    int fd_;
};

#endif // SERIAL_PORT_HPP
