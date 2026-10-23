
#include <AllBlue1_cpp/AllBlue1.h>    

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace allblue1 {

namespace {
speed_t to_speed(int baud) {
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}
}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud) {
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    cfmakeraw(&tty);                 // modo bruto: 8N1, sem eco
    cfsetispeed(&tty, to_speed(baud));
    cfsetospeed(&tty, to_speed(baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;              // read() não bloqueia indefinidamente
    tty.c_cc[VTIME] = 10;            // timeout de 1,0 s (décimos de segundo)

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    buffer_.clear();
}

bool SerialPort::read_line(std::string& line) {
    if (fd_ < 0) return false;
    for (;;) {
        auto pos = buffer_.find('\n');
        if (pos != std::string::npos) {
            line = buffer_.substr(0, pos);
            buffer_.erase(0, pos + 1);
            return true;
        }
        char tmp[256];
        ssize_t n = ::read(fd_, tmp, sizeof(tmp));
        if (n <= 0) return false;
        buffer_.append(tmp, static_cast<std::size_t>(n));
    }
}

}  // namespace allblue1
