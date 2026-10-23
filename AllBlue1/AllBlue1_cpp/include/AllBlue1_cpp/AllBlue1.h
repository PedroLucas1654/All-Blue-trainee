#pragma once
// Porta serial POSIX (Linux/macOS). No Windows use WSL ou adapte com a WinAPI.
#include <string>

namespace allblue1 {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud = 9600);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Lê uma linha terminada em '\n'. Retorna false em timeout (~1 s) ou erro.
    bool read_line(std::string& line);

private:
    int fd_ = -1;
    std::string buffer_;
};

}  // namespace allblue1
