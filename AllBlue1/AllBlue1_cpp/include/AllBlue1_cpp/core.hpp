#pragma once
// Núcleo: parser NMEA (sem dependência de hardware, fácil de testar)
#include <string>
#include <vector>

namespace allblue1 {

struct GpsData {
    bool has_fix = false;
    double latitude = 0.0;     // graus decimais (S negativo)
    double longitude = 0.0;    // graus decimais (W negativo)
    double altitude_m = 0.0;
    double speed_kmh = 0.0;
    double course_deg = 0.0;
    double hdop = 0.0;
    int satellites = 0;
    std::string utc_time;      // HH:MM:SS
    std::string utc_date;      // DD/MM/YYYY
};

enum class Sentence { None, GGA, RMC };

// Confere o checksum XOR entre '$' e '*'
bool validate_checksum(const std::string& sentence);

// "4807.038" + 'N' -> 48.1173
double nmea_to_degrees(const std::string& value, char hemisphere);

std::vector<std::string> split(const std::string& s, char sep);

class NmeaParser {
public:
    // Processa uma linha. Retorna o tipo reconhecido (None se inválida/ignorada).
    Sentence feed_line(const std::string& line);
    const GpsData& data() const { return data_; }

private:
    void parse_gga(const std::vector<std::string>& f);
    void parse_rmc(const std::vector<std::string>& f);
    GpsData data_;
};

}  // namespace allblue1
