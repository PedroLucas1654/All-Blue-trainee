#include <AllBlue1_cpp/core.hpp>

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace allblue1 {

namespace {
double to_double(const std::string& s) {
    return s.empty() ? 0.0 : std::strtod(s.c_str(), nullptr);
}
int to_int(const std::string& s) {
    return s.empty() ? 0 : std::atoi(s.c_str());
}
std::string fmt_time(const std::string& t) {  // 123519 -> 12:35:19
    if (t.size() < 6) return "";
    return t.substr(0, 2) + ":" + t.substr(2, 2) + ":" + t.substr(4, 2);
}
std::string fmt_date(const std::string& d) {  // 230394 -> 23/03/1994
    if (d.size() < 6) return "";
    int yy = std::atoi(d.substr(4, 2).c_str());
    std::string century = (yy >= 80) ? "19" : "20";  // época do GPS: 1980
    return d.substr(0, 2) + "/" + d.substr(2, 2) + "/" + century + d.substr(4, 2);
}

// Valida e converte a posição. Falha se faltar campo, hemisfério inválido,
// formato ddmm.mmmm quebrado ou coordenada fora do intervalo possível.
bool parse_position(const std::string& lat, const std::string& ns,
                    const std::string& lon, const std::string& ew,
                    double& lat_out, double& lon_out) {
    if (lat.empty() || ns.empty() || lon.empty() || ew.empty()) return false;
    if ((ns[0] != 'N' && ns[0] != 'S') || (ew[0] != 'E' && ew[0] != 'W')) return false;
    auto dlat = lat.find('.');
    auto dlon = lon.find('.');
    if (dlat == std::string::npos || dlat < 2 || dlon == std::string::npos || dlon < 2)
        return false;
    lat_out = nmea_to_degrees(lat, ns[0]);
    lon_out = nmea_to_degrees(lon, ew[0]);
    return std::fabs(lat_out) <= 90.0 && std::fabs(lon_out) <= 180.0;
}

// Só marca "tem fix" se a posição também for utilizável. Sem posição válida,
// zera as coordenadas: nunca deixa valor antigo passar por leitura nova.
void apply_fix(GpsData& d, bool fix, const std::string& lat, const std::string& ns,
               const std::string& lon, const std::string& ew) {
    double la = 0.0, lo = 0.0;
    if (fix && parse_position(lat, ns, lon, ew, la, lo)) {
        d.has_fix = true;
        d.latitude = la;
        d.longitude = lo;
    } else {
        d.has_fix = false;
        d.latitude = 0.0;
        d.longitude = 0.0;
    }
}
}  // namespace

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(cur);
    return out;
}

bool validate_checksum(const std::string& s) {
    auto star = s.find('*');
    if (s.empty() || s[0] != '$' || star == std::string::npos) return false;

    // ignora só \r\n no final; exige EXATAMENTE 2 dígitos hexadecimais depois do '*'
    std::size_t end = s.size();
    while (end > star && (s[end - 1] == '\r' || s[end - 1] == '\n')) --end;
    if (end - star - 1 != 2) return false;
    if (!std::isxdigit(static_cast<unsigned char>(s[star + 1])) ||
        !std::isxdigit(static_cast<unsigned char>(s[star + 2])))
        return false;

    unsigned char calc = 0;
    for (std::size_t i = 1; i < star; ++i) calc ^= static_cast<unsigned char>(s[i]);
    unsigned long given = std::strtoul(s.substr(star + 1, 2).c_str(), nullptr, 16);
    return calc == given;
}

double nmea_to_degrees(const std::string& value, char hemisphere) {
    auto dot = value.find('.');
    if (value.empty() || dot == std::string::npos || dot < 2) return 0.0;
    double deg = to_double(value.substr(0, dot - 2));
    double min = to_double(value.substr(dot - 2));
    double result = deg + min / 60.0;
    return (hemisphere == 'S' || hemisphere == 'W') ? -result : result;
}

Sentence NmeaParser::feed_line(const std::string& raw) {
    std::string line = raw;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();

    if (!validate_checksum(line)) return Sentence::None;

    auto body = line.substr(1, line.find('*') - 1);
    auto f = split(body, ',');
    if (f.empty() || f[0].size() < 5) return Sentence::None;

    std::string type = f[0].substr(f[0].size() - 3);  // GPGGA / GNGGA -> GGA
    if (type == "GGA") { parse_gga(f); return Sentence::GGA; }
    if (type == "RMC") { parse_rmc(f); return Sentence::RMC; }
    return Sentence::None;
}

// $GPGGA,hora,lat,N,lon,E,fix,sats,hdop,alt,M,...
void NmeaParser::parse_gga(const std::vector<std::string>& f) {
    if (f.size() < 10) return;
    data_.utc_time = fmt_time(f[1]);
    apply_fix(data_, to_int(f[6]) > 0, f[2], f[3], f[4], f[5]);
    data_.satellites = to_int(f[7]);
    data_.hdop = to_double(f[8]);
    data_.altitude_m = to_double(f[9]);
}

// $GPRMC,hora,status,lat,N,lon,E,vel(nós),curso,data,...
void NmeaParser::parse_rmc(const std::vector<std::string>& f) {
    if (f.size() < 10) return;
    data_.utc_time = fmt_time(f[1]);
    apply_fix(data_, f[2] == "A", f[3], f[4], f[5], f[6]);
    data_.speed_kmh = to_double(f[7]) * 1.852;
    data_.course_deg = to_double(f[8]);
    data_.utc_date = fmt_date(f[9]);
}

}  // namespace allblue1
