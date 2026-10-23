#include <AllBlue1_cpp/core.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::fprintf(stderr, "FALHOU %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            std::exit(1);                                                  \
        }                                                                  \
    } while (0)

static bool near(double a, double b, double eps = 1e-4) { return std::fabs(a - b) < eps; }

// Monta "$corpo*CS" com o checksum certo (para testar campos vazios/inválidos)
static std::string with_checksum(const std::string& body) {
    unsigned char c = 0;
    for (char ch : body) c ^= static_cast<unsigned char>(ch);
    char buf[8];
    std::snprintf(buf, sizeof(buf), "*%02X", c);
    return "$" + body + buf;
}

int main() {
    const std::string gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    const std::string rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";

    // --- checksum ---
    CHECK(allblue1::validate_checksum(gga));
    CHECK(allblue1::validate_checksum(rmc));
    CHECK(allblue1::validate_checksum(rmc + "\r\n"));
    CHECK(!allblue1::validate_checksum(gga.substr(0, gga.size() - 2) + "00"));  // valor errado
    CHECK(!allblue1::validate_checksum("lixo"));
    CHECK(!allblue1::validate_checksum(gga.substr(0, gga.size() - 3)));         // sem checksum
    CHECK(!allblue1::validate_checksum(gga.substr(0, gga.size() - 2)));         // '*' sem dígitos
    CHECK(!allblue1::validate_checksum(gga.substr(0, gga.size() - 2) + "4Z"));  // hex inválido
    CHECK(!allblue1::validate_checksum(gga + "junk"));                          // lixo depois
    CHECK(!allblue1::validate_checksum(gga.substr(0, 20)));                     // truncada

    // --- conversão de coordenadas ---
    CHECK(near(allblue1::nmea_to_degrees("4807.038", 'N'), 48.1173));
    CHECK(near(allblue1::nmea_to_degrees("4807.038", 'S'), -48.1173));
    CHECK(near(allblue1::nmea_to_degrees("01131.000", 'E'), 11.516667));

    // --- frases boas ---
    allblue1::NmeaParser p;
    CHECK(p.feed_line(gga) == allblue1::Sentence::GGA);
    CHECK(p.data().has_fix);
    CHECK(p.data().satellites == 8);
    CHECK(near(p.data().altitude_m, 545.4));
    CHECK(near(p.data().hdop, 0.9));

    CHECK(p.feed_line(rmc + "\r\n") == allblue1::Sentence::RMC);
    CHECK(near(p.data().latitude, 48.1173));
    CHECK(near(p.data().speed_kmh, 22.4 * 1.852));
    CHECK(p.data().utc_time == "12:35:19");
    CHECK(p.data().utc_date == "23/03/1994");

    // --- linha inválida é ignorada ---
    allblue1::NmeaParser q;
    CHECK(q.feed_line("$GPRMC,123519,V,,,,,,,230394,,*00") == allblue1::Sentence::None);
    CHECK(!q.data().has_fix);

    // --- fix "válido" SEM posição: não pode reaproveitar coordenada antiga ---
    allblue1::NmeaParser r;
    CHECK(r.feed_line(rmc) == allblue1::Sentence::RMC);
    CHECK(r.data().has_fix && near(r.data().latitude, 48.1173));
    auto rmc_vazio = with_checksum("GPRMC,123519,A,,,,,022.4,084.4,230394,003.1,W");
    CHECK(r.feed_line(rmc_vazio) == allblue1::Sentence::RMC);
    CHECK(!r.data().has_fix);
    CHECK(r.data().latitude == 0.0 && r.data().longitude == 0.0);

    // mesma coisa pela GGA (qualidade 1, mas campos de posição vazios)
    CHECK(r.feed_line(rmc) == allblue1::Sentence::RMC);
    auto gga_vazio = with_checksum("GPGGA,123519,,,,,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(r.feed_line(gga_vazio) == allblue1::Sentence::GGA);
    CHECK(!r.data().has_fix);
    CHECK(r.data().latitude == 0.0);

    // hemisfério inválido e coordenada impossível também derrubam o fix
    auto rmc_hemi = with_checksum("GPRMC,123519,A,4807.038,X,01131.000,E,022.4,084.4,230394,003.1,W");
    CHECK(r.feed_line(rmc_hemi) == allblue1::Sentence::RMC);
    CHECK(!r.data().has_fix);
    auto rmc_fora = with_checksum("GPRMC,123519,A,9907.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    CHECK(r.feed_line(rmc_fora) == allblue1::Sentence::RMC);
    CHECK(!r.data().has_fix);

    std::puts("OK: todos os testes passaram");
    return 0;
}