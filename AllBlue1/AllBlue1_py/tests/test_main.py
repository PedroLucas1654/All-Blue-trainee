import pytest

from AllBlue1_py.main import NmeaParser, nmea_to_degrees, validate_checksum

GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"


def test_checksum():
    assert validate_checksum(GGA)
    assert validate_checksum(RMC)
    assert not validate_checksum(GGA[:-2] + "00")
    assert not validate_checksum("lixo")


def test_nmea_to_degrees():
    assert nmea_to_degrees("4807.038", "N") == pytest.approx(48.1173)
    assert nmea_to_degrees("4807.038", "S") == pytest.approx(-48.1173)
    assert nmea_to_degrees("01131.000", "E") == pytest.approx(11.516667)


def test_parse_gga_e_rmc():
    p = NmeaParser()
    assert p.feed_line(GGA) == "GGA"
    assert p.data.has_fix
    assert p.data.satellites == 8
    assert p.data.altitude_m == pytest.approx(545.4)
    assert p.data.hdop == pytest.approx(0.9)

    assert p.feed_line(RMC + "\r\n") == "RMC"
    assert p.data.latitude == pytest.approx(48.1173)
    assert p.data.speed_kmh == pytest.approx(22.4 * 1.852)
    assert p.data.utc_time == "12:35:19"
    assert p.data.utc_date == "23/03/1994"


def test_linha_invalida_ignorada():
    p = NmeaParser()
    assert p.feed_line("$GPRMC,123519,V,,,,,,,230394,,*00") is None
    assert not p.data.has_fix


def _with_checksum(body: str) -> str:
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return f"${body}*{calc:02X}"


def test_checksum_malformado():
    assert not validate_checksum(GGA[:-3])          # sem checksum
    assert not validate_checksum(GGA[:-2])          # '*' sem dígitos
    assert not validate_checksum(GGA[:-2] + "4Z")   # caractere hex inválido
    assert not validate_checksum(GGA + "junk")      # lixo depois do checksum
    assert not validate_checksum(GGA[:20])          # linha truncada
    assert validate_checksum(RMC + "\r\n")          # \r\n no final é aceito


def test_fix_sem_posicao_nao_reaproveita_coordenada_antiga():
    p = NmeaParser()
    assert p.feed_line(RMC) == "RMC"
    assert p.data.has_fix and p.data.latitude == pytest.approx(48.1173)

    vazio = _with_checksum("GPRMC,123519,A,,,,,022.4,084.4,230394,003.1,W")
    assert p.feed_line(vazio) == "RMC"
    assert not p.data.has_fix
    assert p.data.latitude == 0.0 and p.data.longitude == 0.0

    # mesma coisa pela GGA (qualidade 1, campos de posição vazios)
    p.feed_line(RMC)
    gga_vazio = _with_checksum("GPGGA,123519,,,,,1,08,0.9,545.4,M,46.9,M,,")
    assert p.feed_line(gga_vazio) == "GGA"
    assert not p.data.has_fix
    assert p.data.latitude == 0.0


def test_hemisferio_invalido_e_coordenada_impossivel():
    p = NmeaParser()
    hemi = _with_checksum("GPRMC,123519,A,4807.038,X,01131.000,E,022.4,084.4,230394,003.1,W")
    assert p.feed_line(hemi) == "RMC"
    assert not p.data.has_fix
    fora = _with_checksum("GPRMC,123519,A,9907.038,N,01131.000,E,022.4,084.4,230394,003.1,W")
    assert p.feed_line(fora) == "RMC"
    assert not p.data.has_fix


def test_campo_numerico_invalido_nao_derruba_o_parser():
    p = NmeaParser()
    ruim = _with_checksum("GPRMC,123519,A,4807.038,N,01131.000,E,abc,084.4,230394,003.1,W")
    assert p.feed_line(ruim) is None  # ValueError tratado dentro do feed_line
