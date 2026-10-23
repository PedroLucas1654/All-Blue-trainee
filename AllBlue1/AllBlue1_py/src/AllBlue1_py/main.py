"""GPS GY-NEO6MV2 (u-blox NEO-6M): parser NMEA + nó ROS 2 que publica gps/fix e gps/vel."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

# ROS e pyserial são opcionais na importação: assim o parser roda nos testes sem ROS.
try:
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None


@dataclass
class GpsData:
    has_fix: bool = False
    latitude: float = 0.0       # graus decimais (S negativo)
    longitude: float = 0.0      # graus decimais (W negativo)
    altitude_m: float = 0.0
    speed_kmh: float = 0.0
    course_deg: float = 0.0
    hdop: float = 0.0
    satellites: int = 0
    utc_time: str = ""          # HH:MM:SS
    utc_date: str = ""          # DD/MM/YYYY


_CHECKSUM_RE = re.compile(r"\$([^*]*)\*([0-9A-Fa-f]{2})")


def validate_checksum(sentence: str) -> bool:
    """Confere o checksum XOR entre '$' e '*'.

    Exige EXATAMENTE 2 dígitos hexadecimais depois do '*' e nada depois deles
    (além de \\r\\n no final).
    """
    m = _CHECKSUM_RE.fullmatch(sentence.rstrip("\r\n"))
    if m is None:
        return False
    calc = 0
    for ch in m.group(1):
        calc ^= ord(ch)
    return calc == int(m.group(2), 16)


def nmea_to_degrees(value: str, hemisphere: str) -> float:
    """'4807.038' + 'N' -> 48.1173"""
    dot = value.find(".")
    if dot < 2:
        return 0.0
    result = float(value[: dot - 2] or 0) + float(value[dot - 2:]) / 60.0
    return -result if hemisphere in ("S", "W") else result


def _f(s: str) -> float:
    return float(s) if s else 0.0


def _i(s: str) -> int:
    return int(s) if s else 0


def _fmt_time(t: str) -> str:
    return f"{t[0:2]}:{t[2:4]}:{t[4:6]}" if len(t) >= 6 else ""


def _fmt_date(d: str) -> str:
    if len(d) < 6:
        return ""
    century = "19" if int(d[4:6]) >= 80 else "20"  # época do GPS: 1980
    return f"{d[0:2]}/{d[2:4]}/{century}{d[4:6]}"


def _parse_position(lat: str, ns: str, lon: str, ew: str):
    """Devolve (lat, lon) em graus decimais, ou None se faltar campo, o hemisfério
    for inválido, o formato ddmm.mmmm estiver quebrado ou a coordenada for impossível."""
    if not (lat and ns and lon and ew):
        return None
    if ns not in ("N", "S") or ew not in ("E", "W"):
        return None
    if lat.find(".") < 2 or lon.find(".") < 2:
        return None
    la = nmea_to_degrees(lat, ns)
    lo = nmea_to_degrees(lon, ew)
    if abs(la) > 90.0 or abs(lo) > 180.0:
        return None
    return la, lo


def _apply_fix(d: GpsData, fix: bool, pos) -> None:
    """Só marca 'tem fix' se a posição também for utilizável. Sem posição válida,
    zera as coordenadas: nunca deixa valor antigo passar por leitura nova."""
    if fix and pos is not None:
        d.has_fix = True
        d.latitude, d.longitude = pos
    else:
        d.has_fix = False
        d.latitude = d.longitude = 0.0


class NmeaParser:
    def __init__(self) -> None:
        self.data = GpsData()

    def feed_line(self, line: str) -> str | None:
        """Processa uma linha. Retorna 'GGA'/'RMC' se reconhecida, senão None."""
        line = line.strip()
        if not validate_checksum(line):
            return None
        fields = line[1: line.index("*")].split(",")
        if len(fields[0]) < 5:
            return None
        kind = fields[0][-3:]  # GPGGA / GNGGA -> GGA
        try:
            if kind == "GGA":
                self._parse_gga(fields)
                return "GGA"
            if kind == "RMC":
                self._parse_rmc(fields)
                return "RMC"
        except ValueError:  # campo numérico malformado: ignora a frase, não derruba o nó
            return None
        return None

    # $GPGGA,hora,lat,N,lon,E,fix,sats,hdop,alt,M,...
    def _parse_gga(self, f: list[str]) -> None:
        if len(f) < 10:
            return
        d = self.data
        d.utc_time = _fmt_time(f[1])
        _apply_fix(d, _i(f[6]) > 0, _parse_position(f[2], f[3], f[4], f[5]))
        d.satellites = _i(f[7])
        d.hdop = _f(f[8])
        d.altitude_m = _f(f[9])

    # $GPRMC,hora,status,lat,N,lon,E,vel(nós),curso,data,...
    def _parse_rmc(self, f: list[str]) -> None:
        if len(f) < 10:
            return
        d = self.data
        d.utc_time = _fmt_time(f[1])
        _apply_fix(d, f[2] == "A", _parse_position(f[3], f[4], f[5], f[6]))
        d.speed_kmh = _f(f[7]) * 1.852
        d.course_deg = _f(f[8])
        d.utc_date = _fmt_date(f[9])


class GpsNode(Node):
    """Lê a serial e publica:
    gps/fix (sensor_msgs/NavSatFix) e gps/vel (geometry_msgs/TwistStamped, ENU em m/s)."""

    def __init__(self) -> None:
        super().__init__("gps_node")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = self.declare_parameter("baud", 9600).value
        self.frame_id = self.declare_parameter("frame_id", "gps_link").value
        # sigma ≈ HDOP * hdop_to_sigma_m. 0 (ou negativo) = covariância desconhecida.
        self.hdop_to_sigma_m = float(self.declare_parameter("hdop_to_sigma_m", 2.5).value)

        self.fix_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "gps/vel", 10)

        self.parser = NmeaParser()
        self.buffer = ""
        # timeout=0: leitura não bloqueante; o timer consulta a porta
        self.ser = serial.Serial(port, baud, timeout=0)
        self.get_logger().info(f"Lendo GPS em {port} @ {baud} baud")
        self.create_timer(0.05, self.poll)

    def poll(self) -> None:
        waiting = self.ser.in_waiting
        if not waiting:
            return
        self.buffer += self.ser.read(waiting).decode("ascii", errors="ignore")
        # tudo antes do último '\n' são linhas completas; o resto é uma linha pela metade
        *lines, self.buffer = self.buffer.split("\n")
        if len(self.buffer) > 4096:  # proteção contra lixo sem '\n'
            self.buffer = ""
        for line in lines:
            if self.parser.feed_line(line) == "RMC":
                self.publish(self.parser.data)

    def publish(self, d: GpsData) -> None:
        stamp = self.get_clock().now().to_msg()  # hora do PC; UTC do GPS está em d.utc_time

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self.frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS

        if d.has_fix:
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude = d.latitude
            fix.longitude = d.longitude
            fix.altitude = d.altitude_m
            # Covariância = ESTIMATIVA, não garantia: sigma ≈ HDOP * hdop_to_sigma_m.
            # O padrão (2,5 m) vem do valor de CEP citado no datasheet do NEO-6 em
            # condições ideais; o erro real depende do céu visível e do multipath.
            # Calibre com dados reais ou use hdop_to_sigma_m:=0.0 para "desconhecida".
            if self.hdop_to_sigma_m > 0.0:
                sh = max(d.hdop, 1.0) * self.hdop_to_sigma_m
                sv = sh * 2.0
                fix.position_covariance = [sh * sh, 0.0, 0.0, 0.0, sh * sh, 0.0, 0.0, 0.0, sv * sv]
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            else:
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.latitude = fix.longitude = fix.altitude = float("nan")
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

        if d.has_fix:
            # curso é medido a partir do Norte, no sentido horário
            v = d.speed_kmh / 3.6
            c = math.radians(d.course_deg)
            vel = TwistStamped()
            vel.header = fix.header
            vel.twist.linear.x = v * math.sin(c)  # Leste
            vel.twist.linear.y = v * math.cos(c)  # Norte
            self.vel_pub.publish(vel)


def main(args=None) -> None:
    if rclpy is None or serial is None:
        raise SystemExit("Faltam dependências: ROS 2 (rclpy) e/ou python3-serial.")
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
