import curses
import time
import argparse
import json
import logging
import socket  # Import socket to send UDP packets
from bluepy.btle import Scanner, DefaultDelegate
from config import target_devices, kalman_config

# Set up logging
logging.basicConfig(filename='locator.log', level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Load calibration data
def load_calibration(file_path="calibrated_beacons.json"):
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        logger.error(f"Calibration file {file_path} not found.")
        return {}

class KalmanFilter:
    def __init__(self, process_variance=1e-3, measurement_variance=2.0, initial_estimate=0.0, initial_error=1.0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.error = initial_error

    def update(self, measurement):
        kalman_gain = self.error / (self.error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error = (1 - kalman_gain) * self.error + self.process_variance
        logger.debug(f"Kalman Filter updated: RSSI={measurement}, Estimated RSSI={self.estimate}")
        return self.estimate

def calculate_distance(rssi, tx_power, path_loss_exponent=2):
    if rssi == 0:
        return -1.0
    distance = 10 ** ((tx_power - rssi) / (10 * path_loss_exponent))
    logger.debug(f"Distance calculated: RSSI={rssi}, TX Power={tx_power}, Distance={distance:.2f} meters")
    return distance

# Dictionary to cache iBeacon data, keyed by MAC address
ibeacon_cache = {}

def extract_ibeacon_data(dev):
    if dev.addr in ibeacon_cache:
        logger.debug(f"Returning cached iBeacon data for {dev.addr}")
        return ibeacon_cache[dev.addr]

    # Parse the iBeacon data only if not cached
    for (adtype, desc, value) in dev.getScanData():
        logger.debug(f"Scan data: {adtype}, {desc}, {value}")
        if desc == "Manufacturer":
            try:
                if value[0:4] == "4c00" and len(value) >= 46:
                    uuid = value[8:40]
                    major = int(value[40:44], 16)
                    minor = int(value[44:48], 16)
                    tx_power = int(value[48:50], 16) - 256  # Signed 8-bit value
                    ibeacon_cache[dev.addr] = (f"{uuid}-{major}-{minor}", tx_power)
                    logger.debug(f"Cached iBeacon data for {dev.addr}")
                    return ibeacon_cache[dev.addr]
            except Exception as e:
                logger.error(f"Error parsing iBeacon data: {e}")
    return None


class ScanDelegate(DefaultDelegate):
    def __init__(self, kalman_filters, target_devices, calibration_data, udp_socket, udp_address, udp_port):
        """
        Initializes the ScanDelegate class, which processes BLE scan data.

        Args:
            kalman_filters: A dictionary of Kalman filters for devices.
            target_devices: A dictionary of target MAC addresses to track.
            calibration_data: A dictionary of calibrated Tx Power values.
            udp_socket: A socket to send UDP data to the Clojure server.
            udp_address: The Clojure server's address.
            udp_port: The Clojure server's UDP port.
        """
        DefaultDelegate.__init__(self)
        self.kalman_filters = kalman_filters
        self.target_devices = target_devices
        self.calibration_data = calibration_data
        self.device_info = {}
        self.udp_socket = udp_socket  # UDP socket for sending data
        self.udp_address = udp_address  # Clojure server address
        self.udp_port = udp_port  # Clojure server port

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr not in self.target_devices:
            return
        ibeacon_data = extract_ibeacon_data(dev)
        if ibeacon_data is None:
            return
        ibeacon_uuid, default_tx_power = ibeacon_data

        tx_power = self.calibration_data.get(dev.addr, default_tx_power)

        if dev.addr not in self.kalman_filters:
            self.kalman_filters[dev.addr] = KalmanFilter(
                process_variance=kalman_config['process_variance'],
                measurement_variance=kalman_config['measurement_variance'],
                initial_estimate=dev.rssi,
                initial_error=kalman_config['initial_error']
            )
        kalman = self.kalman_filters[dev.addr]
        filtered_rssi = kalman.update(dev.rssi)
        raw_distance = calculate_distance(dev.rssi, tx_power)
        kalman_distance = calculate_distance(filtered_rssi, tx_power)

        self.device_info[dev.addr] = {
            "uuid": ibeacon_uuid,
            "raw_rssi": dev.rssi,
            "filtered_rssi": filtered_rssi,
            "raw_distance": raw_distance,
            "kalman_distance": kalman_distance,
            "tx_power": tx_power
        }

        logger.info(f"Device discovered: MAC={dev.addr}, UUID={ibeacon_uuid}, RSSI={dev.rssi}, Distance={raw_distance:.2f}m")

        # Send data to the Clojure UDP server as JSON
        try:
            json_data = json.dumps({"uuid": ibeacon_uuid, "rssi": dev.rssi})
            self.udp_socket.sendto(json_data.encode('utf-8'), (self.udp_address, self.udp_port))
            logger.info(f"Sent UDP data to {self.udp_address}:{self.udp_port} - {json_data}")
        except Exception as e:
            logger.error(f"Error sending UDP data: {e}")


def curses_display(stdscr, delegate):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    scanner = Scanner().withDelegate(delegate)

    try:
        while True:
            stdscr.erase()
            scanner.scan(0.1)

            stdscr.addstr(0, 0, "BLE Scanner (Target Devices with Raw & Kalman Filtering)")
            stdscr.addstr(1, 0, "Press 'q' to quit")
            stdscr.addstr(3, 0, f"{'MAC Address':<20} {'iBeacon UUID':<40} {'RSSI (Raw)':<12} {'Distance (Raw)':<12} {'Distance (Kalman)':<15} {'TX Power':<10}")
            stdscr.addstr(4, 0, "-" * 110)

            for idx, (addr, info) in enumerate(delegate.device_info.items(), start=5):
                uuid = info["uuid"]
                raw_rssi = info["raw_rssi"]
                raw_distance = info["raw_distance"]
                kalman_distance = info["kalman_distance"]
                tx_power = info["tx_power"]

                stdscr.addstr(idx, 0, f"{addr:<20} {uuid:<40} {raw_rssi:<12.2f} {raw_distance:<12.2f} {kalman_distance:<15.2f} {tx_power:<10}")

            stdscr.refresh()

            key = stdscr.getch()
            if key == ord('q'):
                break

            time.sleep(0.1)
    except Exception as e:
        logger.error(f"Error during curses display: {e}")


def main(stdscr, args):
    kalman_filters = {}
    udp_socket = None

    # Load calibration data
    calibration_data = load_calibration()

    try:
        # Create a UDP socket to send data to the Clojure server
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info(f"UDP socket created to send data to {args.udp_host}:{args.udp_port}")

        # Initialize the BLE scanner delegate with the UDP socket
        delegate = ScanDelegate(kalman_filters, target_devices, calibration_data, udp_socket, args.udp_host, args.udp_port)
        curses_display(stdscr, delegate)
    finally:
        if udp_socket:
            udp_socket.close()
            logger.info("UDP socket closed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BLE Scanner with Kalman Filtering and UDP data sending.")
    parser.add_argument("--udp-host", type=str, default="localhost", help="Clojure UDP server host (default: localhost).")
    parser.add_argument("--udp-port", type=int, default=5000, help="Clojure UDP server port (default: 5000).")
    args = parser.parse_args()

    logger.info("Starting BLE Scanner application.")

    # Start the curses interface
    curses.wrapper(main, args)

    logger.info("BLE Scanner application terminated.")
