import curses
import time
import argparse
import json
import logging
import typing as t

import paho.mqtt.client as mqtt
from bluepy.btle import DefaultDelegate
from bluepy.btle import Scanner

from config import kalman_config
from config import target_devices
from config import threshold_detection_distance_m

# Set up logging
logging.basicConfig(filename="locator.log", level=logging.DEBUG)
logger = logging.getLogger(__name__)


# Load calibration data
def load_calibration(file_path="calibrated_beacons.json"):
    """
    Load the calibrated Tx Power (RSSI at 1 meter) for each BLE beacon from a JSON file.

    Args:
        file_path (str): Path to the JSON file containing calibration data.

    Returns:
        dict: A dictionary mapping MAC addresses to calibrated Tx Power values.
    """
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        logger.error(f"Calibration file {file_path} not found.")
        return {}


# Kalman Filter class remains the same
class KalmanFilter:
    def __init__(
        self,
        process_variance=1e-3,
        measurement_variance=2.0,
        initial_estimate=0.0,
        initial_error=1.0,
    ):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.error = initial_error

    def update(self, measurement):
        kalman_gain = self.error / (self.error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error = (1 - kalman_gain) * self.error + self.process_variance
        # logger.debug(
        #     f"Kalman Filter updated: RSSI={measurement}, Estimated RSSI={self.estimate}"
        # )
        return self.estimate


# Distance calculation with calibrated Tx Power
def calculate_distance(rssi, tx_power, path_loss_exponent=2):
    """
    Calculate the distance using the calibrated Tx Power (RSSI at 1 meter).

    Args:
        rssi (float): Received RSSI value.
        tx_power (float): Calibrated Tx Power (RSSI at 1 meter).
        path_loss_exponent (float): Path loss exponent (default: 2 for free space).

    Returns:
        float: Estimated distance in meters.
    """
    if rssi == 0:
        return -1.0
    distance = 10 ** ((tx_power - rssi) / (10 * path_loss_exponent))
    # logger.debug(
    #     f"Distance calculated: RSSI={rssi}, TX Power={tx_power}, Distance={distance:.2f} meters"
    # )
    return distance


def extract_ibeacon_data(dev):
    """
    Extracts iBeacon data from a BLE device.

    Args:
        dev: A BLE device discovered by the scanner.

    Returns:
        tuple: (UUID, Tx Power) or None if not an iBeacon.
    """
    for adtype, desc, value in dev.getScanData():
        logger.debug(f"Scan data: {adtype}, {desc}, {value}")
        if desc == "Manufacturer":
            try:
                if value[0:4] == "4c00" and len(value) >= 46:
                    uuid = value[8:40]
                    major = int(value[40:44], 16)
                    minor = int(value[44:48], 16)
                    tx_power = int(value[48:50], 16) - 256
                    return (f"{uuid}-{major}-{minor}", tx_power)
            except Exception as e:
                logger.error(f"Error parsing iBeacon data: {e}")
    return None


class ScanDelegate(DefaultDelegate):
    def __init__(
        self,
        kalman_filters: dict,
        target_devices: dict,
        calibration_data: dict,
        mqtt_client: t.Any = None,
    ):
        """
        Initializes the ScanDelegate class, which processes BLE scan data.

        Args:
            kalman_filters: A dictionary of Kalman filters for devices.
            target_devices: A dictionary of target MAC addresses to track.
            calibration_data: A dictionary of calibrated Tx Power values.
            mqtt_client: Optional MQTT client for publishing data.
        """
        DefaultDelegate.__init__(self)
        self.kalman_filters = kalman_filters
        self.target_devices = target_devices
        self.calibration_data = calibration_data
        self.device_info = {}
        self.mqtt_client = mqtt_client  # Optional MQTT client

    def handleDiscovery(self, dev, isNewDev, isNewData):
        """
        Handles the discovery of BLE devices and applies Kalman filtering and distance calculation.

        Args:
            dev: The discovered BLE device.
            isNewDev: Whether the device is newly discovered.
            isNewData: Whether the device has new data.
        """
        if dev.addr not in self.target_devices:
            return

        # Use calibrated Tx power if available, otherwise use the default Tx power
        tx_power = self.calibration_data.get(dev.addr, -58.54545454545455)

        if dev.addr not in self.kalman_filters:
            self.kalman_filters[dev.addr] = KalmanFilter(
                process_variance=kalman_config["process_variance"],
                measurement_variance=kalman_config["measurement_variance"],
                initial_estimate=dev.rssi,
                initial_error=kalman_config["initial_error"],
            )
        kalman = self.kalman_filters[dev.addr]
        filtered_rssi = kalman.update(dev.rssi)
        raw_distance = calculate_distance(dev.rssi, tx_power)
        if raw_distance <= threshold_detection_distance_m:
            logger.info(f"** FOUND BEACON {dev.addr} WITHIN 3 INCHES **")
        kalman_distance = calculate_distance(filtered_rssi, tx_power)

        self.device_info[dev.addr] = {
            "uuid": "TODO",  # ibeacon_uuid,
            "raw_rssi": dev.rssi,
            "filtered_rssi": filtered_rssi,
            "raw_distance": raw_distance,
            "kalman_distance": kalman_distance,
            "tx_power": tx_power,
        }

        # logger.info(
        #     f"Device discovered: MAC={dev.addr}, UUID={ibeacon_uuid}, RSSI={dev.rssi}, Distance={raw_distance:.2f}m"
        # )

        # If MQTT is enabled, send the data
        if self.mqtt_client:
            # logger.info("sending mqtt")
            self.mqtt_client.publish(f"beacons/{dev.addr}/rssi", dev.rssi)
            self.mqtt_client.publish(f"beacons/{dev.addr}/kalman_rssi", filtered_rssi)
            self.mqtt_client.publish(f"beacons/{dev.addr}/distance", raw_distance)
            self.mqtt_client.publish(
                f"beacons/{dev.addr}/kalman_distance", kalman_distance
            )


def curses_display(stdscr, delegate):
    """
    Displays the BLE scan data in the terminal using the curses library.

    Args:
        stdscr: The curses standard screen object.
        delegate: The ScanDelegate object that processes the BLE scan data.
    """
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    scanner = Scanner().withDelegate(delegate)

    while True:
        stdscr.erase()
        scanner.scan(0.1)

        stdscr.addstr(0, 0, "BLE Scanner (Target Devices with Raw & Kalman Filtering)")
        stdscr.addstr(1, 0, "Press 'q' to quit")
        stdscr.addstr(
            3,
            0,
            f"{'MAC Address':<20} {'iBeacon UUID':<40} {'RSSI (Raw)':<12} {'Distance (Raw)':<12} {'Distance (Kalman)':<15} {'TX Power':<10}",
        )
        stdscr.addstr(4, 0, "-" * 110)

        for idx, (addr, info) in enumerate(delegate.device_info.items(), start=5):
            uuid = info["uuid"]
            raw_rssi = info["raw_rssi"]
            raw_distance = info["raw_distance"]
            kalman_distance = info["kalman_distance"]
            tx_power = info["tx_power"]

            stdscr.addstr(
                idx,
                0,
                f"{addr:<20} {uuid:<40} {raw_rssi:<12.2f} {raw_distance:<12.2f} {kalman_distance:<15.2f} {tx_power:<10}",
            )

        stdscr.refresh()

        try:
            key = stdscr.getch()
            if key == ord("q"):
                break
        except Exception:
            pass

        time.sleep(0.1)


def main(stdscr, args):
    """
    Main function that initializes the scanning process and MQTT client if enabled.

    Args:
        stdscr: The curses standard screen object.
        args: Parsed command-line arguments.
    """
    kalman_filters = {}
    mqtt_client = None

    # Load calibration data
    calibration_data = load_calibration()

    # Initialize MQTT client if MQTT is enabled
    if args.mqtt:
        logger.info("mqtt created")
        mqtt_client = mqtt.Client("BLE_Tracker")
        mqtt_client.connect(args.mqtt_host, args.mqtt_port, 60)
        mqtt_client.loop_start()

    delegate = ScanDelegate(
        kalman_filters, target_devices, calibration_data, mqtt_client
    )
    curses_display(stdscr, delegate)

    # Stop MQTT loop when done
    if mqtt_client:
        mqtt_client.loop_stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="BLE Scanner with Kalman Filtering and optional MQTT publishing."
    )
    parser.add_argument(
        "--mqtt",
        action="store_true",
        help="Enable MQTT publishing of RSSI and distance data.",
    )
    parser.add_argument(
        "--mqtt-host",
        type=str,
        default="localhost",
        help="MQTT broker host (default: localhost).",
    )
    parser.add_argument(
        "--mqtt-port", type=int, default=1883, help="MQTT broker port (default: 1883)."
    )
    args = parser.parse_args()

    logger.info("Starting BLE Scanner application.")

    # Start the curses interface
    curses.wrapper(main, args)

    logger.info("BLE Scanner application terminated.")
