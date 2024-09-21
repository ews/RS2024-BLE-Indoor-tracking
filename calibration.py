import time
import logging
import json
from bluepy.btle import DefaultDelegate
from bluepy.btle import Scanner
from config import target_devices  # Import target devices from config.py

# Set up logging
logging.basicConfig(filename="calibration.log", level=logging.DEBUG)
logger = logging.getLogger(__name__)


class CalibrationDelegate(DefaultDelegate):
    def __init__(self, target_device):
        DefaultDelegate.__init__(self)
        self.target_device = target_device
        self.rssi_values = []

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr == self.target_device:
            logger.debug(f"Discovered target device {dev.addr} with RSSI {dev.rssi}")
            self.rssi_values.append(dev.rssi)


def calibrate_beacon(target_device, scan_time=10):
    """
    Calibrates the target BLE beacon by averaging the RSSI over a specified scan time.

    Args:
        target_device (str): MAC address of the target BLE device.
        scan_time (int): Time to scan in seconds.

    Returns:
        float: The average RSSI at 1 meter (calibrated P_tx).
    """
    scanner = Scanner().withDelegate(CalibrationDelegate(target_device))
    start_time = time.time()

    logger.info(
        f"Starting calibration for device {target_device} for {scan_time} seconds."
    )
    while time.time() - start_time < scan_time:
        scanner.scan(1.0)  # Scan every 1 second

    rssi_values = scanner.delegate.rssi_values
    if len(rssi_values) == 0:
        logger.error("No RSSI data received for calibration.")
        raise ValueError("No RSSI data collected.")

    # Calculate the average RSSI
    average_rssi = sum(rssi_values) / len(rssi_values)
    logger.info(f"Calibration complete. Average RSSI: {average_rssi} dBm at 1 meter.")

    return average_rssi


def save_calibration_to_file(
    mac_address, calibrated_tx_power, file_path="calibrated_beacons.json"
):
    """
    Saves the calibrated Tx Power for a specific beacon to a JSON file.

    Args:
        mac_address (str): The MAC address of the beacon.
        calibrated_tx_power (float): The calibrated Tx Power (RSSI at 1 meter).
        file_path (str): The path to the JSON file to store the calibration data.
    """
    try:
        # Load existing data if the file already exists
        try:
            with open(file_path, "r") as file:
                data = json.load(file)
        except FileNotFoundError:
            data = {}

        # Add or update the calibration data
        data[mac_address] = calibrated_tx_power

        # Save back to the file
        with open(file_path, "w") as file:
            json.dump(data, file, indent=4)

        logger.info(f"Saved calibration for {mac_address}: {calibrated_tx_power} dBm")
        print(f"Calibration saved to {file_path}")

    except Exception as e:
        logger.error(f"Error saving calibration data: {e}")
        print(f"Error saving calibration data: {e}")


def choose_mac_address():
    """
    Presents the user with a list of MAC addresses from the config file and prompts them to select one.

    Returns:
        str: The selected MAC address.
    """
    mac_addresses = list(target_devices.keys())

    print("Available MAC addresses for calibration:")
    for i, mac in enumerate(mac_addresses, 1):
        print(f"{i}. {mac}")

    while True:
        try:
            choice = int(
                input("Select a MAC address by entering the corresponding number: ")
            )
            if 1 <= choice <= len(mac_addresses):
                return mac_addresses[choice - 1]
            else:
                print(
                    f"Invalid choice, please select a number between 1 and {len(mac_addresses)}."
                )
        except ValueError:
            print("Invalid input, please enter a number.")


if __name__ == "__main__":
    # Offer the user a selection of MAC addresses from config.py
    target_mac_address = choose_mac_address()

    try:
        # Run the calibration for 10 seconds (or as needed)
        calibrated_tx_power = calibrate_beacon(target_mac_address, scan_time=10)
        print(f"Calibrated Tx Power (RSSI at 1 meter): {calibrated_tx_power} dBm")

        # Save the calibration result to a file
        save_calibration_to_file(target_mac_address, calibrated_tx_power)

    except Exception as e:
        logger.error(f"Calibration failed: {e}")
        print(f"Error during calibration: {e}")
