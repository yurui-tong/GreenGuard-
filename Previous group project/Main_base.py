# import time
# import numpy as np
# import GPSFunction as GPS
# from send import send_continuously
# from Basecofic import collect_gps_samples, average_gps   
# import pymap3d as pm  # Import the library you installed


# gps_data = collect_gps_samples(10)  # start with 10 samples
# base = average_gps(gps_data)
# lat_base = base["lat"]
# lon_base = base["lon"]
# height_base = base["height"]

# while True:
#     current = GPS.get_GPS(port="COM6")

# # Extract current values
#     lat = current["lat"]
#     lon = current["lon"]
#     height = current["height"]



#     # Convert Geodetic (Lat/Lon) to ENU (East, North, Up)
#     # x = East (meters), y = North (meters), z = Up (meters)
#     x_current, y_current, hm = pm.geodetic2enu(lat, lon, height, lat_base, lon_base, height_base)

#     # Format correction as a string
#     correc_str = f"{x_current:.6f},{y_current:.6f}"

#     print("Sending correction:", correc_str)
#     send_continuously(correc_str)

#     time.sleep(0.5)  # send once per second



import time
import pymap3d as pm
import numpy as np
from serial import Serial
from pyubx2 import UBXReader, NMEA_PROTOCOL, UBX_PROTOCOL
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.exception import TimeoutException, XBeeException

# --- CONFIGURATION ---
GPS_PORT = "COM6"           # Your u-blox USB port
RADIO_PORT = "COM4"         # Your XBee USB port
REMOTE_ID = "0013A20041F4A26D" # Benjamin's Rover ID
BAUD_RATE = 115200

# --- HELPER FUNCTIONS (Previously in separate files) ---

def get_GPS_packet(stream):
    """
    Reads from the OPEN serial stream until it finds a valid NAV-PVT packet.
    Returns: dict with lat/lon/height or None
    """
    ubr = UBXReader(stream, protfilter= UBX_PROTOCOL)
    
    # Try 20 times to find a packet in the buffer (prevents infinite blocking)
    for _ in range(20):
        try:
            (raw_data, parsed) = ubr.read()
            if parsed is None: continue

            # We want NAV-PVT for precision
            if parsed.identity == "NAV-PVT":
                # Filter out "Null Island" (0,0) or invalid data
                if abs(parsed.lat) < 1.0 and abs(parsed.lon) < 1.0: continue
                
                return {
                    "lat": parsed.lat,    # Scale to degrees
                    "lon": parsed.lon,
                    "height": parsed.hMSL / 1000.0 # Scale mm to meters
                }
        except Exception:
            continue
            
    return None

def send_radio_async(device, remote, message):
    """
    Sends data without waiting for an ACK (Fire and Forget).
    """
    try:
        device.send_data_async(remote, message.encode("utf-8"))
        print(f"   [Radio] Sent: {message}")
    except XBeeException as e:
        print(f"   [Radio Error] {e}")

def get_average_position(data_list):
    """
    Calculates the mean of a list of GPS dictionaries.
    """
    lats = [d["lat"] for d in data_list]
    lons = [d["lon"] for d in data_list]
    heights = [d["height"] for d in data_list]
    return {
        "lat": np.mean(lats),
        "lon": np.mean(lons),
        "height": np.mean(heights)
    }

# --- MAIN SYSTEM ---

def run_master_station():
    print("--- STARTING MASTER STATION ---")
    
    # 1. INITIALIZE HARDWARE
    try:
        print(f"1. Connecting to XBee on {RADIO_PORT}...")
        xbee = XBeeDevice(RADIO_PORT, 115200)
        xbee.open()
        remote = RemoteXBeeDevice(xbee, XBee64BitAddress.from_hex_string(REMOTE_ID))
        
        print(f"2. Connecting to GPS on {GPS_PORT}...")
        gps_serial = Serial(GPS_PORT, 115200, timeout=1)
        
    except Exception as e:
        print(f"\nCRITICAL HARDWARE ERROR: {e}")
        print("-> Tip: Unplug both USB cables and plug them back in.")
        return

    try:
        # 2. CALIBRATION PHASE
        print("\n--- CALIBRATING (DO NOT MOVE ANTENNA) ---")
        samples = []
        start_time = time.time()
        
        # Collect for 5 seconds
        while time.time() - start_time < 80:
            packet = get_GPS_packet(gps_serial)
            if packet:
                samples.append(packet)
                print(f"   Sample {len(samples)}: {packet['lat']:.5f}...", end='\r')
        
        if not samples:
            print("\n GPS ERROR: No satellites found. Cannot set Origin.")
            return

        base = get_average_position(samples)
        print(f"\n ORIGIN SET: {base['lat']:.6f}, {base['lon']:.6f}\n")

        # 3. CONTINUOUS LOOP
        print("--- STARTING STREAM (Press Ctrl+C to Stop) ---")
        
        while True:
            # A. Read Live GPS
            current = get_GPS_packet(gps_serial)
            
            if current:
                # B. Calculate Drift (Current - Base)
                # Returns East (x), North (y), Up (z) in Meters
                x, y, z = pm.geodetic2enu(
                   current["lat"], current["lon"], current["height"],
                   base["lat"], base["lon"], base["height"]
                )
                
                # C. Send Correction
                # Example: "0.02,-0.15"
                msg = f"{x:.8f},{y:.8f},{z:.8f}"
                send_radio_async(xbee, remote, msg)
                
            else:
                print("   [GPS] Searching for satellites...")
                
            # D. Loop Speed (Prevent CPU hogging)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\n>>> STOPPING... <<<")
    finally:
        # 4. CLEANUP (Crucial to prevent "Access Denied" next time)
        if xbee.is_open(): xbee.close()
        if gps_serial.is_open: gps_serial.close()
        print("Ports Closed. Hardware Released.")

if __name__ == "__main__":
    run_master_station()