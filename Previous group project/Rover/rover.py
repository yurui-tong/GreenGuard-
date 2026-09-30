import time
import numpy as np

class GPS_rover: 
    def __init__(self, PORT='COM6', BAUD_RATE=115200, TIMEOUT=1): 
        from pyubx2 import UBXReader, NMEA_PROTOCOL, UBX_PROTOCOL
        from serial import Serial

        self.PORT = PORT 
        self.BAUD_RATE = BAUD_RATE 
        self.stream = Serial(self.PORT, self.BAUD_RATE, timeout=TIMEOUT)
        self.ubr = UBXReader(self.stream, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL)
        self.last_GPS = None

    def get_GPS(self):
        latest_valid_data = None

        while self.stream.in_waiting > 0:
            try:
                raw_data, parsed = self.ubr.read()
                if parsed is None:
                    continue

                lat, lon, height, time_str = None, None, None, None

                if parsed.identity == "NAV-PVT":
                    time_str = f"{parsed.hour:02d}:{parsed.min:02d}:{parsed.second:02d}"
                    lat = getattr(parsed, "lat", None)   # in 1e-7 degrees
                    lon = getattr(parsed, "lon", None)
                    height = getattr(parsed, "hMSL", None)     # mm above mean sea level

                if lat is not None and lon is not None:
                    return {
                        "time": time_str,
                        "lat": lat,
                        "lon": lon,
                        "height": height/1000. # convert mm to m
                    }
            except Exception:
                continue

        if latest_valid_data:
            self.last_GPS = latest_valid_data
            return latest_valid_data
        else:
            return None


    def collect_gps_samples(self, duration_sec=60):
        data_list = []
        start_time = time.time()
        print(f"Collecting baseline GPS for {duration_sec}s...")
        while time.time() - start_time < duration_sec:
            while self.stream.in_waiting == 0:
                time.sleep(0.01)
            data = self.get_GPS()
            if data:
                data_list.append(data)
        return data_list

    def average_gps(self, data_list):
        lats = [d["lat"] for d in data_list]
        lons = [d["lon"] for d in data_list]
        heights = [d["height"] for d in data_list]

        avg_lat = np.mean(lats)
        avg_lon = np.mean(lons)
        avg_height = np.mean(heights)

        self.last_GPS = {"lat": avg_lat, "lon": avg_lon, "height": avg_height}
        return self.last_GPS
    
    def close(self):
        self.stream.close()