import numpy as np
import matplotlib.pyplot as plt
import csv

class data_processing:
    def __init__(self, data): 
        ## Unpack and convert lists to arrays
        gps_data   = np.asarray(data["GPS_rover_data_list"])
        GPS_cor_data = np.asarray(data["GPS_cor_list"])
        self.yaw_data   = np.asarray(data["yaw_list"])
        cor_data   = np.asarray(data["cor_list"])
        state_data = np.asarray(data["state_list"])
        self.time_data  = np.asarray(data["time_list"])

        ## Unpack columns
        self.gps_x,   self.gps_y   = gps_data[:, 0], gps_data[:, 1]
        self.gps_cor_x, self.gps_cor_y = GPS_cor_data[:, 0], GPS_cor_data[:, 1]
        self.cor_x,   self.cor_y, self.cor_z = cor_data[:, 0], cor_data[:, 1], cor_data[:, 2]
        self.state_x, self.state_y, self.state_yaw = state_data[:, 0], state_data[:, 1], state_data[:, 2]
        self.state_vx, self.state_vy, self.state_yaw_rate = state_data[:, 3], state_data[:, 4], state_data[:, 5]


    def plots(self):
        plt.figure(figsize=(8, 6))
        valid_gps = ~np.isnan(self.gps_x)
        valid_gps_cor = ~np.isnan(self.gps_cor_x)

        plt.plot(self.gps_x[valid_gps], self.gps_y[valid_gps], color="blue",  label="GPS",marker='o',linestyle='None')
        plt.plot(self.gps_cor_x[valid_gps_cor], self.gps_cor_y[valid_gps_cor], color="red",   label="Corrected",marker='x',linestyle='None')
        plt.plot(self.state_x, self.state_y, color="green", label="Kalman")

        plt.title("Rover Trajectory Comparison")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)

        plt.show(block=True)

    def save_data_to_csv(self, filename="test_data.csv"):
        if input("Save data to CSV? (yes/no): ").strip().lower() == "yes":
            filename = input("Insert filename (without extension): ").strip()
            if not filename:
                print("No filename provided, saving as 'test_data.csv'.")
                filename_with_extensions = "test_data.csv"
            else:
                filename_with_extensions = filename + ".csv"
            
            ## Ensure all arrays have the same length
            L = min(len(self.gps_x), len(self.cor_x), len(self.state_x))

            with open(filename_with_extensions, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time [s]","x_pos [m]", "y_pos [m]", "yaw [rad]",
                                "x_cor [m]", "y_cor [m]", "z_cor [m]",
                                "x_pos_kf [m]", "y_pos_kf [m]", "yaw_kf [rad]","vx_kf [m/s]","vy_kf [m/s]","yaw_rate_kf [rad/s]"])
                for i in range(L):
                    writer.writerow([self.time_data[i], self.gps_x[i], self.gps_y[i], self.yaw_data[i],
                                    self.cor_x[i], self.cor_y[i], self.cor_z[i],
                                    self.state_x[i], self.state_y[i], self.state_yaw[i], self.state_vx[i], self.state_vy[i], self.state_yaw_rate[i]])
            print(f"Data saved as {filename_with_extensions}")
