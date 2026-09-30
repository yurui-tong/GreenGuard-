import numpy as np

class kal_fil:
    def __init__(self):
        from filterpy.kalman import KalmanFilter
        # State: [x, y, yaw, vx, vy, vyaw]
        # We set dim_z=3 initially, but we will change it dynamically
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.eye(6)
       
        # H for updating Position (x,y) - Maps state to [x, y]
        # Shape: (2, 6)
        self.H_pos = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ])
       
        # H for updating Heading (yaw) - Maps state to [yaw]
        # Shape: (1, 6)
        self.H_yaw = np.array([
            [0, 0, 1, 0, 0, 0]
        ])

        self.kf.P *= 5
        self.kf.Q = np.eye(6) * 0.05
       
        # Measurement noise
        self.R_pos = np.diag([0.5, 0.5])   # GPS Noise (meters)
        self.R_yaw = np.array([[0.05]])    # IMU Noise (radians)

    def predict(self, dt):
        """
        Time Update: Projects the state ahead by dt
        """
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt    
        self.kf.predict()

    def update_heading(self, yaw):
        """
        Fast Measurement Update: Uses IMU data only
        """
        # Temporarily set dim_z to 1 for this update
        self.kf.dim_z = 1
        self.kf.update(np.array([yaw]), R=self.R_yaw, H=self.H_yaw)

    def update_position(self, pos_x, pos_y):
        """
        Slow Measurement Update: Uses GPS data only
        """
        # Temporarily set dim_z to 2 for this update
        z = np.array([pos_x, pos_y])
        self.kf.dim_z = 2
        self.kf.update(z, R=self.R_pos, H=self.H_pos)