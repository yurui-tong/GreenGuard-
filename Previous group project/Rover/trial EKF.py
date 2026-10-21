import numpy as np

class Ekal_fil:
    def __init__(self):
        from filterpy.kalman import KalmanFilter
        # State: [x, y, yaw, vx, vy, vyaw]
        # We set dim_z=3 initially, but we will change it dynamically
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.eye(6)
        self._x= self.kf.x[0]
        self._y= self.kf.x[1]
        self._psi=self.kf.x[2]
        self._vx= self.kf.x[3]
        self._vy= self.kf.x[4]
        self._omega= self.kf.x[5]
        # H for updating Position (x,y) - Maps state to [x, y]
        # Shape: (2, 6)
        self.H_pos = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]])
       
        # H for updating Heading (yaw) - Maps state to [yaw]
        # Shape: (1, 6)
        self.H_yaw = np.array([
            [0, 0, 1, 0, 0, 0]])

        self.kf.P *= 5
        self.kf.Q = np.eye(6) * 0.05
       
        # Measurement noise
        self.R_pos = np.diag([0.5, 0.5])   # GPS Noise (meters)
        self.R_yaw = np.array([[0.05]])    # IMU Noise (radians)
        
    def motion_model(self, dt):
        return np.array([[self._x+self._vx/self._omega * (np.cos(self._psi + self._omega* dt)- np.cos(self._psi))],
                         [self._y+ self._vy/self._omega (np.sin(self._psi)- np.sin(self._psi + self._omega *dt))],
                         [self._psi+ self._omega*dt ],
                         [self._vx],
                         [self._vy],
                         [self._omega]])
        
    def motion_jacobian(self, dt):
        pass

    def predict(self, dt): # inspired by https://github.com/Janudis/Extended-Kalman-Filter-GPS_IMU
        """
        Time Update: Projects the state ahead by dt
        """  
        # Read the current state before changing it.
        x_old = self.kf.x.copy()

        # Evaluate both using that same state.
        J = self.motion_jacobian(x_old, dt)
        x_next = self.motion_model(x_old, dt)

        # Predict uncertainty and state.
        self.kf.P = J @ self.kf.P @ J.T + self.kf.Q
        self.kf.x = x_next

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