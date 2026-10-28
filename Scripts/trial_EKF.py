import numpy as np

class Ekal_fil:
    def __init__(self):
        from filterpy.kalman import KalmanFilter
        # State: [x, y, yaw, vx, vy, vyaw]
        # We set dim_z=3 initially, but we will change it dynamically
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

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
        v = self._vx*np.cos(self._psi) + self._vy*np.sin(self._psi)
        theta = self._psi + self._omega*dt


        
        if abs(self._omega * dt) < 1e-6:
            return np.array([
                [self._x + v * np.cos(self._psi) * dt],
                [self._y + v * np.sin(self._psi) * dt],
                [theta],
                [v * np.cos(theta)],
                [v * np.sin(theta)],
                [self._omega],], dtype=float)
            
        else:
            A = (np.sin(theta) - np.sin(self._psi)) / self._omega
            B = (np.cos(self._psi) - np.cos(theta)) / self._omega
            return np.array([[self._x + v*A],
                         [self._y + v*B],
                         [theta],
                         [v*np.cos(theta)],
                         [v*np.sin(theta)],
                         [self._omega],], dtype=float)
        
    def motion_jacobian(self, dt):
        v = self._vx * np.cos(self._psi) + self._vy * np.sin(self._psi)
        dv = -self._vx * np.sin(self._psi) + self._vy * np.cos(self._psi)

        theta = self._psi + self._omega * dt
        A = np.sin(theta) - np.sin(self._psi)
        B = np.cos(self._psi) - np.cos(theta)
        
        
        if abs(self._omega * dt) < 1e-6:
            c = np.cos(self._psi)
            s = np.sin(self._psi)

            return np.array([
                [1, 0, dt*(dv*c - v*s), dt*c*c, dt*s*c, -0.5*v*s*dt**2],
                [0, 1, dt*(dv*s + v*c), dt*c*s, dt*s*s,  0.5*v*c*dt**2],
                [0, 0, 1,               0,      0,       dt],
                [0, 0, dv*c - v*s,       c*c,    s*c,    -v*s*dt],
                [0, 0, dv*s + v*c,       c*s,    s*s,     v*c*dt],
                [0, 0, 0,               0,      0,       1],], dtype=float)
        
        else:
            return np.array([
        [1, 0, (dv * A + v * (np.cos(theta) - np.cos(self._psi)))
            / self._omega,
            np.cos(self._psi) * A / self._omega,
            np.sin(self._psi) * A / self._omega,
            v * dt * np.cos(theta) / self._omega
            - v * A / self._omega**2,
        ],
        [
            0, 1,
            (dv * B + v * (np.sin(theta) - np.sin(self._psi)))
            / self._omega,
            np.cos(self._psi) * B / self._omega,
            np.sin(self._psi) * B / self._omega,
            v * dt * np.sin(theta) / self._omega
            - v * B / self._omega**2,
        ],
        [0, 0, 1, 0, 0, dt],
        [
            0, 0,
            dv * np.cos(theta) - v * np.sin(theta),
            np.cos(self._psi) * np.cos(theta),
            np.sin(self._psi) * np.cos(theta),
            -v * dt * np.sin(theta),
        ],
        [
            0, 0,
            dv * np.sin(theta) + v * np.cos(theta),
            np.cos(self._psi) * np.sin(theta),
            np.sin(self._psi) * np.sin(theta),
            v * dt * np.cos(theta),
        ],
        [0, 0, 0, 0, 0, 1],], dtype=float)

    def predict(self, dt):
    # Refresh your variables from the current filter state.
        (self._x, self._y, self._psi, self._vx, self._vy, self._omega) = self.kf.x.ravel()

        J = self.motion_jacobian(dt)
        x_next = self.motion_model(dt)
        
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