import numpy as np


def wrap_angle(angle):
    angle = angle % (2 * math.pi) 

    if angle < 0:
        angle += 2 * math.pi
    elif angle > 2 * math.pi:
        angle -= 2 * math.pi

    return angle
   

class Kalman_filter:
    def __init__(self,x=0,y=0,z=0,yaw=0,vx=0,vy=0,vz=0,yaw_rate=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw_rate = yaw_rate
        self.is_initialized = False
        pass
    
    def get_vehicle_state():  

        pass
    
    
    
    
    def get_vehicle_state_posecovariance():
        pass
    
    def predictionstep(self, pose, dt):
        if not self.isInitialised() and self.Init_on_first_prediction:
            state = np.zeros(4)
            cov = np.zeros((4,4))
            state[0] = pose.x
            state[1] = pose.y
            state[2] = pose.yaw
            state[3] = pose.v_x
            cov[0,0] = enc_pos**2
            cov[1,1] = enc_pos**2
            cov[2,2] = init_psi**2
            cov[3,3] = init_vel**2
            self.setState(state)
            self.setCovariance(cov)
        elif self.isInitialised():
            state = self.getstate()
            cov = self.getcovariance()
            x = state[0]
            y = state[1]
            yaw = state[2]
            v = pose.v_x
            yaw_v = dt * pose.yaw_r
            yaw = yaw if np.isnan(yaw_v) else wrap_angle(yaw + yaw_v)
            x_new = x + dt * v * np.cos(yaw)
            y_new = y + dt * v * np.sin(yaw)
            psi_new = yaw
            v_new = (x_new - x) / dt
            state = np.array([x_new, y_new, psi_new, v_new])
            #self.setState(state)

            F = np.zeros((4,4))     #F matrix generation
            F = [[1, 0, (-dt*v*sin(yaw)), (dt*cos(yaw))],
                [0, 1, (dt*v*cos(yaw)), (dt*sin(yaw))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]

            Q = np.zeros((4,4))
            Q[0,0] = enc_pos
            Q[1,1] = enc_pos
            Q[2,2] = enc_pos
            Q[3,3] = accel

            cov = F @ cov @ np.transpose(F) + Q

            setState(state)
            setCovariance(cov)
                        
                
    def handle_wheelenco_measurement(self, whlpose, dt):
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()
        
            z = np.zeros(4)
            z[0], z[1], z[2], z[3] = whlpose.x, whlpose.y, whlpose.yaw, 0
        
            z_hat = np.zeros(4)
            z_hat[0], z_hat[1], z_hat[2], z_hat[3] = state[0], state[1], state[2], 0
        
            H = np.zeros((4, 4))
            H[0, 0], H[1, 1], H[2, 2], H[3, 3] = 1, 1, 1, 0
        
            R = np.zeros((4, 4))
            R[0, 0], R[1, 1], R[2, 2] = WHL_ENC_POS_STD ** 2, WHL_ENC_POS_STD ** 2, WHL_ENC_POS_STD ** 2
        
            y = z - z_hat
            S = H @ cov @ H.T + R
            K = cov @ H.T @ np.linalg.inv(S)
        
            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov
        
            self.setState(state)
            self.setCovariance(cov)


    
    def handle_AMCL_measurement(self, amcl_pose, dt):
        if self.is_initialised():
            state = self.get_state()
            cov = self.get_covariance()
        
            z = np.zeros(2)
            z[0] = amcl_pose.x
            z[1] = amcl_pose.y
        
            z_hat = np.zeros(2)
            z_hat[0] = state[0]
            z_hat[1] = state[1]
        
            H = np.zeros((2, 4))
            H[0, 0] = 1
            H[1, 1] = 1
        
            R = np.zeros((2, 2))
            R[0, 0] = AMCL_STD * AMCL_STD
            R[1, 1] = AMCL_STD * AMCL_STD
        
            y = z - z_hat
            S = np.dot(np.dot(H, cov), H.T) + R
            K = np.dot(np.dot(cov, H.T), np.linalg.inv(S))
        
            state = state + np.dot(K, y)
            cov = np.dot((np.identity(4) - np.dot(K, H)), cov)
        
            self.set_state(state)
            self.set_covariance(cov)

    
    def handle_indoorgps_measurement(gps_pose,dt):
        pass
    def handle_gazebo_odom_measurement(gazebo_pose,dt):
        pass
    
    def is_kf_initialized():
        return self.is_initialized
        
print("hello world")

kf =Kalman_filter()
