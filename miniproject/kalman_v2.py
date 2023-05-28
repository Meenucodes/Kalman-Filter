import numpy as np


def wrap_angle(angle):
        #check if given angle is greater than 360 degree (mode operation),
        #if angle is less than -pi angle = angle + 2*pi
        #if angle is greater than pi angle = angle - 2*pi
    pass

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
        state = get_state()
        return state
    
    
    
    
    def get_vehicle_state_posecovariance():
        pos_cov = np.zeros((2,2))
        cov = get_covariance()
        if is_intialised() and cov.size != 0:
            pos_cov[0,0] = cov[0,0]
            pos_cov[0,1] = cov[0,1]
            pos_cov[1,0] = cov[1,0]
            pos_cov[1,1] = cov[1,1]
        return pos_cov

    
    def predictionstep(self, pose, dt):                 #predicts nxt step based on current state and time.
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

            F = np.zeros((4,4))     #state transition matrix generation     #updates the state and cov using motion model and noise model.
            F = [[1, 0, (-dt*v*sin(yaw)), (dt*cos(yaw))],
                [0, 1, (dt*v*cos(yaw)), (dt*sin(yaw))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]

            Q = np.zeros((4,4))  #Q= noise covariance matrix
            Q[0,0] = enc_pos**2
            Q[1,1] = enc_pos**2
            Q[2,2] = enc_pos**2
            Q[3,3] = accel**2

            #multiplies the old covariance matrix by the system's state transition matrix
            #calculates the transpose of the system's state transition matrix.
            #adds the process noise covariance matrix to thepredicted covariance matrix. The result is the new covariance matrix of the system's state estimate.
            cov = F @ cov @ np.transpose(F) + Q   #cov= old covariance matrix
                                                
            setState(state)
            setCovariance(cov)
                        
                
    def handle_wheelenco_measurement(wheel_pose,dt):
        pass
    
    def handle_amcl_measurement(amcl_pose,dt):
        pass
    
    def handle_indoorgps_measurement(gps_pose,dt):
            if self.isIntialized():
                state = self.getState()
                cov = self.getCovariance()
                z = np.zeros(2)
                z[0], z[1] = pose1GPS.x, pose1GPS.y
                z_hat = np.zeros(2)
                z_hat[0], z_hat[1] = state[0], state[1]
                H = np.zeros((2,4))
                H[0,0], H[0,1], H[1,0], H[1,1] = 1, 0, 0, 1
                R = np.zeros((2,2))
                R[0,0] = indoor_gps**2
                R[1,1] = indoor_gps**2
                y = z - z_hat
                S = H @ cov @ H.T + R
                K = cov @ H.T @ np.linalg.inv(S)
                state = state + K @ y
                cov = (np.identity(4) - K @ H) @ cov
                self.setState(state)
                self.setCovariance(cov)
    
    
    def handle_gazebo_odom_measurement(gazebo_pose,dt):
        pass
    
    def is_kf_initialized():
        return self.is_initialized
        
print("hello world")

kf =Kalman_filter()