import numpy as np
import math as math


def wrap_angle(angle):
        #check if given angle is greater than 360 degree (mode operation),
        #if angle is less than -pi angle = angle + 2*pi
        #if angle is greater than pi angle = angle - 2*pi
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
            state = getState()
            return state 
 
    """ If the vehicle is initialized, it calls the getState
    to get the vehicle's current state, 
    which is a tuple of four values representing 
    the vehicle's position and velocity."""       

    """ def getVehicleState(self):                              
        if self.is_initialized:
            state = self.getState()
            psi = math.atan2(state[3], state[2])     #returns the angle between the x-axis and the vector (state[3], state[2]),
                                                        #which represents the vehicle's velocity in the x-y plane
            V = math.sqrt(state[2]**2 + state[3]**2)    #returns the magnitude of velocity vector
            return State(state[0], state[1], psi, V)
        return VehicleState()
 """

    
    
    def get_vehicle_state_posecovariance():
        pos_cov = np.zeros((2,2))
        cov = getCovariance()
        if is_intialized() and cov.size != 0:
            pos_cov[0,0] = cov[0,0]
            pos_cov[0,1] = cov[0,1]
            pos_cov[1,0] = cov[1,0]
            pos_cov[1,1] = cov[1,1]
        return pos_cov

    """first checks if the filter has been initialized. If it has, it gets the current state and covariance matrices. 
    If not, it initializes them with the current pose and some default values.
    multiplies the old covariance matrix by the state transition matrix, calculates 
    the transpose of the state transition matrix, 
    and adds the process noise covariance matrix to the predicted covariance matrix."""
    def predictionstep(self, pose, dt):                 #predicts nxt step based on current state and time.
        if not self.is_initialized() and self.Init_on_first_prediction:
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
            F = [[1, 0, (-dt*v*np.sin(yaw)), (dt*np.cos(yaw))],
                [0, 1, (dt*v*np.cos(yaw)), (dt*np.sin(yaw))],
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
                        
    """first checks if the Kalman filter has been initialized, 
    and if so, retrieves the current state and covariance."""          
    def handle_wheelenco_measurement(self, whlpose, dt):
        if self.is_initialized:
            state = self.getState()
            cov = self.getCovariance()
        
            z = np.zeros(4)          #creates a measurement vector z using the wheel pose measurement and the current state
            z[0], z[1], z[2], z[3] = whlpose.x, whlpose.y, whlpose.yaw, 0
        
            z_hat = np.zeros(4)         #creates a predicted measurement matrix z_hat
            z_hat[0], z_hat[1], z_hat[2], z_hat[3] = state[0], state[1], state[2], 0
        
            H = np.zeros((4, 4))        #H=jacobian of measurement function wrt state
            H[0, 0], H[1, 1], H[2, 2], H[3, 3] = 1, 1, 1, 0
        
            R = np.zeros((4, 4))       #measurement noise matrix
            R[0, 0], R[1, 1], R[2, 2] = WHL_ENC_POS_STD ** 2, WHL_ENC_POS_STD ** 2, WHL_ENC_POS_STD ** 2
        
            y = z - z_hat                       #y=measurement residual
            S = H @ cov @ H.T + R               #S=innovation covariance matrix
            K = cov @ H.T @ np.linalg.inv(S)   #K=kalman gain
        
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
            return True
 
    
    def handle_indoorgps_measurement(self,gps_pose,dt):
            if self.isIntialized():
                state = self.getState()
                cov = self.getCovariance()
                
                z = np.zeros(2)
                z[0], z[1] = gps_pose.x, gps_pose.y
                
                z_hat = np.zeros(2)
                z_hat[0], z_hat[1] = state[0], state[1]
                
                H = np.zeros((2,4))
                H[0,0], H[0,1], H[1,0], H[1,1] = 1, 0, 0, 1
                
                R = np.zeros((2,2))
                R[0,0] = gps_pose**2
                R[1,1] = gps_pose**2
                
                y = z - z_hat
                S = H @ cov @ H.T + R
                K = cov @ H.T @ np.linalg.inv(S)
                
                state = state + K @ y
                cov = (np.identity(4) - K @ H) @ cov
                
                self.setState(state)
                self.setCovariance(cov)
    
    
    def handleGazeboOdomMeasurement(self, gazebo_pos, dt):
        if self.isIntialized():
            state = self.getState()
            cov = self.getCovariance()
            
            z = np.zeros(2)
            z[0], z[1] = gazebo_pos.x, gazebo_pos.y
            
            z_hat = np.zeros(2)
            z_hat[0], z_hat[1] = state[0], state[1]
            
            H = np.zeros((2,4))
            H[0,0], H[0,1], H[1,0], H[1,1] = 1, 0, 0, 1
            
            R = np.zeros((2,2))
            R[0,0] = gazebo_pos**2
            R[1,1] = gazebo_pos**2
            
            y = z - z_hat
            S = H @ cov @ H.T + R
            K = cov @ H.T @ np.linalg.inv(S)
            
            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov
            self.setState(state)
            self.setCovariance(cov)

    
    def is_kf_initialized(self):
        return self.is_initialized
        
print("hello world")

kf = Kalman_filter()
print(kf.is_kf_initialized())
wheel_pose = np.array((0.5, 0.2, 0, 0.05))
dt =0.1
kf.handle_wheelenco_measurement(wheel_pose, dt)
print(kf.getVehicleState())