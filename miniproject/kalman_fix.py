import numpy as np
import math as math
import matplotlib.pyplot as plt

GAZEBO_ODOM = 20
ENCODER_POS = 5
WHEEL_ENCODER_POS = 1
VEL_STD = 10
PSI_STD = 1
INDOOR_GPS_STD = 20
AMCL_STD = 10
ACCEL_STD = 10

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

""" class VehicleState:
    def __init__(self,x=0,y=0,yaw=0,yaw_rate=0):
        self.x=x
        self.y = y
        self.yaw = yaw
        self.yaw_rate = yaw_rate """

class Kalman_filter:
    def __init__(self,state_init,vx=0,vy=0,vz=0,yaw_rate=0):
        """ self.x = pose[0]
        self.y = pose[1]
        self.z = pose[2] """
        #self.yaw = pose[3]
        self.state = state_init.reshape(4,1)
        self.predicted_state = np.zeros((4,1))
        self.measured_state = np.zeros((4,1))
        self.cov = np.array([[25, 0, 0, 0], [0,25,0, 0], [0,0, 1, 0], [0, 0, 0, 100]])
        self.vx = vx
        self.vy = vy
        self.vz = vz
        #self.yaw_rate = yaw_rate
        self.is_initialized = True
        
    
    
    def getState(self):  
        return self.state
    
    def getCovariance(self):
        return self.cov
    """ If the vehicle is initialized, it calls the getState
    to get the vehicle's current state, 
    which is a tuple of four values representing 
    the vehicle's position and velocity."""       


    def getVehicleState(self):                              
        if self.is_initialized:
            state = self.getState()
            psi = math.atan2(state[3], state[2])     #returns the angle between the x-axis and the vector (state[3], state[2]),                                         #which represents the vehicle's velocity in the x-y plane
            V = math.sqrt(state[2]**2 + state[3]**2)  #returns the magnitude of velocity vector
            self.state = [state[0], state[1], psi, V]   
            return self.state
        return self.getVehicleState(self)
 

    
    
    def get_vehicle_state_posecovariance(self):
        pos_cov = np.zeros((2,2))
        cov = self.getCovariance()
        if self.is_initialized and cov.size != 0:
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
    def predictionstep(self, dt):                 #predicts nxt step based on current state and time.
        if not self.is_initialized and self.Init_on_first_prediction:
            state = np.zeros((4,1))
            cov = np.zeros((4,4))
            state[0] = self.state[0]
            state[1] = self.state[1]
            state[2] = self.state[2]
            state[3] = self.state[3]
            cov[0,0] = ENCODER_POS**2
            cov[1,1] = ENCODER_POS**2
            cov[2,2] = PSI_STD**2
            cov[3,3] = PSI_STD**2
            #self.state = state
            self.cov = cov
        elif self.is_initialized:
            state = self.getState()
            cov = self.getCovariance()
            x = state[0]
            y = state[1]
            yaw = state[2]
            v = self.vx
            yaw_v = dt * state[3]
            yaw = yaw if np.isnan(yaw_v) else wrap_angle(yaw + yaw_v)
            x_new = x + dt * v * np.cos(yaw)
            y_new = y + dt * v * np.sin(yaw)
            psi_new = yaw
            v_new = (x_new - x) / dt
            state = np.array([x_new, y_new, psi_new, v_new]).reshape(4,1)
            print(state)
            exit()
            #self.setState(state)

            F = np.zeros((4,4))     #state transition matrix generation     #updates the state and cov using motion model and noise model.
            F = [[1, 0, (-dt*v*np.sin(yaw)), (dt*np.cos(yaw))],
                [0, 1, (dt*v*np.cos(yaw)), (dt*np.sin(yaw))],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]

            Q = np.zeros((4,4))  #Q= noise covariance matrix
            Q[0,0] = ENCODER_POS**2
            Q[1,1] = ENCODER_POS**2
            Q[2,2] = ENCODER_POS**2
            Q[3,3] = ACCEL_STD**2

            #multiplies the old covariance matrix by the system's state transition matrix
            #calculates the transpose of the system's state transition matrix.
            #adds the process noise covariance matrix to thepredicted covariance matrix. The result is the new covariance matrix of the system's state estimate.
            cov = F @ cov @ np.transpose(F) + Q   #cov= old covariance matrix
                                                
            self.state = state
            self.cov = cov
                        
    """first checks if the Kalman filter has been initialized, 
    and if so, retrieves the current state and covariance."""          
    def handle_wheelenco_measurement(self, whlpose, dt):
        if self.is_initialized:
            state = self.getState()
            cov = self.getCovariance()
        
            z = np.zeros(4)          #creates a measurement vector z using the wheel pose measurement and the current state
            z[0], z[1], z[2], z[3] = whlpose[0], whlpose[1], whlpose[2], 0
        
            z_hat = np.zeros(4)         #creates a predicted measurement matrix z_hat
            z_hat[0], z_hat[1], z_hat[2], z_hat[3] = state[0], state[1], state[2], 0
        
            H = np.zeros((4, 4))        #H=jacobian of measurement function wrt state
            H[0, 0], H[1, 1], H[2,2], H[3,3] = 1, 1, 1, 1
        
            R = np.zeros((4, 4))       #measurement noise matrix
            R[0, 0], R[1, 1], R[2,2]= WHEEL_ENCODER_POS ** 2, WHEEL_ENCODER_POS ** 2, WHEEL_ENCODER_POS ** 2,
        
            y = z - z_hat                       #y=measurement residual
            print(H @ cov)
            S = H @ cov @ H.T + R               #S=innovation covariance matrix
            
            K = cov @ H.T @ np.linalg.inv(S)   #K=kalman gain
            print(K, y)
            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov
            print("state", state, 'covariance', cov)
            self.state = state
            self.cov = cov
            return True



    def handle_AMCL_measurement(self, amcl_pose, dt):
        if self.is_initialized:
            state = self.getState()
            cov = self.getCovariance()
        
            z = np.zeros(2)
            z[0] = amcl_pose[0]
            z[1] = amcl_pose[1]
        
            z_hat = np.zeros(2)
            z_hat[0] = state[0]
            z_hat[1] = state[1]
        
            H = np.zeros((2, 4))
            H[0, 0] = 1
            H[1, 1] = 1
        
            R = np.zeros((2, 2))
            R[0, 0] = AMCL_STD**2
            R[1, 1] = AMCL_STD**2
        
            y = z - z_hat
            S = np.dot(np.dot(H, cov), H.T) + R
            K = np.dot(np.dot(cov, H.T), np.linalg.inv(S))
        
            state = state + np.dot(K, y)
            cov = np.dot((np.identity(4) - np.dot(K, H)), cov)
            print("state", state, 'covariance', cov)
            self.state = state
            self.cov = cov
            return True
    
    def handle_indoorgps_measurement(self,gps_pose,dt):
            if self.is_initialized:
                state = self.getState()
                cov = self.getCovariance()
                
                z = np.zeros(2)
                z[0], z[1] = gps_pose[0], gps_pose[1]
                
                z_hat = np.zeros(2)
                z_hat[0], z_hat[1] = state[0], state[1]
                
                H = np.zeros((2,4))
                H[0,0], H[0,1], H[1,0], H[1,1] = 1, 0, 0, 1
                
                R = np.zeros((2,2))
                R[0,0] = INDOOR_GPS_STD**2
                R[1,1] = INDOOR_GPS_STD**2
                
                y = z - z_hat
                S = H @ cov @ H.T + R
                K = cov @ H.T @ np.linalg.inv(S)
                
                state = state + K @ y
                cov = (np.identity(4) - K @ H) @ cov
                #print("state", state, 'covariance', cov)
                self.state = state
                self.cov = cov
                return True
    
    
    def handle_GazeboOdomMeasurement(self, gazebo_pos, dt):
        if self.is_initialized:
            state = self.getState()
            cov = self.getCovariance()
            
            z = np.zeros(2)
            z[0], z[1] = gazebo_pos[0], gazebo_pos[1]
            
            z_hat = np.zeros(2)
            z_hat[0], z_hat[1] = state[0], state[1]
            
            H = np.zeros((2,4))
            H[0,0], H[0,1], H[1,0], H[1,1] = 1, 0, 0, 1
            
            R = np.zeros((2,2))
            R[0,0] = GAZEBO_ODOM**2 #constant
            R[1,1] = GAZEBO_ODOM**2
            
            y = z - z_hat
            S = H @ cov @ H.T + R
            K = cov @ H.T @ np.linalg.inv(S)
            
            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov
            print("state", state, 'covariance', cov)
            self.state = state
            self.cov = cov
            return True
        
    
if __name__ == '__main__':
    pose_ini =np.array([0, 0, 0, 0])

    kf = Kalman_filter(pose_ini,0,0,0,0.1)
    #pose_ini = np.array((0.5, 0.2, 0, 0.05))
    dt = 0.1
    pose_estimated_array = []
    x_estimated = []
    y_estimated = []
    yaw_extimated = []
    t = np.arange(0,10,0.1)

    for i in range(100):
        kf.predictionstep(dt)
        pose_estimated = kf.getState()
        pose_estimated_array.append(pose_estimated)
        
    for i in range(len(pose_estimated_array)):
        pose = pose_estimated_array[i]
        x_estimated.append(pose[0])
        y_estimated.append(pose[1])
        yaw_extimated.append(pose[3])
    plt.plot(t,yaw_extimated)
    plt.show()
    plt.plot(x_estimated,y_estimated)
    plt.show()
    print(pose_estimated_array[50])
    
     # print(kf.handle_AMCL_measurement(pose,dt))