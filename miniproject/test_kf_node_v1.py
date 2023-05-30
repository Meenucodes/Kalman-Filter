import rospy
import random
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from marvelmind_nav.msg import hedge_pos
from amr_cloud_pkg.msg import supervizTx
from amr_common_pkg.constant import *
from amr_common_pkg.parameter import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from kalmanFilter import KalmanFilter

class ExtendedKalmanTest:
    def __init__(self):
        self.node_handle = rospy.init_node('ekf_test')
        self.publish_rate = 10
        self.indoorGPSReceived = False
        self.mtrPoseReceived = False
        self.amclReceived = False
        self.gazeboOdomReceived = False
        self.cmdVelReceived = False
        self.noiseCounter = 0
        self.xNoiseFactor = 0.0
        self.yNoiseFactor = 0.0
        self.mean = 0.0
        self.standardDeviation = 0.05
        self.xNoise = 0.0
        self.yNoise = 0.0
        self.yawNoise = 0.0
        self.generator = random.Random()
        self.distribution = np.random.normal(self.mean, self.standardDeviation)
        self.hedgePose = hedge_pos()
        self.supervisorTx = supervizTx()
        self.amclPose = PoseWithCovarianceStamped()
        self.cmdVel = Twist()
        self.gazeboOdom = Odometry()
        self.gazeboNoisyPose = Pose()
        self.sub_hedge_pose_tpc = rospy.Subscriber('hedge_pos', hedge_pos, self.hedge_pose_callback)
        self.sub_supervisor_tx_tpc = rospy.Subscriber('supervisor_tx', supervizTx, self.supervisor_tx_callback)
        self.sub_amcl_pose_tpc = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.sub_gazebo_odom_tpc = rospy.Subscriber('gazebo_odom', Odometry, self.gazebo_odom_callback)
        self.sub_cmd_vel_tpc = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.pub_kalman_pose_tpc = rospy.Publisher('kalman_pose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_kalman_predicted_pose_tpc = rospy.Publisher('kalman_predicted_pose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_noisy_gazebo_pose_tpc = rospy.Publisher('noisy_gazebo_pose', Pose, queue_size=10)
        

    def __init__(self):
        self.distribution = (mean, standardDeviation)
        self.sub_hedge_pose_tpc = rospy.Subscriber("hedge_pos", marvelmind_nav.hedge_pos, self.callback_indoor_gps_tpc, queue_size=Parameter.QUEUE_SIZE)
        self.sub_supervisor_tx_tpc = rospy.Subscriber("superviz_txinfo", amr_cloud_pkg.supervizTx, self.callback_supervisor_tx_tpc, queue_size=Parameter.QUEUE_SIZE)
        self.sub_amcl_pose_tpc = rospy.Subscriber("amcl_pose", geometry_msgs.PoseWithCovarianceStamped, self.callback_amcl_pose_tpc, queue_size=Parameter.QUEUE_SIZE)
        self.sub_gazebo_odom_tpc = rospy.Subscriber("odom", nav_msgs.Odometry, self.callback_gazebo_odom_tpc, queue_size=Parameter.QUEUE_SIZE)
        self.sub_cmd_vel_tpc = rospy.Subscriber("cmd_vel", geometry_msgs.Twist, self.callback_cmd_vel_tpc, queue_size=Parameter.QUEUE_SIZE)
        self.pub_kalman_pose_tpc = rospy.Publisher("kalman_pose_tpc", geometry_msgs.Pose, queue_size=Parameter.QUEUE_SIZE)
        self.pub_kalman_predicted_pose_tpc = rospy.Publisher("kalman_predicted_pose_tpc", geometry_msgs.Pose, queue_size=Parameter.QUEUE_SIZE)
        self.pub_noisy_gazebo_pose_tpc = rospy.Publisher("noisy_gazebo_pose_tpc", geometry_msgs.Pose, queue_size=Parameter.QUEUE_SIZE)
        self.publish_rate = Parameter.PUBLISH_RATE
        
    def run(self):
        pass
    
    def callback_indoor_gps_tpc(self, msg):
        pass
    
    def callback_supervisor_tx_tpc(self, msg):
        pass
    
    def callback_amcl_pose_tpc(self, msg):
        pass
    
    def callback_gazebo_odom_tpc(self, msg):
        pass
    
    def callback_cmd_vel_tpc(self, msg):
        pass
    
    def generate_normal_dist_noisy_odom(self, vehPose):
        pass
    
    
    def callback_indoor_gps_tpc(msg):
        global hedgePose, indoorGPSReceived
    hedgePose = msg
    indoorGPSReceived = True

    def callback_supervisor_tx_tpc(msg):
        global supervisorTx, mtrPoseReceived
        supervisorTx = msg
        mtrPoseReceived = True

    def callback_amcl_pose_tpc(msg):
        global amclPose, amclReceived
        amclPose = msg
        amclReceived = True

    def callback_gazebo_odom_tpc(msg):
        global gazeboOdom, gazeboOdomReceived
        gazeboOdom = msg
        gazeboOdomReceived = True

    def callback_cmd_vel_tpc(msg):
        global cmdVel, cmdVelReceived
        cmdVel = msg
        cmdVelReceived = True

    def run():
        global publish_rate
        rate = rospy.Rate(publish_rate)
        ekf = ExtendedKalmanFilter()
        vehPose = VehiclePose()
        kalmanPose = geometry_msgs.msg.Pose()
        kalmanPredictedPose = geometry_msgs.msg.Pose()
        vehState = np.zeros(4)
        vehPredictedState = np.zeros(4)
        
    while rospy.ok():
        vehState = [0, 0, 0, 0]
    if cmdVelReceived:
        cmdVelReceived = False
        vehPose.x = vehPredictedState[0]
        vehPose.y = vehPredictedState[1]
        vehPose.yaw = vehPredictedState[2]
        vehPose.v_x = gazeboOdom.twist.twist.linear.x
        vehPose.yaw_r = gazeboOdom.twist.twist.angular.z
        ekf.predictionStep(vehPose, 0.1)
        if ekf.isInitialised():
            vehPredictedState = ekf.getVehicleState()
            kalmanPredictedPose.position.x = vehPredictedState[0]
            kalmanPredictedPose.position.y = vehPredictedState[1]
            kalmanPredictedPose.orientation.z = vehPredictedState[2]
    if indoorGPSReceived:
        indoorGPSReceived = False
        vehPose.x = hedgePose.x_m
        vehPose.y = hedgePose.y_m
    if amclReceived:
        amclReceived = False
        vehPose.x = amclPose.pose.pose.position.x
        vehPose.y = amclPose.pose.pose.position.y
        vehPose.yaw = amclPose.pose.pose.orientation.z
        ekf.handleAMCLMeasurement(vehPose, 0.1)
    if gazeboOdomReceived:
        gazeboOdomReceived = False
        vehPose.x = gazeboOdom.pose.pose.position.x
        vehPose.y = gazeboOdom.pose.pose.position.y
        vehPose.yaw = gazeboOdom.pose.pose.orientation.z
        vehPose = generate_normal_dist_noisy_odom(vehPose)
        gazeboNoisyPose.position.x = vehPose.x
        gazeboNoisyPose.position.y = vehPose.y
        ekf.handleGazeboOdomMeasurement(vehPose, 0.1)
    if ekf.isInitialised():
        vehState = ekf.getVehicleState()
    kalmanPose.position.x = vehState[0]
    kalmanPose.position.y = vehState[1]
    kalmanPose.orientation.z = vehState[2]
    pub_kalman_pose_tpc.publish(kalmanPose)
    pub_kalman_predicted_pose_tpc.publish(kalmanPredictedPose)
    pub_noisy_gazebo_pose_tpc.publish(gazeboNoisyPose)
    rospy.spinOnce()
    rate.sleep()

    def generate_normal_dist_noisy_odom(self, vehPose):
        noisyPose = VehiclePose()
        
        if self.noiseCounter == 50:
            self.noiseCounter = 0
            self.xNoise = self.distribution
            self.yNoise = self.distribution
            self.yawNoise = self.distribution
            self.xNoiseFactor = self.xNoise / 10
            self.yNoiseFactor = self.yNoise / 10
        else:
            self.xNoise = 0.0
            self.yNoise = 0.0
        
        noisyPose.x = vehPose.x + self.xNoise
        noisyPose.y = vehPose.y + self.yNoise
        noisyPose.yaw = vehPose.yaw + self.yawNoise
        self.gazeboNoisyPose.position.x = noisyPose.x
        self.gazeboNoisyPose.position.y = noisyPose.y
        
        return noisyPose

if __name__ == '__main__':
    rospy.init_node('test_kalman_filter_node')
    node = ExtendedKalmanTest()
    node.run()
    rospy.spin()
