import matplotlib.pyplot as plt
import numpy as np

robot_params={"lr":1, "lf":1, "overhang":1, "track":2, "wheel_dia":1, "wheel_offset":0.25, "scale_x":1, "scale_y":1}
origin_to_front_middle = (robot_params["lf"]+robot_params["overhang"])*robot_params["scale_x"]
origin_to_front = robot_params["lf"]*robot_params["scale_x"]
origin_to_rear = robot_params["lr"]*robot_params["scale_x"]
origin_to_side = (robot_params["track"]/2)*robot_params["scale_y"]
origin_to_wheel_inner = (robot_params["track"]/2-robot_params["wheel_offset"])*robot_params["scale_y"]
origin_to_wheel_outer = (robot_params["track"]/2+robot_params["wheel_offset"])*robot_params["scale_y"]
wheel_radius = (robot_params["wheel_dia"]/2)*robot_params["scale_x"]


front_middle = np.array([[origin_to_front_middle],[0]])
front_left = np.array([[origin_to_front],[origin_to_side]])
rear_left = np.array([[-origin_to_rear],[origin_to_side]])
rear_right = np.array([[-origin_to_rear],[-origin_to_side]])
front_right = np.array([[origin_to_front],[-origin_to_side]])

robo_outline = np.column_stack((front_middle,front_left,rear_left,rear_right,front_right,front_middle))
 ## defining left wheels
lw_front_left = np.array([[wheel_radius],[origin_to_wheel_inner]])
lw_front_right = np.array([[wheel_radius],[origin_to_wheel_outer]])
lw_rear_left = np.array([[-wheel_radius],[origin_to_wheel_outer]])
lw_rear_right = np.array([[-wheel_radius],[origin_to_wheel_inner]])
left_wheel_outline = np.column_stack((lw_front_left,lw_front_right,lw_rear_left,lw_rear_right,lw_front_left))

 ## defining right wheel
rw_front_left = np.array([[wheel_radius],[-origin_to_wheel_outer]])
rw_front_right = np.array([[wheel_radius],[-origin_to_wheel_inner]])
rw_rear_left = np.array([[-wheel_radius],[-origin_to_wheel_inner]])
rw_rear_right = np.array([[-wheel_radius],[-origin_to_wheel_outer]])
right_wheel_outline = np.column_stack((rw_front_left,rw_front_right,rw_rear_left,rw_rear_right,rw_front_left))

##rotation matrix
theta = np.pi/4
R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) 
robo_outline_rotated = np.matmul(R,robo_outline)
left_wheel_outline_rotated = np.matmul(R,left_wheel_outline)
right_wheel_outline_rotated = np.matmul(R,right_wheel_outline)

plt.plot(robo_outline_rotated[0,:],robo_outline_rotated[1,:])
plt.plot(left_wheel_outline_rotated[0,:],left_wheel_outline_rotated[1,:])
plt.plot(right_wheel_outline_rotated[0,:],right_wheel_outline_rotated[1,:])
plt.show()



