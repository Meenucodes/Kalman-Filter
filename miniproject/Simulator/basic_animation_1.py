import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import matplotlib.animation as an

prev_x = 0   #global variables for previous x and y pos
prev_y = 0

def make_robot():  #defining robot outline
    robot_params={"lr":1, "lf":1, "overhang":1, "track":2, "wheel_dia":1, "wheel_offset":0.25, "scale_x":0.25, "scale_y":0.25}
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
    
    return robo_outline,left_wheel_outline,right_wheel_outline

def sim_run(num,obs,x,y,y2,t):
    global prev_x, prev_y
    obs[0].set_data(x[0:num],y[0:num])
    obs[1].set_data(x[0:num],y2[0:num])
    obs[2].set_data([x[num],x[num]],[0,10])
    robo_outline,left_wheel_outline,right_wheel_outline = make_robot()
    # finding angle of last position
    delta_y = y[num] - prev_y  
    prev_y = y[num]
    delta_x = x[num] - prev_x
    prev_x = x[num]
    theta = np.arctan2(delta_y,delta_x)
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])  #rotation matrix
    robo_outline_rotated = np.matmul(R,robo_outline)
    left_wheel_outline_rotated = np.matmul(R,left_wheel_outline)
    right_wheel_outline_rotated = np.matmul(R,right_wheel_outline)

    
    robo_trans = np.tile([[x[num]],[y[num]]],(1,6))   ## defining translational matrix
    wheel_trans = np.tile([[x[num]],[y[num]]],(1,5))
    robo_outline = robo_outline_rotated + robo_trans
    left_wheel_outline = left_wheel_outline_rotated + wheel_trans
    right_wheel_outline = right_wheel_outline_rotated + wheel_trans
    obs[4].set_data(robo_outline[0,:],robo_outline[1,:])
    obs[5].set_data(left_wheel_outline[0,:],left_wheel_outline[1,:])
    obs[6].set_data(right_wheel_outline[0,:],right_wheel_outline[1,:])
    
    # rotating a line
    line_length = 1
    x1 = -line_length/2
    y1 = 0
    x2 = line_length/2
    y2 = 0
    line = np.array([[x1, x2],[y1, y2]])
    line = line + np.array([[x[num], x[num]],[5, 5]])
    R = np.array([[np.cos(x[num]), np.sin(x[num])],[-np.sin(x[num]),np.cos(x[num])]])
    line_new = np.matmul(R, line)
    obs[2].set_data(line_new[0, :], line_new[1, :])   
    return obs[0],obs[1],obs[2],obs[4],obs[5],obs[6]

def main():
    print("Robot Simulator")
    # do all robotic stuffs below
    t = np.linspace(0,4*np.pi,101)
    x = t
    y = np.sin(t)
    y2 = np.cos(t)
    frame_count = len(t)
    
    
       
    #do all animation stuffs below
    #animations starts here##############
    #dots per inch shall not be too high
    #face color is RGB NORMALISED
    fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
    # divide the simulation area into 9 parts (3x3)
    gs = gridspec.GridSpec(3,3)
    #world to simulate robot
    ax_world = fig.add_subplot(gs[0:3,0:2],facecolor=(0.9,0.9,0.9))
    sine, = ax_world.plot([],[],'k')
    robot_outline, = ax_world.plot([],[],'k')
    left_wheel_outline, = ax_world.plot([],[],'k')
    right_wheel_outline, = ax_world.plot([],[],'k')
    
    plt.xlim(x[0],x[-1])
    plt.ylim(-3,3)
    ax_world.grid(True)
    
    ax_plot1 = fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
    cosine, = ax_plot1.plot([],[],'k')
    plt.xlim(x[0],x[-1])
    plt.ylim(-1.5,1.5)
    ax_plot1.grid(True)
    
    ax_plot2 = fig.add_subplot(gs[1,2],facecolor=(0.9,0.9,0.9))
    ver_line, =ax_plot2.plot([],[],'k')
    plt.xlim(0,x[-1])
    plt.ylim(0,10)
    
    ax_plot3 = fig.add_subplot(gs[2,2],facecolor=(0.9,0.9,0.9))
    slant_line, =ax_plot3.plot([],[],'k',linewidth=2)
    
    plt.xlim(0,x[-1])
    plt.ylim(0,10)
    
    plot_obs= [sine,cosine,ver_line,slant_line,robot_outline,left_wheel_outline,right_wheel_outline]
   
    #show the plot
    plt.tight_layout()
    #run animation
    robo_simulator = an.FuncAnimation(fig,sim_run, frames = frame_count, \
        fargs =(plot_obs,x,y,y2,t),interval = 20, repeat= True, blit=True)
    
    plt.show()
     
    
    
    
if __name__=='__main__':
    main()