import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import matplotlib.animation as an



def sim_run(num,obs,x,y,t):
    obs[0].set_data(x[0:num],y[0:num])
    return obs[0],

def sim_run(num,obs,x,y2,t):
    obs[1].set_data(x[0:num],y2[0:num])
    return obs[1],

def sim_run(num,obs,x):
    obs[2].set_data(x[0:num])
    return obs[2],













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
    plot_obs =[sine]
    plt.xlim(x[0],x[-1])
    plt.ylim(-1.5,1.5)
    ax_world.grid(True)
    
    ax_plot1 = fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
    cosine, = ax_plot1.plot([],[],'k')
    plot_obs =[sine,cosine]
    plt.xlim(x[0],x[-1])
    plt.ylim(-1.5,1.5)
    ax_plot1.grid(True)
    
    ax_plot2 = fig.add_subplot(gs[1,2],facecolor=(0.9,0.9,0.9))
    x = 0
    line, =ax_plot2.plot(x,'k')
    plot_obs =[sine,cosine,line]
    ax_plot2.grid(True)
    
    ax_plot3 = fig.add_subplot(gs[2,2],facecolor=(0.9,0.9,0.9))
    
   
    #show the plot
    plt.tight_layout()
    #run animation
    robo_simulator = an.FuncAnimation(fig,sim_run, frames = frame_count, \
        fargs =(plot_obs,x,y,t),interval = 20, repeat= True, blit=True)
    
    plt.show()
    
    
    
    
if __name__=='__main__':
    main()