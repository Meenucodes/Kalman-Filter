import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
import matplotlib.animation as an



def sim_run(num,obs,x,y,y2,t):
    obs[0].set_data(x[0:num],y[0:num])
    obs[1].set_data(x[0:num],y2[0:num])
    obs[2].set_data([x[num],x[num]],[0,10])
    obs[3].set_data([x[num]-2,x[num]+2],[0,10])   
    return obs[0],obs[1],obs[2],obs[3],

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
    plt.xlim(x[0],x[-1])
    plt.ylim(-1.5,1.5)
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
    plot_obs= [sine,cosine,ver_line,slant_line]
    plt.xlim(0,x[-1])
    plt.ylim(0,10)
    
   
    #show the plot
    plt.tight_layout()
    #run animation
    robo_simulator = an.FuncAnimation(fig,sim_run, frames = frame_count, \
        fargs =(plot_obs,x,y,y2,t),interval = 20, repeat= True, blit=True)
    
    plt.show()
    
    
    
    
if __name__=='__main__':
    main()