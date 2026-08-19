
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib import rc
from math import pi, cos, sin

from matplotlib.patches import Wedge, Rectangle
import matplotlib.transforms as transforms
from matplotlib.transforms import Affine2D


from io import BytesIO
from PIL import Image



def plot_err_pos(e_x,e_z,t):

    plt.figure(figsize=(10, 6))
    # Plot errors
    plt.subplot(2, 1, 1)
    plt.plot(t, e_z, '-r', label='e_z')
    plt.grid()
    plt.title('Errors')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(t, e_x, label='e_x')
    plt.grid()
    plt.legend()
    plt.xlabel('time (s)')

    #plt.tight_layout()
    plt.show()
    return

def plot_err_ref(e_x,e_z,e_t,t):

    plt.figure(figsize=(10, 6))
    # Plot errors
    plt.subplot(3, 1, 1)
    plt.plot(t, e_z, '-r', label='$e_z$')
    plt.grid()
    plt.title('Errors with reference')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, e_x, label='$e_x$')
    plt.grid()
    plt.legend()


    plt.subplot(3, 1, 3)
    plt.plot(t, e_t*180/np.pi, label=r'$e_\theta [deg]$')
    plt.grid()
    plt.legend()
    plt.xlabel('Time (s)')

    #plt.tight_layout()
    plt.show()
    return

def plot_command(eta,delta,t):
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t[:-1], eta, label=r'$\eta$ (kg/s)')
    plt.grid()
    plt.legend()
    plt.title('Mass flow rate command')
    plt.subplot(2, 1, 2)
    plt.plot(t[:-1], delta * 180 / pi, 'r', label=r'$\delta$ (deg)')
    plt.grid()
    plt.title('Angle command')
    plt.xlabel('time (s)')
    plt.legend()
    plt.show()

    return


def plot_perf_ref(x,z,xdes,zdes,theta,thetades,t):

    # plt.subplot(2, 1, 1)
    #plt.figure()

    fig,(ax1, ax2, ax3) = plt.subplots(3, 1) #  , sharey=True
    #plt.title('Position tracking')
    ax1.set_title('Reference tracking')
    ax1.plot(t, x, label=r'$x$ [m]')
    ax1.plot(t, xdes, label=r'$x_{ref}$ [m]')
    ax1.grid()
    ax1.legend()
    #plt.subplot(2, 1, 2)
    ax2.plot(t, z, label=r'$z$ [m]')
    ax2.plot(t, zdes, label=r'$z_{ref}$ [m]')
    ax2.grid()
    ax2.legend()
    ##ax2.yaxis.set_inverted(True)
    ax3.plot(t, theta*180/np.pi, label=r'$\theta$ [deg]')
    ax3.plot(t, thetades*180/np.pi, label=r'$\theta_{ref}$ [deg]')
    ax3.grid()
    ax3.legend()
    plt.xlabel('Time [s]')

    #plt.show()


    return

def plot_perf_pos(x,z,xdes,zdes,t):

    # plt.subplot(2, 1, 1)
    #plt.figure()

    fig,(ax1, ax2) = plt.subplots(2, 1) #  , sharey=True
    #plt.title('Position tracking')
    ax1.set_title('Position tracking')
    ax1.plot(t, x, label=r'$x$ [m]')
    ax1.plot(t, xdes, label=r'$x_{ref}$ [m]')
    ax1.grid()
    ax1.legend()
    #plt.subplot(2, 1, 2)
    ax2.plot(t, z, label=r'$z$ [m]')
    ax2.plot(t, zdes, label=r'$z_{ref}$ [m]')
    ax2.grid()
    plt.xlabel('Time [s]')
    ##ax2.yaxis.set_inverted(True)
    ax2.legend()
    plt.show()


    return


def plot_traj(x,z,xdes,zdes):

    fig, ax = plt.subplots(1, 1, figsize=(6,4))
    plt.plot(x,z, '-or', label='Traj')
    plt.plot(xdes,zdes, '--og', label='Traj ref')
    plt.xlabel('pos x (m)')
    plt.ylabel('pos z (m)')
    ##ax.yaxis.set_inverted(True)
    plt.grid()
    plt.title('Trajectory')
    plt.legend()

    return

# Animated plot
def animated_plot_traj(x,z,xdes,zdes,Nsim):

    # --- Set up the figure and axis ---
    Fig, ax = plt.subplots(1, 1, figsize=(6, 4))
    ax.set_xlabel('pos x (m)')
    ax.set_ylabel('pos z (m)')
    ##ax.yaxis.set_inverted(True)  # Invert y-axis for altitude-like display
    ax.grid()
    ax.set_title('Trajectory')

    # Initialize empty lines for trajectory and reference
    lines_traj = plt.plot(x[:0], z[:0], '-or', label='Traj')
    line_traj = lines_traj[0]
    lines_ref = plt.plot([], [], '--og', label='Traj ref') # ,
    line_ref = lines_ref[0]
    ax.legend()
    #ax.legend(loc='upper right')
    #ax.set_xlim(-100, 1000)
    #ax.set_ylim(-1000, 1000)
    ax.autoscale_view()

    # plt.show()

    current_frame = 0
    frames = []
    dynamic_artists = []

    # --- Create the animation ---
    # Parameters
    draw_speed = 5  # Animation speed (milliseconds per frame). Lower = faster.
    save_as_gif = False #True  # Set to False to disable GIF saving
    gif_filename = 'trajectory_animation.gif'  # Output filename
    

    # --- Animation function ---
    def update_anim(frame):
        global current_frame, dynamic_artists
        """Update the plot for each frame of the animation."""

        current_frame += draw_speed
        if current_frame >= Nsim:
            current_frame = Nsim 

        # for artist in dynamic_artists:
        #         if artist:
        #             artist.remove()
        # dynamic_artists = []

        # Update trajectory line (up to current frame)
        line_traj.set_data((x[:current_frame+1], z[:current_frame+1]))

        #trajectory, = ax.plot(x[:current_frame+1],z[:current_frame+1], "r-")
        #dynamic_artists.append(trajectory)

        # Update reference line (full trajectory)
        line_ref.set_data((xdes, zdes))
        #line_ref, = ax.plot([], [], '--og', label='Traj ref')



        # Adjust axis limits dynamically (optional)
        ax.set_xlim(min(x[:current_frame+1])-10, 1.05*max(x[:current_frame+1]))
        ax.set_ylim(min(z[:current_frame+1])-10, 1.05*max(z[:current_frame+1]))

        #frames.append(_copy_frame(fig))

        return (line_traj, line_ref)
        #return dynamic_artists


    def _copy_frame(fig):
        """Copie la frame actuelle de l'axe sous forme d'image."""
        with BytesIO() as buff:
            fig.savefig(buff, format='raw')
            buff.seek(0)
            data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
        w, h = fig.canvas.get_width_height()
        im = data.reshape((int(h), int(w), -1))
        return im

    gif_duration=50.0
    fps_gif = 20
    gif_frames_speed = int(Nsim/fps_gif/gif_duration)

    # Create animation
    ani = FuncAnimation(Fig,update_anim,
        frames=Nsim,  # Number of frames = number of points in trajectory
        interval=gif_frames_speed,  # Delay between frames (ms)
        blit=False,  # Optimize rendering
        repeat=False
    )
    # --- Display the animation ---
    #plt.tight_layout()
    #plt.show()

    # --- Save as GIF (if enabled) ---
    if save_as_gif:
        frames_pil = [Image.fromarray(frame) for frame in frames]
        # Sauvegarder le premier frame pour initialiser le GIF
        frames_pil[0].save(
        gif_filename,
        save_all=True,
        append_images=frames_pil[1:],
        duration=int(1/fps_gif),  # Durée de chaque frame en ms
        loop=0,  # 0 = boucle infinie
        disposal=2   # transparency=0,
        )


    return





def animated_plot_traj_vehicle(x,z,t,xdes,zdes,theta,eta,delta,Nsim,save_gif):
    """ Animated plot """

    # dim vehicle
    scale = 5
    w_rect = scale * 3.5 # width rectangle
    h_rect = scale * 10.0
    h_tri = scale * 5.0
    w_tri = scale * 2.5

    h_max_fire = 10*h_tri # 4

    fig,ax = plt.subplots()
    plt.title("Animated plot vehicle trajectory")


    plt.grid()
    ax.set_xlabel('Pos x [m]')
    ax.set_ylabel('Pos z [m]') 
    ax.set_xlim([min(xdes)-4*w_rect,max(xdes)+4*w_rect])
    ax.set_ylim([min(zdes)-4*h_rect,max(zdes)+4*h_rect])
    ##ax.yaxis.set_inverted(True) 

    traj_real_plot, = ax.plot([],[],linestyle="dashed",color="b",label="Traj real")  # real traj

    traj_des_plot, = ax.plot([],[],color="g",label="Traj ref")
    #dynamic_artists = []
    ax.legend(loc='upper left')

    #rectangle =  plt.Rectangle((x[0]-w_rect/2, z[0]-h_rect/2),  w_rect, h_rect, edgecolor='black', lw=1)
    #axis.add_artist(rectangle)

    #triangle = plt.Polygon(((x[0], z[0]-h_rect/2), (x[0]+w_tri/2, z[0]-h_rect/2-h_tri), (x[0]-w_tri/2, z[0]-h_rect/2-h_tri)),fc=(1,0,0,0.5), ec=(0,0,0,1), lw=1)
    #axis.add_artist(triangle)

    eta_mean = np.mean(eta)
    eta_min = 0
    eta_max = 3*eta_mean #max(eta)
    #eta_max = max(eta)

    # Use the 'YlOrRd' colormap (Yellow -> Orange -> Red)
    cmap = plt.cm.YlOrRd # RGBA

    rectangle_verts = np.array([
    [-w_rect/2, -h_rect/2],
    [w_rect/2, -h_rect/2],
    [w_rect/2, h_rect/2],
    [-w_rect/2, h_rect/2],
    ])

    # Triangle (equilateral, side=0.5)
    triangle_verts = np.array([
        [0, -h_rect/2],
        [-w_tri/2, -h_rect/2-h_tri],
        [w_tri/2, -h_rect/2-h_tri],
    ])

    left_leg_verts = np.array([
        [-w_rect/2, -h_rect/4],
        [-3*w_tri/2, -h_rect/2],
        [-3*w_rect/4, -3*h_rect/5-h_tri],
    ])

    right_leg_verts = np.array([
        [w_rect/2, -h_rect/4],
        [3*w_tri/2, -h_rect/2],
        [3*w_rect/4, -3*h_rect/5-h_tri],
    ])

    
    eta_norm = eta[0]/eta_max  
    fire_lines = np.array([
        [0, -h_rect/2-h_tri],
        [0, -h_rect/2-h_tri - eta_norm*h_max_fire]])
        #[-w_tri/3, -h_rect/2-h_tri]])  # -w_tri/2
    fire_color = cmap(eta_norm)   #  color=fire_color
    

    body_top_center = np.array([0,h_rect/2])

    rectangle = plt.Polygon(rectangle_verts, closed=True, edgecolor='blue', fc='none', lw=1.5)
    triangle = plt.Polygon(triangle_verts, closed=True, edgecolor='grey', fc='none', lw=1.5)

    body_top = Wedge(body_top_center,w_rect/2,180,0,fc='none',edgecolor="black")

    left_leg = plt.Polygon(left_leg_verts, closed=False, edgecolor='black', fc='none', lw=1)
    right_leg = plt.Polygon(right_leg_verts, closed=False, edgecolor='black', fc='none', lw=1)


    time_display = ax.text(300, 900, str(np.round(t[0],2))+" s",
                ha='center', va='center',
                fontsize=12, bbox={'facecolor':'white','edgecolor': 'black', 'pad': 6})  #  'alpha': 0.3,
 

    
    #ax.set_ylabel('Time (s): ' + str(i/10))


    fire_line1 = plt.Polygon(fire_lines, closed=True, color=fire_color, lw=2) # edgecolor 'orange' , fc='red'


    rect_moon = Rectangle((-100, -200), 1000, 200,           # Position and dimensions
    facecolor="grey",       # Fill color
    alpha=0.3,              # Transparency 
    edgecolor="dimgrey",    # Optional border color
    linewidth=1.5,          # Border width
    zorder=3                # Keeps it layered cleanly over background elements
    )

    ax.text(360,-120, 'Moon', fontsize=14, style='italic',
        bbox={'facecolor': 'blue', 'alpha': 0.3, 'pad': 6})

    ax.add_patch(rectangle)
    ax.add_patch(triangle)
    ax.add_patch(body_top)
    ax.add_patch(left_leg)
    ax.add_patch(right_leg)
    ax.add_patch(fire_line1)

    ax.add_patch(rect_moon)          # Draw the rectangle on the axes
    

    def update_data(frame):  # current_frame
        #global dynamic_artists  # nonlocal

        frame_buffer= max(0,frame-100)
        #traj_real_plot.set_data(x[frame_buffer:frame],z[frame_buffer:frame])

        traj_real_plot.set_data(x[0:frame],z[0:frame])
        traj_des_plot.set_data(xdes[0:frame],zdes[0:frame])

        #set_title('Animated plot vehicle trajectory | Frame ' + str(frame))   # t[frame]
        time_display.set_text(str(np.round(t[frame],2))+" s")

        x_f = x[frame]
        z_f = z[frame]
        theta_f = theta[frame]
        delta_theta_f = theta_f - np.pi/2  # delta theat around equilibrium # strange sign for displaying in reverse
        delta_f = delta[frame-1]


        # --- Rectangle / body transformation ---
        body_transform = Affine2D().rotate_around(
            0, 0, delta_theta_f      # + np.pi for the inverse $z$ coord
        ).translate(x_f, z_f)
        rectangle.set_xy(body_transform.transform(rectangle_verts))

        body_top.set_center(body_transform.transform(body_top_center))
        body_top.set_theta1(0 + delta_theta_f*180/np.pi)
        body_top.set_theta2(180 + delta_theta_f*180/np.pi)

        left_leg.set_xy(body_transform.transform(left_leg_verts))
        right_leg.set_xy(body_transform.transform(right_leg_verts))
    
        # --- Triangle transformation ---
        tri_transform = Affine2D().rotate_around(
            0, 0, delta_theta_f + delta_f
        ).translate(x_f, z_f+0.1)  # Offset for visibility
        triangle.set_xy(tri_transform.transform(triangle_verts))


        eta_norm = eta[frame-1]/eta_max  
        fire_color = cmap(eta_norm)
        fire_lines = np.array([
        [0, -h_rect/2-h_tri],
        [0, -h_rect/2-h_tri - eta_norm*h_max_fire]])
        fire_line1.set_xy(tri_transform.transform(fire_lines))
        fire_line1.set(color=fire_color)


        animate_objects = [time_display, traj_real_plot, traj_des_plot, rectangle, body_top, triangle, left_leg, right_leg, fire_line1] 

        return animate_objects

    if save_gif:
        skip_frames = 7 # 6
        frames_list = np.arange(0, len(t), skip_frames)
    else:
        frames_list = len(t)

    animation_traj =  FuncAnimation(fig, 
                                    func=update_data, 
                                    interval=10,  # 20  # 2
                                    frames=frames_list,  # 
                                    repeat=save_gif,
                                    blit=True)


    if save_gif:
        # To save the animation using Pillow as a gif
        writer = PillowWriter(fps=20,
                            metadata=dict(artist='Me'),
                            bitrate=1800) # 1800 500
        # palette='web'
        animation_traj.save('lunar_lander_traj.gif', writer=writer)

    plt.show()



    return

