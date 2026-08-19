import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib import rc
from math import pi, cos, sin
import control as ctrl  # Install with: pip install control

#from Jacobi_XU import *

from useful_functions import *

# --- INITIALIZATION (init_BO_BF.m) ---
from init_BO_BF import * # import constants
# run init_BO_BF.m #run just one time

from plot_perf_comp import *

from PID import PID


from io import BytesIO
from PIL import Image

# System linear transfer functions with actuators
#tf_syst_z = lti(np.convolve(tf_actionneur_eta.num,tfz_th.num),np.convolve(tf_actionneur_eta.den,tfz_th.den))  
tf_syst_z = tf_actionneur_eta * tfz_th
#tf_syst_theta = lti(np.convolve(tf_actionneur_delta.num,tft_th.num),np.convolve(tf_actionneur_delta.den,tft_th.den))  
tf_syst_theta = tf_actionneur_delta * tft_th  

#tf_syst_x = tfx_th

#close all

# --- MAIN SCRIPT ---
# Initial state
h0 = 800          # Initial altitude (m)
x0 = 800          # Initial horizontal position (m)
z0 = h0          # Altitude in chosen frame (z upward)
theta_0 = pi/2    # Initial angle (rad)
V_0 = 0 #10    # 50       # Initial velocity (m/s)
Vz_0 = V_0 * cos(theta_0 -0.1 )  # Vertical velocity (m/s)
Vx_0 = V_0 * sin(theta_0 -0.1)  # Horizontal velocity (m/s)   #v_min_sat
# - pi/2

# Full state vector: [x, z, Vx, Vz, theta, q, m, I]
X0 = np.array([x0, z0, Vx_0, Vz_0, theta_0, 0, m_0, I_0])
X0lin = np.array([x0, z0, Vx_0, Vz_0, 0, 0, m_0, I_0])  # Linearized state
nx = X0.shape[0]

# Control parameters
thetades_max = 40 * pi / 180  #30 40 50 # Max desired angle (rad)
linear_control = 0  # 1: PID, 0: SMC, -1: SMC with integrators, -2: Super-Twisting SMC
plotall = 0         # Flag to plot all results



# --- PERtURBAtIONS ---
# (perturbation_handler.m)
incertitude = 1
delta_l_q = incertitude * 0.40       # Mass center offset of l_q  [m]
delta_l_I = incertitude * 0.15 * l_I      # Mass center offset of l_I  [m]
delta_I_spv = incertitude * 0.15 * I_spv  # Inertia offset
delta_m = 0*incertitude * 0.15 * m_0  # Initial mass offset/uncertainties

# List of constant parameters
list_param = [g, g_0, I_spv, l_I, l_q, m_0, I_0, eta_eq, Kzm, delta_l_q, delta_l_I, delta_I_spv, delta_m]

# --- LINEARIZAtION (JacobiXU) ---
# Jacobian matrices A, B
[A, B] = Jacobi_XU(X0, U0, list_param)  # Linearized system matrices

nu = B.shape[1]

#print("A=",A,"B=",B)


# --- CONtROL CONFIGURAtION ---
#linear_control == 1:


# --- SIMULATION ---

# Run simulation
# state = sol.y  # state: [x, z, Vx, Vz, theta, q, m, I]


# --- TRAJECTORY DEFINITION ---
t_f = 300 #350 #400 # 50 # 130 # 800        # Final time (s)
dt = 0.1 #0.05 # 0.005 # 0.01  # Time step (s)
Te = dt
t = np.arange(0, t_f + dt, dt)  # Time array
Nsim = len(t)

# Final position objectives
h_f = 0   # 200 m | 800          # Final altitude (m)
x_f = 0     # 500 m | 800        # Final horizontal position (m)

# --- DESIRED STATES / traj --- 
thetades = pi/2 * np.ones_like(t)  # Default dsired angle 
qdes = np.zeros_like(t)    # Desired angular velocity

t_delay = 0 #20
pos_ref, speed_ref = gen_traj_ref_second_order(t,t_delay,x0,z0,x_f,h_f) #

xdes = pos_ref[0,:]   
zdes = pos_ref[1,:]  
vxdes = speed_ref[0,:]    # Desired horizontal velocity
vzdes = speed_ref[1,:]   # Desired vertical velocity



PID_z = PID(0, 0, 0, 0, Te)
PID_x = PID(0, 0, 0, 0, Te)
PID_theta = PID(0, 0, 0, 0, Te)

M_phi_z = 70  # Desired phase margin (degrees)
w_u_z = 0.9     # Ultimate frequency (rad/s)
factor_i_z = 5  # 4 5 # Integral factor
PID_z.PID_tuning(tf_syst_z,M_phi_z,w_u_z,factor_i_z,False)


M_phi_x = 86
w_u_x = 0.25  # 0.5
factor_i_x = 3
PID_x.PID_tuning(tfx_th,M_phi_x,w_u_x,factor_i_x,False)

M_phi_t = 80 # theta
w_u_t = 1.7  # 1.0
factor_i_t = 20 # 20
PID_theta.PID_tuning(tf_syst_theta,M_phi_t,w_u_t,factor_i_t,False)

#   w_u 
#   w_i = wu/5

# 2nd order perf coeff
xi_x, xi_theta, xi_z = 1, 1, 1
t5x, t5theta, t5z = 32.0, 3.4, 1.4
w_x, w_theta, w_z = 4.75 / t5x, 4.75 / t5theta, 4.75 / t5z

list_perf = [w_x, xi_x, w_theta, xi_theta, w_z, xi_z]


delta_sat = 20*np.pi/180 
delta_theta_sat = 40*np.pi/180  #40 #100


#eta = np.zeros_like(t)  # Control input 1 (mass flow rate)
#delta = np.zeros_like(t)  # Control input 2 (angle)

state = np.zeros((nx,Nsim))
#state = np.full_like(t, X0)
state[:,0] = X0
u = np.zeros((nu,Nsim-1))
u_prev = np.zeros(nu)

#Kz_P,Kz_I,Kz_D = PID_tuning()

### Main simulation loop
for k in range(0,Nsim - 1):

    #print("---- Time step",k)  #  ,"/",N



    u[:, k:k+1] = np.zeros((nu,1))

    # 2nd order with mass variation compensation
    #u[:, k:k+1] = second_order_controller()

    # PID with mass variation compensation
    #u[:, k:k+1],thetades[k] = PID_controller(state[:, k:k+1],pos_ref[:,k:k+1],PID_x, PID_theta, PID_z, list_param)

    # 2nd order dynamic (with feedback linearization)

    u[:, k:k+1],thetades[k] = feedback_lin_second_order(state[:, k:k+1],pos_ref[:,k:k+1],speed_ref[:,k:k+1],u_prev,Te,list_perf,list_param)


    # Sliding Mode Control
    #u[:, k:k+1],thetades[k] = SMC_dynamic_controller(Xglob,Thetadot,Xref_locplan,Xref,t,k,data,perf_SMC,1)


    # MPC (TO DO)

    # System dynamic - add actuators dynamic tf_actionneur_eta tf_actionneur_delta
    state[:, k+1: k+2] = RK4(k,state[:, k:k+1],u[:, k],np.zeros((nu,1)),Te,nx,list_param)  # Xglob[:, k + 1]
    #Xglob_RK4[6:9, k + 1] = fun.normalize_angle(Xglob_RK4[6:9, k + 1])
    
    u_prev = u[:, k]


# Extract results
x = state[0,:]
z = state[1,:]
vx = state[2,:]
vz = state[3,:]
theta = state[4,:]
q = state[5,:]
m = state[6,:]
I = state[7,:]


pos = state[0:2,:]

eta = u[0,:]
delta = u[1,:]


# Compute RMSE for position tracking
pos_RMSE = np.mean(np.sqrt(np.sum((pos - pos_ref[:, :])**2, axis=0)))
final_pos_RMSE = np.mean(np.sqrt(np.sum((pos[:,Nsim-20:] - pos_ref[:, Nsim-20:])**2, axis=0)))


 
print(f"Position RMSE: {pos_RMSE:.4f}")  # 5.3782  #15s
print(f"Final Position RMSE: {final_pos_RMSE:.4f}")  # 5.3782  #15s


#theta_wrap = np.arctan2(np.sin(theta), np.cos(theta))  # Wrap to [-pi, pi]

# --- ERROR CALCULATION ---
e_x = x - xdes
e_z = z - zdes
e_t = theta - thetades
e_dx = vx - vxdes
e_dz = vz - vzdes
e_dt = q - qdes

# Sliding surfaces (for SMC)
if linear_control != 1:
    se_z = np.zeros_like(t)  # Placeholder: Replace with actual sliding surface
    se_x = np.zeros_like(t)
    se_t = np.zeros_like(t)

# --- PERFORMANCE METRICS ---
sse = np.sum(e_x**2 + e_z**2)
etot = np.sum(np.sqrt(e_x**2 + e_z**2))
emoyenne = etot / len(e_x)
t_ef = 5
ef = (1 / (t_ef/dt)) * np.sum(np.sqrt(e_x[-int(t_ef/dt):]**2 + e_z[-int(t_ef/dt):]**2))

# --- PLOTTING ---
##plot_err_ref(e_x,e_z,e_t,t)


# Traj
#plot_traj(x,z,xdes,zdes)

##plot_perf_pos(x,z,xdes,zdes,t)
#plot_perf_ref(x,z,xdes,zdes,theta,thetades,t)

#print("STOP")


# --- ADDITIONAL PLOTS ---

##animated_plot_traj(x,z,xdes,zdes,Nsim)
save_gif = False
save_gif = True
animated_plot_traj_vehicle(x,z,t,xdes,zdes,theta,eta,delta,Nsim,save_gif)

#plot_command(eta,delta,t)


print("STOP")
