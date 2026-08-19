import numpy as np



def PID_controller(state_k,pos_ref,PID_x,PID_theta,PID_z,list_param):  # ,data
    #with mass compensation

    z = state_k[1,0]
    vz = state_k[3,0]
    m = state_k[6,0]
    z_ref = pos_ref[1,0] 
    err_z = z_ref - z  # beware order

    x = state_k[0,0]
    vx = state_k[2,0]
    x_ref = pos_ref[0,0] 
    err_x = x_ref - x

    theta = state_k[4,0]


    theta_eq = np.pi/2  #data

    # list_param = [g, g_0, I_spv, l_I, l_q, m_0, I_0, eta_eq, Kzm, delta_l_q, delta_l_I, delta_I_spv, delta_m]
    m_0 = list_param[5] 
    eta_eq = list_param[7]
    Kzm = list_param[8]

    #eta =  1.0*C0_z *err_z + eta_eq  + Kzm *(m-m_0) 
    eta =  PID_z.compute_ctrl_input(err_z) + Kzm *(m-m_0) +  eta_eq 

    # saturation 
    eta = max(0,eta)


    delta_thetades = PID_x.compute_ctrl_input(err_x) 

    delta_theta_sat = 100*np.pi/180  #40
     # saturation
    delta_thetades = min(max(delta_thetades,-delta_theta_sat),delta_theta_sat)

    err_theta = delta_thetades - (theta-theta_eq)

    delta = PID_theta.compute_ctrl_input(err_theta) 

    delta_sat = 20*np.pi/180 

    delta= min(max(delta,-delta_sat),delta_sat)

    #delta = 0
    

    #uk = 0

    uk = np.array([[eta,delta]]).T
    

    return uk, delta_thetades + np.pi/2



def feedback_lin_second_order(state_k,pos_ref,speed_ref,u_prev,Te,list_perf,list_param):
    """ Give the control input for a feedback linearization second order command 
    """

    # State
    x = state_k[0,0]
    z = state_k[1,0]
    vx = state_k[2,0]
    vz = state_k[3,0]
    theta = state_k[4,0] 
    q = state_k[5,0] #w_theta
    m = state_k[6,0]
    I = state_k[7,0] # inertia

    xdes = pos_ref[0,0]
    zdes = pos_ref[1,0]

    vxdes = speed_ref[0,0]
    vzdes = speed_ref[1,0]

    # Parameters
    g = list_param[0]  #gravité  lunaire
    g_0 = list_param[1]  #acc ref de la propulsion
    I_spv =list_param[2]  #impulsion specifique
    l_I = list_param[3] 
    l_q = list_param[4] 
    m_0 = list_param[5] 
    I_0 = list_param[6] 


    w_x = list_perf[0]
    xi_x = list_perf[1]
    w_theta = list_perf[2]
    xi_theta = list_perf[3]
    w_z = list_perf[4]
    xi_z = list_perf[5]

    #eta = (m-m_0) +  eta_eq 

    prev_eta,prev_delta = u_prev[0],u_prev[1]

    # Pos Z -> eta
    eta = -m/(g_0*I_spv*np.sin(theta))*(-g - 2*xi_z*w_z*(vz - vzdes) - w_z**2*(z - zdes))  #
    m_predict = m - eta*Te/2
    #eta = -m_predict/(g_0*I_spv*np.sin(theta))*(-g - 2*xi_z*w_z*(vz - vzdes) - w_z**2*(z - zdes)) 

    # saturation
    eta_min = 0.1
    eta = max(eta_min,eta)  #eta>=0


    #eta_predict = (eta + prev_eta)/2

    # Pos x -> theta
    #print(m/(eta*g_0*I_spv)*(-2*xi_x*w_x*(vx - vxdes) - w_x**2*(x - xdes)))
    thetades = np.acos( saturation_linear(m/(eta*g_0*I_spv)*( -2*xi_x*w_x*(vx - vxdes) - w_x**2*(x - xdes)),-1,1) )  # *l_q

    #thetades = np.acos( saturation_linear(m_predict/(eta*g_0*I_spv)*( -2*xi_x*w_x*(vx - vxdes ) - w_x**2*(x - xdes)),-1,1) ) #- prev_delta  #  


    #cos(pi/2 + delta_theta) = -sin(delta_theta)
    #thetades = -m/(eta*g_0*I_spv)*( - 2*xi_x*w_x*(vx - vxdes) - w_x**2*(x - xdes)) + np.pi/2
    #thetades = -m_predict/(eta*g_0*I_spv)*( - 2*xi_x*w_x*(vx - vxdes) - w_x**2*(x - xdes)) + np.pi/2

    delta_theta_sat = 40*np.pi/180  #40

    # saturation
    #delta_thetades = min(max(thetades - np.pi/2-delta_theta_sat,),delta_theta_sat)
    thetades = saturation_linear(thetades,np.pi/2-delta_theta_sat,np.pi/2+delta_theta_sat)
    #thetades = np.pi/2 + delta_thetades

    # Theta -> delta
    delta = np.asin( saturation_linear(-I/(eta*g_0*I_spv*l_q)*( -2*xi_theta*w_theta*q - w_theta**2*(theta - thetades)),-1,1) ) 
    #delta = -I/(eta*g_0*I_spv*l_q)*( -2*xi_theta*w_theta*q - w_theta**2*(theta - thetades))# -qdes

    I_predict = I - l_I**2*eta**Te/2
    #delta = np.asin( saturation_linear(-I_predict/(eta*g_0*I_spv*l_q)*( -2*xi_theta*w_theta*q - w_theta**2*(theta - thetades)),-1,1) ) 
    #delta = -I_predict/(eta*g_0*I_spv*l_q)*( -2*xi_theta*w_theta*q - w_theta**2*(theta - thetades))

    #delta = 0

    # saturation
    delta_sat = 20*np.pi/180 
    delta = min(max(delta,-delta_sat),delta_sat)
    

    uk = np.array([[eta, delta]]).T

    return uk, thetades

def f_dyn(t, state_k, u_k, list_param):
# continuous dynamic function   
# sf_BO_atterisseur
# u_k : eta, delta
# state : [x,z,vx,vz,theta,w_theta,m,I]

    [eta,delta] = u_k
    vx = state_k[2,0]
    vz = state_k[3,0]
    theta = state_k[4,0] 
    q = state_k[5,0] #w_theta
    m = state_k[6,0]
    I = state_k[7,0] # inertia

    # list_param = [g, g_0, I_spv, l_I, l_q, m_0, I_0, eta_eq, Kzm, delta_l_q, delta_l_I, delta_I_spv, delta_m]

    g = list_param[0]  #gravité  lunaire
    g_0 = list_param[1]  #acc l ration de r f de la propulsion
    I_spv = list_param[2]  #impulsion spécifique
    l_I = list_param[3] 
    l_q = list_param[4] 
    m_0 = list_param[5] 
    I_0 = list_param[6] 
    delta_l_q = list_param[9] 
    delta_l_I = list_param[10] 
    delta_I_spv = list_param[11] 
    delta_m = list_param[12] 


    # non_lin with parameters discrepancies
    Xdot = np.array([[vx,
        vz,
        1/(m+delta_m)*eta*g_0*(I_spv+delta_I_spv)*np.cos(theta+delta), # a_x
        -1/(m+delta_m)*eta*g_0*(I_spv+delta_I_spv)*np.sin(theta+delta) + g,   # a_z
        q,
        -1/I*eta*g_0*(I_spv+delta_I_spv)*(l_q+delta_l_q)*np.sin(delta), # + 0.05/I_0*eta_eq*g_0*I_spv*l_q ,  # a_theta
        -eta , #dm variation masse
        -(l_I+delta_l_I)**2*eta  # dI variation
        ]]).T

    # lin
    """eta_eq = g/(g_0*I_spv)*m_0
    Xdot = np.array([[vx,
        vz,
        1/m*eta*g_0*I_spv*np.cos(theta+delta), # a_x
        -1/m*eta*g_0*I_spv + g,   # a_z
        q,
        -1/I*eta*g_0*I_spv*l_q*np.sin(delta), # + 0.05/I_0*eta_eq*g_0*I_spv*l_q ,  # a_theta
        -eta , #dm variation masse
        -l_I**2*eta
        ]]).T"""
    
    return Xdot



def RK4(t,Xk,uk,wk,Te,n,data):
    """ Runge-Kutta order 4"""
    # Xk : state at k
    # uk : command at k
    # Te : sampling period
    # Xk_1 : state at k+1 
    k_1 = f_dyn(t,Xk,uk,data)
    #Xk_12 = [Xk[i] + Te/2*k_1[i] for i in range(n) ] # len(Xk)
    Xk_12 = Xk + Te/2*k_1
    k_2 = f_dyn(t,Xk_12,uk,data)
    #Xk_23 = [Xk[i] + Te/2*k_2[i] for i in range(n) ]
    Xk_23 = Xk + Te/2*k_2    
    k_3 = f_dyn(t,Xk_23,uk,data)
    #Xk_34 = [Xk[i] + Te*k_3[i] for i in range(n) ]
    Xk_34 = Xk + Te*k_3
    k_4 = f_dyn(t,Xk_34,uk,data)

    #Xk_1 = [Xk[i] + Te/6*(k_1[i] + 2*k_2[i] + 2*k_3[i] + k_4[i]) for i in range(n) ]
    Xk_1 = Xk + Te/6*(k_1 + 2*k_2 + 2*k_3 + k_4)

    return Xk_1

def Jacobi_XU(X, U, list_param):
    """
    Returns the Jacobian matrices M (state Jacobian) and N (input Jacobian)
    for a given operating point (X, U).

    Parameters:
    -----------
    X : numpy.ndarray
        State vector [x, z, Vx, Vz, theta, q, m, I].
    U : numpy.ndarray
        Control input vector [eta, delta].
    list_param : list or numpy.ndarray
        List of constant parameters: [g, g_0, I_spv, l_I, l].

    Returns:
    --------
    M : numpy.ndarray
        State Jacobian matrix (8x8).
    N : numpy.ndarray
        Input Jacobian matrix (8x2).
    """

    # Extract constant parameters
    g = list_param[0]       # Lunar gravity
    g_0 = list_param[1]    # Reference acceleration for propulsion
    I_spv = list_param[2]  # Specific impulse
    l_I = list_param[3]    # Distance parameter
    l_q = list_param[4]      # Lever arm

    # Extract state variables
    theta = X[4]  # Angle (rad)
    m = X[6]      # Mass (kg)
    I = X[7]      # Moment of inertia (kg·m²)

    # Extract control inputs
    eta = U[0]    # Mass flow rate (kg/s)
    delta = U[1]  # Angle command (rad)

    # Initialize Jacobian matrices
    M = np.zeros((8, 8))
    N = np.zeros((8, 2))

    # Fill M (state Jacobian)
    M[0, 2] = 1.0               # dx/dVx = 1
    M[1, 3] = 1.0               # dz/dVz = 1
    M[4, 5] = 1.0               # dtheta/dq = 1

    # Nonlinear terms in M
    M[2, 4] = -1/m * eta * g_0 * I_spv * np.sin(theta + delta)  # dVx/dtheta
    M[3, 4] = -1/m * eta * g_0 * I_spv * np.cos(theta + delta)  # dVz/dtheta

    M[2, 6] = -1/m**2 * eta * g_0 * I_spv * np.cos(theta + delta)  # dVx/dm
    M[3, 6] = 1/m**2 * eta * g_0 * I_spv * np.sin(theta + delta)   # dVz/dm

    M[5, 7] = 1/I**2 * eta * g_0 * I_spv * l_q * np.sin(delta)       # dq/dI

    # Fill N (input Jacobian)
    N[2, 0] = 1/m * g_0 * I_spv * np.cos(theta + delta)           # dVx/deta
    N[3, 0] = -1/m * g_0 * I_spv * np.sin(theta + delta)          # dVz/deta
    N[2, 1] = -1/m * eta * g_0 * I_spv * np.sin(theta + delta)    # dVx/ddelta
    N[3, 1] = -1/m * eta * g_0 * I_spv * np.cos(theta + delta)    # dVz/ddelta

    N[5, 0] = -1/I * g_0 * I_spv * l_q * np.sin(delta)              # dq/deta
    N[5, 1] = -1/I * eta * g_0 * I_spv * l_q * np.cos(delta)        # dq/ddelta

    N[6, 0] = -1.0                                              # dm/deta
    N[7, 0] = -l_I**2                                           # dI/deta

    return M, N

def saturation_linear(x,x_min,x_max):

    return min(max(x,x_min),x_max)

def gen_traj_ref_second_order(t,t_delay,x0,z0,x_f,h_f):

    #traj 2nd ordre

    w_x_traj = 0.0302
    w_z_traj = 0.0209

    t_d = t-t_delay
    #zdes = np.full_like(t, h_f)  # Desired altitude
    #xdes = np.full_like(t, x_f)  # Desired horizontal position
    xdes = x0 - np.heaviside(t_d, 1) * (x0-x_f)*(1-(1+w_x_traj*t_d)*np.exp(-w_x_traj*t_d))
    zdes = z0 - np.heaviside(t_d, 1) * (z0-h_f)*(1-(1+w_z_traj*t_d)*np.exp(-w_z_traj*t_d))

    vxdes = np.heaviside(t_d, 1) * (x_f - x0)*w_x_traj**2*t_d * np.exp(-w_x_traj*t_d)
    vzdes = np.heaviside(t_d, 1) * (h_f - z0)*w_z_traj**2*t_d * np.exp(-w_z_traj*t_d)

    pos_ref = np.array([xdes,zdes])

    speed_ref = np.array([vxdes,vzdes])

    return pos_ref, speed_ref