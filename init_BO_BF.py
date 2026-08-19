import numpy as np
from scipy.signal import TransferFunction, lti
import control as ctrl

# --- PARAMETERS ---
# Mass parameters
m_i = 15000      # Inert mass (kg)
m_c = 8000       # Fuel mass (kg)
m_0 = m_i + m_c  # Total initial mass (kg)
I_0 = 100000     # Moment of inertia (kg·m²)
l_I = 2.0917     # Distance parameter (m) # l


# Gravity and propulsion
g = 1.62         # Lunar gravity (m/s²)
g_0 = 9.81       # Earth gravity reference (m/s²)
I_spv = 3000 / g_0  # Specific impulse (s)
l_q = 2.0  # Lever arm between the center of mass and the application point of the thrust force (m)
# l_q or l

# Lunar parameters
M_Lune = 7.348e22  # Mass of the Moon (kg)
G = 6.67e-11       # Gravitational constant (m³/kg·s²)
R_Lune = 1737e3    # Radius of the Moon (m)
v_min_sat = np.sqrt(G * M_Lune / R_Lune)  # Minimum orbital velocity (m/s)

# Equilibrium thrust rate
eta_eq = m_0 * g / (g_0 * I_spv)  # Equilibrium mass flow rate (kg/s)

# Initial conditions
theta_0 = np.pi / 2  # Initial angle (rad)

# Initial control input
U0 = np.array([eta_eq, 0])  # [eta, delta]

# Reference positions
zref = 1  # Reference altitude (m)
xref = 1  # Reference horizontal position (m)

# --- ACTUATOR MODELING ---
# Actuator dynamics (2nd-order transfer functions)
num_actionneur = 1  # Numerator for actuator transfer functions

# Natural frequencies and damping ratio
w_0_delta = 15  # Natural frequency for delta actuator (rad/s)
w_0_eta = 8      # Natural frequency for eta actuator (rad/s)
xi = 0.9         # Damping ratio

# Denominator coefficients for delta and eta actuators
den_actionneur_delta = [1 / w_0_delta**2, 2 * xi / w_0_delta, 1]
den_actionneur_eta = [1 / w_0_eta**2, 2 * xi / w_0_eta, 1]

# Transfer functions for actuators
#tf_actionneur_eta = TransferFunction(num_actionneur, den_actionneur_eta)
#tf_actionneur_delta = TransferFunction(num_actionneur, den_actionneur_delta)

tf_actionneur_eta = ctrl.tf(num_actionneur, den_actionneur_eta)
tf_actionneur_delta = ctrl.tf(num_actionneur, den_actionneur_delta)

# --- TRANSFER FUNCTIONS FOR LINEARIZED SYSTEM ---
# Gain for z (altitude) dynamics
Kz = -1 / m_0 * g_0 * I_spv
#tfz_th = TransferFunction([Kz], [1, 0, 0])  # Transfer function for z dynamics
tfz_th = ctrl.tf([Kz], [1, 0, 0])

# Gain for x (horizontal) dynamics
Kx = -1 / m_0 * eta_eq * g_0 * I_spv
#tfx_th = TransferFunction([Kx], [1, 0, 0])  # Transfer function for x dynamics
tfx_th = ctrl.tf([Kx], [1, 0, 0])

# Gain for theta (angle) dynamics
Kt = -1 / I_0 * eta_eq * g_0 * I_spv * l_q
#tft_th = TransferFunction([Kt], [1, 0, 0])  # Transfer function for theta dynamics
tft_th = ctrl.tf([Kt], [1, 0, 0])

# --- MASS LOSS COMPENSATION ---
# Gain for mass loss compensation
Km = 1 / m_0**2 * eta_eq * g_0 * I_spv
Kzm = -Km / Kz  # Compensation gain for z dynamics