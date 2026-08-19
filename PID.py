
import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# import time

class PID:
    def __init__(self, Kp, Ki, Kd, Nd, dt, ctrl_input_limits=(None, None)):
        """
        Initialisation du régulateur PID (Forme parallèle discrète).
        """
        # Gains continus convertis en gains discrets lors de l'exécution
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Nd = Nd

        self.dt = dt
        
        self.min_ctrl_input, self.max_ctrl_input = ctrl_input_limits
        
        # Variables internes
        self._integral_error = 0.0  # u_i

        self._derivative = 0.0
        self._last_error = 0.0  # epsilon_k
        self._last_time = None

    def PID_tuning(self,tf_syst,M_phi,w_u,factor_i,plot_option):

        # --- PID for z (altitude) ---
        #M_phi = 70  # Desired phase margin (degrees)
        #w_u = 0.9     # Ultimate frequency (rad/s)

        # Bode analysis at w_u_z  # mag_G_wu_z, phase_G_wu_z, _
        #
        mag_G, phase_G_rad, omega_G = ctrl.bode_plot(tf_syst, [w_u], deg=True,  plot = False)
        #mag, phase, omega = ctrl.frequency_response(tf_syst_z, [w_u_z])

        mag_G_wu = mag_G[0] # Magnitude (not in dB) 
        phase_G_wu = phase_G_rad[0]*180/np.pi  # Phase in degrees


        #factor_i = 4  # 5 # Integral factor
        w_i = w_u / factor_i
        tau_i = 1 / w_i
        w_list = [1e-3,1e2]
        I_tf = ctrl.tf([tau_i, 1], [tau_i, 0])

        # Phase margin calculation
        Phi_m = (M_phi - phase_G_wu) * np.pi / 180  # Convert to radians

        if abs(Phi_m) >= np.pi / 2:
            Phi_m = np.sign(Phi_m) * 0.99 * np.pi / 2
            print(f"Max phase margin: {phase_G_wu + 90} °")

        # PID parameters 
        a_PID = (1 + np.sin(Phi_m)) / (1 - np.sin(Phi_m))
        tau_a = np.sqrt(a_PID) / w_u
        tau_b = 1 / (w_u * np.sqrt(a_PID))
        D_tf = ctrl.tf([tau_a, 1], [tau_b, 1])

        #ctrl.bode_plot(D_tf, omega = w_list , Hz=False, deg=True, label='Derivative corrector', wrap_phase=True)
        #plt.legend()   # wrap_phase

        # Gain calculation
        C0 = -1 / (np.sqrt(1 / factor_i**2 + 1) * np.sqrt(a_PID) * mag_G_wu)
        
        PD_tf = C0 * D_tf

        PID_tf = I_tf * PD_tf

        OL_tf = PID_tf * tf_syst # openloop

        #print(C0,tau_a,tau_b)
            
        if plot_option:
            plt.figure()
            ctrl.bode_plot(tf_syst, omega = w_list , Hz=False, deg=True, label='Syst') 
            ctrl.bode_plot(OL_tf, omega = w_list, Hz=False, deg=True, label='OL Syst', display_margins=True)
            plt.legend()
            plt.show()


        # ct.tf('z')
        [self.Kp, self.Ki, self.Kd, self.Nd] = [(tau_i+tau_a-tau_b)*C0/tau_i, C0/tau_i, tau_a*C0 -(tau_i + tau_a - tau_b)*C0/tau_i*tau_b, tau_a*C0/tau_b - (tau_i + tau_a - tau_b)*C0/tau_i]

        # Plot pole zero map and BF response continuous
        if plot_option:
            CL_tf = ctrl.feedback(OL_tf,1) #closed loop        

            pz_map = ctrl.pole_zero_map(CL_tf)
            ctrl.pole_zero_plot(pz_map)

            response = ctrl.step_response(CL_tf)
            plt.plot(response.time, response.outputs)
            plt.xlabel('Time [s]')
            plt.ylabel('Response y')
            plt.grid()
            plt.title('Step response')
            #plt.legend()

        return

    def reset(self):
        """Réinitialise les variables internes du PID (anti-windup/mémoire)."""
        self._integral_error = 0.0
        self._derivative = 0.0
        self._last_error = 0.0
        self._last_time = None

    def compute_ctrl_input(self, error):
        """
        Calcule la commande PID en fonction de l'erreur actuelle.
        """
        # discretize PID tuning
        #error epsilon 

        
        # 1. Terme Proportionnel
        P = self.Kp * error
        
        # 2. Terme Intégral (Euler arrière : Ki_disc = Ki * dt)
        self._integral_error += self.Ki * self.dt * error 
        
        # Anti-Windup : Saturation de l'intégrale pour éviter l'emballement
        #if self.max_output is not None and self.min_output is not None:
            # On limite l'intégrale à la plage de sortie maximale possible
        #     self._integral = max(self.min_output, min(self._integral, self.max_output))
            
        I = self._integral_error
        
        # 3. Terme Dérivé pur (Euler arrière : Kd_disc = Kd / dt)
        #D = self.Kd * (error - self._last_error) / self.dt

        # 3. Filtered derivative
        D = self.Kd*self.Nd /(self.Kd + self.Nd*self.dt) * (error - self._last_error) + self.Kd /(self.Kd + self.Nd*self.dt) * self._derivative
        
        # Somme des trois termes
        ctrl_input = P + I + D


        #ctrl_input + 
        
        # Saturation de la sortie globale
        if self.min_ctrl_input is not None:
            ctrl_input = max(self.min_ctrl_input, ctrl_input)
        if self.max_ctrl_input is not None:
            ctrl_input = min(self.max_ctrl_input, ctrl_input)
            
        # Sauvegarde des états pour le prochain cycle
        self._last_error = error
        self._derivative = D

        
        return ctrl_input