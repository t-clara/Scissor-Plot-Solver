'''
========= Scissor Plot Solver =========
Applications: Stability and controllability curve generation
              of various configurations of existing conventional
              aircrafts
Authors: Thibault Clara
Courses: AE ADSEE-III, TU Delft
'''

# Packages
import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate
import sys # to access the system
import cv2
import ctypes
import os


class ScissorPlotSolver:
    def __init__(self, name_aircraft: str, flap_type: str, airfoil_series: str, show: bool):
        self.name_aircraft = name_aircraft
        self.flap_type = flap_type
        self.airfoil_series = airfoil_series
        self.show = show

        print("Initializing ScissorPlotSolver...\n")
        '''
        === Fuselage ===
        l_f: fuselage length [m]
        b_f: exterior cabin width [m]
        h_f: inner cabin height [m]
        l_fn: distance to leading edge of wing at root [m]
        '''
        self.l_f                           = 32.5
        self.b_f                           = 3.3 
        self.h_f                           = 2.01 
        self.l_fn                          = 14 

        '''
        === Main Wing ===
        S: surface area of primary wing [m^2]
        S_net:
        b: span of primary wing [m]
        MAC: mean aerodynamic chord, denoted as x_bar [m]
        A: aspect ratio [-]
        taper_ratio: taper from root to tip, denoted as lambda [-]
        thickness_chord_ratio_average: (t/c)_average [-]
        sweep_quarter: quarter root chord sweep [rad]
        '''
        self.S                             = 93.5
        self.S_net                         = 0.9 * self.S # don't know how to compute this yet
        self.b                             = 28.08 
        self.MAC                           = 3.8
        self.A                             = 8.43 
        self.taper_ratio                   = 0.235 
        self.thickness_chord_ratio_average = 0.1028 
        self.sweep_quarter                 = 17.45 * np.pi / 180 

        '''
        === Horizontal Tail Surface ===
        S_h: surface area of horizontal tail [m^2]
        b_h: span of horizontal tail [m]
        A_h: aspect ratio of horizontal tail [-]
        taper_ratio_h: taper from root to tip, denoted as lambda [-]
        sweep_quarter_h: quarter root chord sweep [rad]
        l_h: tail length, critical sizing parameter [m]
        Sh_by_S: (Sh/S), ratio of horizontal tail surface to primary wing surface [-]
        tail_volume: ((S_h*l_h)/(S*MAC)) [-]
        '''
        self.S_h                           = 21.72 
        self.b_h                           = 10.04 
        self.A_h                           = 4.64
        self.taper_ratio_h                 = 0.39 
        self.sweep_quarter_h               = 26 * np.pi / 180 # [rad]
        self.l_h                           = 16 
        self.Sh_by_S                       = 0.232
        self.tail_volume                   = 0.978 

        '''
        === Cruise Conditions ===
        M_cruise: Mach number at cruising velocity
        M_cruise_tail: function of M_cruise, experienced by tail
        eta: airfoil efficiency assumption, constant obtained from slides
        '''
        self.M_landing                     = 0.2
        self.M_landing_tail                 = self.M_landing * 0.9  # randomized
        self.M_cruise                      = 0.77
        self.M_cruise_tail                 = self.M_cruise * 0.9  # randomized
        self.eta                           = 0.95
        
    def get_screen_resolution(self):
        user32 = ctypes.windll.user32
        return user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

    def show_images(self, filepath: str, filename: str):
        screen_width, screen_height = self.get_screen_resolution()
        img = cv2.imread(filepath, cv2.IMREAD_ANYCOLOR)
        # Calculate scaling factor to fit the image within the screen size
        scale_factor = min(screen_width / img.shape[1], screen_height / img.shape[0]) * 0.65
        # Resize the image while maintaining aspect ratio
        resized_img = cv2.resize(img, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
        top = 0
        left = screen_width - resized_img.shape[1]        
        cv2.imshow(filename, resized_img)
        cv2.moveWindow(filename, left, top)
        cv2.waitKey(0)

    def initialize_sweep(self):
        self.c_r = (2 * self.S) / (self.b * (1 + self.taper_ratio))
        self.c_t = self.c_r * self.taper_ratio
        self.sweep_LE = np.arctan(np.tan(self.sweep_quarter) + (self.c_r /  (2 * self.b)) * (1 - self.taper_ratio))
        self.sweep_half = np.arctan((self.b / 2 * np.tan(self.sweep_LE) + ((self.c_t - self.c_r) / 2)) / (self.b / 2))

    def velocity_ratio(self, key: str):
        if key == 'fuselage-mounted':
            self.V_ratio = 0.85
        if key == 'fin-mounted':
            self.V_ratio = 0.95
        if key == 'T-tail and canard':
            self.V_ratio = 1.00
        else:
            raise ValueError("Use one of the valid keys: 'fuselage-mounted', 'fin-mounted', 'T-tail and canard'\n")
        
    def CL_h(self, key: str):
        if key == 'full moving':
            self.CL_h = -1
        if key == 'adjustable':
            self.CL_h = -0.8
        if key == 'fixed':
            self.CL_h = -0.35 * self.A_h**(1/3)
        else:
            raise ValueError("Use one of the valid keys: 'full moving', 'adjustable', 'fixed'\n")
    
    def delta_y(self):
        while True:
            key = input("\nInput NACA Series: ")
            if key in ['4', '5', '63', '64', '65', '66']: 
                if key == '4':
                    return 26.0
                if key == '5':
                    return 26.0
                if key == '63':
                    return 22.0
                if key == '64':
                    return 21.3
                if key == '65':
                    return 19.3
                if key == '66':
                    return 18.3
            else:
                print("Use one of the valid keys: '4', '5', '63', '64', '65', '66'.\n")
        
    def display_info_x_ac(self, M):
        print("\n=== INFORMATION DISPLAY FOR TORENBEEK PLOTS ===\n")
        print(f"Displaying information required for reading of 'Fig. E-10. Aerodynamic center of lifting surfaces at subsonic speeds (Ref. E-31). (Source: Torenbeek)'...")
        print(f"Use printed parameters to read corresponding curves and determined the aerodynamic center of the primary wing...")
        beta = np.sqrt(1 - M**2)
        lambda_beta = np.arctan(np.tan(self.sweep_LE) / beta) * 180 / np.pi

        table_data = [
            ["βA", beta * self.A],
            ["λ", self.taper_ratio],
            ["Λ_β [deg]", lambda_beta],
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

        if self.show:
            filepath = "images\Torenbeek_Wing_Fuselage_AC.png"
            filename = "Torenbeek Wing Fuselage Aerodynamic Center Relation"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.x_ac_w = float(input("\nInput x_ac/MAC value read from the Torenbeek plot: ")) * self.MAC
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

    def CL_alpha_DATCOM(self, A, M):
        beta = np.sqrt(1 - M**2)
        CL_alpha = (2 * np.pi * A) / (2 + np.sqrt(4 + ((A * beta) / self.eta)**2 * (1 + (np.tan(self.sweep_half)**2)/(beta**2))))
        return CL_alpha
    
    def solve_stability(self):
        '''
        === Calling DATCOM Method ===
        > Solve for CL_alpha of aircraft less tail configuration, 
        and tail configuration only.
        '''
        self.CL_alpha_Ah = self.CL_alpha_w * (1 + 2.15 * self.b_f / self.b) * self.S_net / self.S + np.pi / 2 * self.b_f**2 / self.S
        CL_alpha_Ah_check = (2 * np.pi * self.A) / (self.A + 2)
        print(f"\nCHECKING VALIDITY OF CL-ALPHA-A-H:")
        print(f"Calculated: {self.CL_alpha_Ah}")
        print(f"Expected Order: {CL_alpha_Ah_check}\n")
        error_margin = 0.10
        if CL_alpha_Ah_check * (1.00 - error_margin) < self.CL_alpha_Ah < CL_alpha_Ah_check * (1.00 + error_margin):
            print("*** SANITY CHECK PASSED ***\n")
        else: 
            print("*** SANITY CHECK FAILED ***\n")
        '''
        === Aerodynamic Centers Calculation ===
        '''
        x_ac_f1 = - (1.8 / self.CL_alpha_Ah) * ((self.b_f * self.h_f * self.l_fn) / (self.S * self.MAC))
        c_g = self.S / self.b # mean geometric chord
        x_ac_f2 = (0.273 / (1 + self.taper_ratio)) * ((self.b_f * c_g * (self.b - self.b_f)) / (self.MAC**2 * (self.b + 2.15 * self.b_f))) * np.tan(self.sweep_quarter)

        x_ac = self.x_ac_w + x_ac_f1 + x_ac_f2
        
        '''
        === Downwash Gradient Calculations ===
        > Geometry derived by hand by Alper & Thibault,
        requires accompanying diagram.
        '''
        i_w = 3 * np.pi / 180 # [rad]
        h_h = 1.5 # [m]
        x_LE_root = self.l_fn
        x_TE_root = x_LE_root + self.c_r
        R = np.sqrt((self.l_h - (self.c_r - self.x_ac_w - x_LE_root))**2 + (h_h + (self.c_r - self.x_ac_w - x_LE_root) * np.sin(i_w))**2)
        gamma = np.arcsin(((2 * x_LE_root - x_TE_root + x_ac) * np.sin(i_w)) / self.c_r)
        beta = np.arccos(((self.l_h + (x_TE_root - 2 * x_LE_root - self.x_ac_w) * np.sin(i_w))) / R)
        Psi = np.pi - gamma + beta
        m_tv = 2 * R * np.sin(Psi) / self.b
        r = 2 * self.l_h / self.b

        K_epsilon_Lambda = (0.1124 + 0.1265 * self.sweep_LE + 0.1766 * self.sweep_LE**2) / (r**2) + 0.1024 / r + 2
        K_epsilon_Lambda0 = 0.1124 / r**2 + 0.1024 / r + 2

        self.downwash_gradient = (K_epsilon_Lambda / K_epsilon_Lambda0) * ((r / (r**2 + m_tv**2)) * (0.4876 / np.sqrt(r**2 + 0.6319 + m_tv**2)) + (1 + (r**2 / (r**2 + 0.7915 + 5.0734 * m_tv**2))**0.3113)*(1 - np.sqrt(m_tv**2 / (1 + m_tv**2)))) * (self.CL_alpha_w / (np.pi * self.A))
        downwash_gradient_check = 4/(self.A + 2)

        print(f"\nCHECKING VALIDITY OF DOWNWASH GRADIENT:")
        print(f"Calculated: {self.downwash_gradient}")
        print(f"Expected Order: {downwash_gradient_check}\n")
        error_margin = 0.10
        if downwash_gradient_check * (1.00 - error_margin) < self.downwash_gradient < downwash_gradient_check * (1.00 + error_margin):
            print("*** SANITY CHECK PASSED ***\n")
        else: 
            print("*** SANITY CHECK FAILED ***\n")
        '''
        === Solving Coefficients of Linear Model ===
        '''
        self.SM = 0.05 # standard
        self.m_s = ((self.CL_alpha_h / self.CL_alpha_Ah) * (1 - self.downwash_gradient) * (self.l_h / self.MAC) * (self.V_ratio) **2)**(-1)
        self.q_s_SM = - (self.x_ac_w - self.SM) / ((self.CL_alpha_h / self.CL_alpha_Ah) * (1 - self.downwash_gradient) * (self.l_h / self.MAC) * (self.V_ratio) **2) 
        self.q_s = - (self.x_ac_w) / ((self.CL_alpha_h / self.CL_alpha_Ah) * (1 - self.downwash_gradient) * (self.l_h / self.MAC) * (self.V_ratio) **2)
        print(f'This is the difference {self.x_ac_w - self.SM}')
        print(f'This is x_ac {self.x_ac_w}')
    
    def solve_controllability(self):
        '''
        === Moment Coefficient of Aerodynamic Center ===
        Contributions from:
        1) Primary Wing
        2) Flaps 
        3) Fuselage
        4) Nacelles
        '''
        '''
        > (1) Primary Wing <
        '''
        #TODO: CHOOSE AIRFOIL
        # (1) Approximation:
        # airfoil_camber = 0.022
        # airfoil_thickness = 0.12
        # Cm_0_airfoil = -np.pi * airfoil_camber / airfoil_thickness

        # (2) Get Actual Value from Database:
        ## Fokker 100 expected to have Fokker 12.5% airfoil but no data can be found.
        print(f"\nRUNNING CALCULATIONS FOR {self.airfoil_series} AIRFOIL...\n")
        Cm_0_airfoil = -0.013 # MARSKE MONARCH AIRFOIL (NACA 43012A)
        # Cm_0_airfoil = -0.349, report with NACA 63-415
        
        # Compute:
        Cm_ac_w = Cm_0_airfoil * ((self.A * np.cos(self.sweep_quarter)**2)/(self.A + 2 * np.cos(self.sweep_quarter)))
        '''
        > (2) Flaps <
        mu_1: flap_chord/extended_wing_chord, deflection angle of flap [deg]
        mu_2: flap_span/wing_span, taper_ratio
        mu_3: flap_span/wing_span, taper_ratio
        '''
        '''
        chord_extension_ratio: (c'/c)
        #TODO: flapped_ref_ratio: i made it up
        delta_CL_max: DATCOM 1978 Method
        CL_max: Landing condition at given flap deflection using Table 7.2 from Torenbeek
        deflection_flap: Table 7.2 from Torenbeek
        '''
        '''
        === DEFLECTION FLAP ANGLE SETTING ===
        '''
        print("=== INFORMATION DISPLAY FOR DEFLECTION FLAP ANGLE SETTING ===")
        table_data = [
            ["Flap Type", self.flap_type]
        ]
        print(tabulate(table_data, headers=["Category", "Value"], tablefmt="grid"))

        if self.show:
            filepath = "images\Table_7.2.png"
            filename = "Flap Deflection Setting"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.deflection_flap = float(input("\nInput δ_f [deg]: "))
                entry = float(input("\nInput CL_max / cos(Λ_.25): "))
                self.CL_max = entry * np.cos(self.sweep_quarter)
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue
    
        chord_extension_ratio = 1.25 #TODO: Editted this 
        flapped_ref_ratio     = 1.0
        flap_span             = 0.58 * self.b #TODO: i made it up
        # Thibault Edit: Function of MAC instead
        c_prime               = chord_extension_ratio * self.MAC #TODO: root chord, MAC chord?
        chord_flap            = 0.3 * self.MAC #TODO: i made it up
        # c_prime               = chord_extension_ratio * self.c_r #TODO: root chord, MAC chord?
        # chord_flap            = 0.3 * self.c_r #TODO: i made it up

        print("\n=== INFORMATION DISPLAY FOR TORENBEEK PLOTS ===\n")

        table_data = [
            ["(cf/c')", chord_flap / c_prime],
            ["FLAP SPAN / WING SPAN", flap_span / self.b],
            ["λ", self.taper_ratio],
            ["Flap Type", self.flap_type]
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

        if self.show:
            filepath = "images\mu_1.jpg"
            filename = "mu_1: Torenbeek Statistical Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.mu_1= float(input("\nInput μ1 value read from Torenbeek plot: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        if self.show:
            filepath = "images\mu_2.png"
            filename = "mu_2: Torenbeek Statistical Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.mu_2 = float(input("\nInput μ2 value read from Torenbeek plot: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        if self.show:
            filepath = "images\mu_3.png"
            filename = "mu_3: Torenbeek Statistical Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.mu_3 = float(input("\nInput μ3 value read from Torenbeek plot: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        print("\n=== INFORMATION DISPLAY FOR 1978 DATCOM PLOTS ===\n")

        table_data = [
            ["(cf/c)", chord_flap / self.MAC],
            ["δ_f [deg]", self.deflection_flap],
            ["(t/c)", self.thickness_chord_ratio_average],
            ["Flap Type", self.flap_type]
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))
        '''
        === ΔCL_max_base ===
        '''
        if self.show:
            filepath = "images\Fig_8.11.png"
            filename = "Delta_CL_max_base: 1978 DATCOM Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.delta_CL_max_base = float(input("\nInput ΔCL_max_base value read from 1978 DATCOM method: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue
        '''
        === k1 ===
        '''
        if self.show:
            filepath = "images\Fig_8.12.png"
            filename = "k1: 1978 DATCOM Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.k1= float(input("\nInput k1 value read from 1978 DATCOM method: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue
        '''
        === k2 ===
        '''
        if self.show:
            filepath = "images\Fig_8.13.png"
            filename = "k2: 1978 DATCOM Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.k2 = float(input("\nInput k2 value read from 1978 DATCOM method: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue
        '''
        === k3 ===
        '''
        if self.show:   
            filepath = "images\Fig_8.14.png"
            filename = "k3: 1978 DATCOM Method"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.k3 = float(input("\nInput k3 value read from 1978 DATCOM method: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        delta_CL_max          = self.delta_CL_max_base * self.k1 * self.k2 * self.k3

        '''
        === CALCULATION Δy ===
        '''
        print("\n=== CALCULATION Δy ===\n")
        print(f"Currently using a {self.airfoil_series} airfoil...")

        self.delta_y_coeff = self.delta_y()
        self.delta_y = self.delta_y_coeff * self.thickness_chord_ratio_average

        '''
        === RATIO (CL_max / Cl_max) ===
        '''
        print("\n=== INFORMATION DISPLAY FOR RATIO (CL_max / Cl_max) ===\n")
        table_data = [
            ["(Δy)", self.delta_y],
            ["Λ_LE [deg]", self.sweep_LE * 180 / np.pi],
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

        if self.show:
            filepath = "images\Fig_8.9.png"
            filename = "RATIO (CL_max / Cl_max)"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.ratio_CL_Cl_max = float(input("\nInput (CL_max / Cl_max): "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        self.x_ac_w = 0.27 * self.MAC
        self.CL_alpha_w = self.CL_alpha_DATCOM(A=self.A, M=self.M_landing)
        self.CL_alpha_h = self.CL_alpha_DATCOM(A=self.A_h, M=self.M_landing_tail)
        self.CL_alpha_Ah = self.CL_alpha_w * (1 + 2.15 * self.b_f / self.b) * self.S_net / self.S + np.pi / 2 * self.b_f**2 / self.S

        CL_max_clean = 2.55 
        delta_Cl_max = (self.ratio_CL_Cl_max) * CL_max_clean + delta_CL_max

        delta_Cm_quarter_flaps = self.mu_2 * (-self.mu_1 * delta_Cl_max * chord_extension_ratio - (self.CL_max + delta_Cl_max * (1 - flapped_ref_ratio)) * (1/8) * chord_extension_ratio * (chord_extension_ratio - 1)) + 0.7 * (self.A / (1 + 2 / self.A)) * self.mu_3 * delta_Cl_max * np.tan(self.sweep_quarter)
        delta_Cm_ac_flaps = delta_Cm_quarter_flaps - delta_CL_max
        
        '''
        > (3) Fuselage <
        '''
        CL_0 = 1.6
        fuselage_contribution = -1.8 * (1 - 2.5 * self.b_f / self.l_f) * ((np.pi * self.b_f * self.h_f * self.l_f)/(4 * self.S * self.MAC)) * (CL_0 / self.CL_alpha_Ah)
        
        '''
        > (4) Nacelles <
        I have no idea how you would calculate this...
        '''
        nacelle_contribution = 0
        
        '''
        > Final Equation <
        Note that 'self.deflection_flap' in [deg], requires conversion to [rad]
        '''
        flaps_contribution = delta_Cm_ac_flaps * self.deflection_flap * (np.pi / 180)
        Cm_ac = Cm_ac_w + flaps_contribution + fuselage_contribution + nacelle_contribution
        print(f'Cm_ac {Cm_ac}')

        print(f"Displaying information required for reading of lift coefficient of a T-tail (Sh/S=1) for various HLD settings (cruise, take off and landing). (Source: Obert AE4-211 31.8)'...\n")
        print(f"Use printed parameters to read corresponding lift coefficients from a wedge at V_min, landing HLD settings...\n")
        table_data = [
            ["Cm_ac", Cm_ac],
            ["CL_h", self.CL_h]
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

        if self.show:
            filepath = "images\Obert.png"
            filename = "Obert Plot"
            self.show_images(filepath=filepath, filename=filename)

        while True:
            try:
                self.CL_Ah= float(input("\nInput CL_Ah value read from the Obert plot: "))
                break
            except ValueError:
                print("ValueError: Enter a valid floating point number.\n")
                continue

        self.m_c = ((self.CL_h / self.CL_Ah) * (self.l_h / self.MAC) * self.V_ratio**2)**(-1)
        self.q_c = (Cm_ac / self.CL_Ah - self.x_ac_w) / ((self.CL_h / self.CL_Ah) * (self.l_h / self.MAC) * self.V_ratio**2)
        
        print("\n=== VERIFICATION KEY VALUES ===\n")
        table_data = [
            ["CL_h", self.CL_h],
            ["CL_A-h", self.CL_Ah],
            ["Cm_ac/CL_A-h", Cm_ac / self.CL_Ah]
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

    #TODO: Make sure to solve for "delta_x_cg_acceptable" and plot a horizontal line at the point where
    # the requirement is just met, returning the Sh/S min for satisfying both stability and controllability
    # curves. 

    def plot(self, cg_range: np.array):
        '''
        y = m*x + q
        Sh/S = m*x_cg + q
        Scale x-axis so we obtain x_cg/MAC
        '''
        x_lemac = 15.81
        x = np.linspace(x_lemac, x_lemac + self.MAC, 100000) #self.l_f/20
        y_s_SM = self.m_s * (x - x_lemac)  + self.q_s_SM
        y_s = self.m_s * (x - x_lemac) + self.q_s
        y_c = self.m_c * (x - x_lemac) + self.q_c

        y_intersection_LHS = self.m_c * (cg_range[0] * self.MAC) + self.q_c
        y_intersection_RHS = self.m_s * (cg_range[1] * self.MAC) + self.q_s_SM
        
        if y_intersection_LHS > y_intersection_RHS:
            minimized_tail_sizing = y_intersection_LHS
        else:
            minimized_tail_sizing = y_intersection_RHS

        plt.figure(figsize=(8, 5.5))  
        plt.title(f'Scissor Plot - {self.name_aircraft}')
        plt.xlabel('xcg/MAC [-]')
        plt.ylabel('Sh/S [-]')
        plt.xlim(0, 1)
        plt.ylim(0, 1)
        
        # Plotting the results of the stability and controlability curves
        plt.plot((x - x_lemac)/self.MAC, y_s_SM, color='blue', label=f'Stability Curve, SM={self.SM}')
        plt.plot((x - x_lemac)/self.MAC, y_s, color='dodgerblue', label=f'Stability Curve, without SM')
        plt.plot((x - x_lemac)/self.MAC, y_c, color='springgreen', label=f'Controllability Curve')

        #Plotting the results of the cg range
        plt.axvline(cg_range[0], linewidth=1, color='magenta', linestyle='--', label='Forward CG')
        plt.axvline(cg_range[1], linewidth=1, color='magenta', linestyle='--', label='Aft CG')
        plt.axhline(y=minimized_tail_sizing, xmin=cg_range[0], xmax=cg_range[1], color='magenta', label=f'CG Range')
        
        print("\n=== SCISSOR PLOT RESULTS ===\n")
        table_data = [
            ["x_cg_forward [% MAC]", round(cg_range[0]*100, 2)],
            ["x_cg_aft [% MAC]", round(cg_range[1]*100, 2)],
            ["Sh/S [%]", round(minimized_tail_sizing*100, 2)],
            ["Target Sh/S (Real Data) [%]", 23.20]
        ]
        print(tabulate(table_data, headers=["Coefficient", "Value"], tablefmt="grid"))

        plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1))  # Place legend outside plot area, to the right
        plt.tight_layout()  # Adjust layout to make room for legend
        folder_path = 'figures'
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        plt.savefig(os.path.join(folder_path, f'Scissor_Plot_{self.name_aircraft}.png'))
        plt.show()