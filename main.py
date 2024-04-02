'''
========= Scissor Plot Solver =========
Applications: Stability and controllability curve generation
              of various configurations of existing conventional
              aircrafts
Authors: Thibault Clara
Courses: AE ADSEE-III, TU Delft
'''

# Imports
from solver import ScissorPlotSolver
from testing import TestScissorPlotSolver

# Packages
import unittest
import numpy as np

'''
=== CG INFORMATION CALCULATED BY ANA ===
~~~ NORMAL ~~~
the most forward cg location is at 0.3722889639160259 [%MAC]
the most aft cg location is at 0.791845196982712 [%MAC]
the most forward cg location is at 0.3648431846377054 [%MAC] (2% margin)
the most aft cg location is at 0.8076821009223663 [%MAC] (2% margin)

~~~ H2 VERSION ~~~
the most forward cg location is at 0.3687187643884546 [%MAC]
the most aft cg location is at 0.833053157140377 [%MAC]
the most forward cg location is at 0.3613443891006855 [%MAC] (2% margin)
the most aft cg location is at 0.8497142202831846 [%MAC] (2% margin)
'''

cg_range_F100 = np.array([0.3648431846377054, 0.8076821009223663])
cg_range_F120H2 = np.array([0.3613443891006855, 0.8497142202831846])

if __name__ == '__main__':
    solver = ScissorPlotSolver(name_aircraft="Fokker 100", flap_type="Double-Slotted Krueger Flaps", airfoil_series="NACA 43012A", show=False)
    solver.initialize_sweep()
    solver.velocity_ratio('T-tail and canard')
    solver.display_info_x_ac(M=solver.M_cruise)
    solver.CL_alpha_w = solver.CL_alpha_DATCOM(A=solver.A, M=solver.M_cruise)
    solver.CL_alpha_h = solver.CL_alpha_DATCOM(A=solver.A_h, M=solver.M_cruise_tail)
    solver.solve_stability()
    solver.CL_h('fixed')
    solver.solve_controllability()
    solver.plot(cg_range=cg_range_F100)