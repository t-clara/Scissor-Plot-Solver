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


if __name__ == '__main__':
    solver = ScissorPlotSolver(name_aircraft="Fokker 100", flap_type="Double-Slotted Krueger Flaps")
    solver.initialize_sweep()
    solver.velocity_ratio('T-tail and canard')
    solver.display_info_x_ac(M=solver.M_cruise)
    solver.CL_alpha_w = solver.CL_alpha_DATCOM(A=solver.A, M=solver.M_cruise)
    solver.CL_alpha_h = solver.CL_alpha_DATCOM(A=solver.A_h, M=solver.M_cruise_tail)
    solver.solve_stability()
    solver.CL_h('fixed')
    solver.solve_controllability()

    #test_suite = unittest.TestLoader().loadTestsFromTestCase(TestScissorPlotSolver)

    # Run the test suite
    #unittest.TextTestRunner(verbosity=2).run(test_suite)

    #solver.plot()