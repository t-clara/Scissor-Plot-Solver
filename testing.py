import unittest
import numpy as np
from solver import ScissorPlotSolver

class TestScissorPlotSolver(unittest.TestCase):
    def setUp(self):
        self.solver = ScissorPlotSolver()
        self.solver.initialize_sweep()
        self.solver.velocity_ratio('T-tail and canard')
        self.solver.display_info_x_ac(M=self.solver.M_cruise)
        self.solver.CL_alpha_w = self.solver.CL_alpha_DATCOM(A=self.solver.A, M=self.solver.M_cruise)
        self.solver.CL_alpha_h = self.solver.CL_alpha_DATCOM(A=self.solver.A_h, M=self.solver.M_cruise_tail)
        self.solver.solve_stability()

    def test_CL_alpha_Ah(self):
        CL_alpha_Ah_check = (2 * np.pi * self.solver.A) / (self.solver.A + 2)
        error_margin = 0.10

        print(f"\nCHECKING VALIDITY OF CL-ALPHA-A-H:")
        print(f"Calculated: {self.CL_alpha_Ah}")
        print(f"Expected Order: {CL_alpha_Ah_check}\n")

        self.assertTrue(CL_alpha_Ah_check * (1.00 - error_margin) < self.solver.CL_alpha_Ah < CL_alpha_Ah_check * (1.00 + error_margin))

    def test_downwash_gradient(self):
        downwash_gradient_check = 4 / (self.solver.A + 2)
        error_margin = 0.10

        print(f"\nCHECKING VALIDITY OF DOWNWASH GRADIENT:")
        print(f"Calculated: {self.downwash_gradient}")
        print(f"Expected Order: {downwash_gradient_check}\n")

        self.assertTrue(downwash_gradient_check * (1.00 - error_margin) < self.solver.downwash_gradient < downwash_gradient_check * (1.00 + error_margin))

        

# if __name__ == '__main__':
#     unittest.main()