import unittest

from PyKDL import Rotation, Vector
import numpy as np

from dvrk_planning.controller.cartesian_teleop_controller import CartesianFollowTeleopController, CartesianIncrementTeleopController
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

class TestSetup(unittest.TestCase):
    def setUp(self):
        self.joint_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
        self.kin_solver = PsmKinematicsSolver(LND400006())
        self.output_start_tf = self.kin_solver.compute_fk(self.joint_pos)
        self.inc_x = 0.01
        self.steps_num = 5
        print("--- TestSetup ---")
        print("output start transform: \n", self.output_start_tf)

class TestIncrementTeleopController(TestSetup):
    def callback_function(self, js):
        print("output joint state to send: \n", js)
        print("output transform result: \n", self.kin_solver.compute_fk(js))

    def setUp(self):
        super().setUp()
        self.itc = CartesianIncrementTeleopController(self.kin_solver)
        self.itc.register(self.callback_function)

        print("--- TestIncrementTeleopController ---")

    def test_increment_pos_x(self):
        print("--- test_increment_pos_x ---")
        self.itc.enable(self.output_start_tf)
        for _ in range(self.steps_num):
            self.itc.update(Vector(self.inc_x, 0.0, 0.0), Rotation.Quaternion(0.0, 0.0, 0.0, 1.0))
        self.itc.disable()

    def test_increment_rot_x(self):
        print("--- test_increment_rot_x ---")
        inc_rot_x = 1.5708/2.0 #45 deg
        self.itc.enable(self.output_start_tf)
        for _ in range(self.steps_num):
            self.itc.update(Vector(0.0, 0.0, 0.0), Rotation.RPY(inc_rot_x, 0.0, 0.0))
        self.itc.disable()

    def test_with_camera_frame_pos_x(self):
        print("--- test_with_camera_frame_pos_x ---")
        output_ref_to_input_rot = Rotation.RPY(0.0, 0.0, 1.5708) # x becomes y
        # we should see increment in y now, for an increment in input x
        itc = CartesianIncrementTeleopController(self.kin_solver, output_ref_to_input_rot = output_ref_to_input_rot)
        itc.register(self.callback_function)
        itc.enable(self.output_start_tf)
        for _ in range(self.steps_num):
            itc.update(Vector(self.inc_x, 0.0, 0.0), Rotation.RPY(0.0, 0.0, 0.0))
        itc.disable()

class TestFollowTeleopController(TestSetup):
    def callback_function(self, js):
        print("output joint state to send: \n", js)
        print("output transform result: \n", self.kin_solver.compute_fk(js))

    def setUp(self):
        super().setUp()
        self.ftc = CartesianFollowTeleopController(self.kin_solver)
        self.ftc.register(self.callback_function)
        print("--- TestFollowTeleopController ---")

    def test_increment_pos_x(self):
        print("--- test_increment_pos_x ---")
        input_start_tf = np.identity(4)
        print("input start transform: \n", input_start_tf)

        self.ftc.enable(input_start_tf, self.output_start_tf)
        for _ in range(5):
            input_start_tf[0,3] = input_start_tf[0,3] + self.inc_x
            self.ftc.update(input_start_tf)
        self.ftc.disable()

    def test_clutch(self):
        def track_output_callback(js):
            print("output joint state to send: \n", js)
            track_output_callback.tracked_tf = self.kin_solver.compute_fk(js)
            print("output transform result: \n", track_output_callback.tracked_tf)

        print("--- test_clutch ---")
        input_start_tf = np.identity(4)
        print("input start transform: \n", input_start_tf)

        ftc = CartesianFollowTeleopController(self.kin_solver)
        ftc.register(track_output_callback)
        ftc.enable(input_start_tf, self.output_start_tf)
        for _ in range(5):
            input_start_tf[0,3] = input_start_tf[0,3] + self.inc_x
            ftc.update(input_start_tf)

        ftc.clutch()
        input_start_tf = np.identity(4) # input becomes 0 again
        print("input clutched back to zero again transform: \n", input_start_tf)
        print("current output tf when clutched: \n", track_output_callback.tracked_tf)

        # output tf should expect to continue incrementing x
        # even though input was zeroed
        ftc.unclutch(input_start_tf, track_output_callback.tracked_tf)
        for _ in range(5):
            input_start_tf[0,3] = input_start_tf[0,3] + self.inc_x
            ftc.update(input_start_tf)
        ftc.disable()

if __name__ == '__main__':
    unittest.main()
