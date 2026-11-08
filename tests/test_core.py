import numpy as np
from types import SimpleNamespace
from so101_ik.kinematics import Kinematics
from so101_ik.controller import SimulationController
from so101_ik.collision import CollisionChecker
from so101_ik.model import TCPConfig

class FakeRobot:
    n = 6
    qlim = np.array([[-1.]*6, [1.]*6])
    q = np.zeros(6)
    def fkine(self, q, end):
        assert end == "tcp_link"
        T = np.eye(4)
        T[:3, 3] = q[:3]
        a, b = q[3:5]
        ca, sa, cb, sb = np.cos(a), np.sin(a), np.cos(b), np.sin(b)
        rx = np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
        ry = np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
        T[:3,:3] = rx @ ry @ np.diag([1.,-1.,-1.])
        return SimpleNamespace(A=T)

class Model:
    n = 6
    qlim = FakeRobot.qlim
    tcp = TCPConfig()
    def __init__(self):
        self.robot = FakeRobot()

def test_ik_fk():
    k = Kinematics(Model())
    sol = k.ik([0.2, 0.1, -0.15], q0=np.zeros(6))
    assert sol.success
    assert sol.direction_error_deg < 2
    assert np.linalg.norm(k.fk(sol.q)[:3,3]-[0.2,0.1,-0.15]) < 0.002

def test_unreachable():
    assert not Kinematics(Model()).ik([4,0,0], q0=np.zeros(6)).success

def test_collision_fail_closed():
    m = Model()
    c = SimulationController(m, Kinematics(m), CollisionChecker(m))
    assert not c.move_tcp([0.2,0,0]).accepted
    d = SimulationController(m, Kinematics(m), CollisionChecker(m), allow_unchecked_collision=True)
    assert d.move_tcp([0.2,0,0]).accepted

def test_collision_rejection():
    m = Model()
    m.robot.isselfcollided = lambda q: True
    c = SimulationController(m, Kinematics(m), CollisionChecker(m))
    assert c.move_tcp([0.2,0,0]).reason == "Self collision"
