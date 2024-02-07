import casadi_kin_dyn.casadi_kin_dyn as cas_kin_dyn
from robot_descriptions.a1_description import URDF_PATH
import numpy as np

urdf = open(URDF_PATH, "r").read()
kindyn = cas_kin_dyn.CasadiKinDyn(
    urdf, root_joint=cas_kin_dyn.CasadiKinDyn.JointType.FREE_FLYER
)

print(kindyn.nq())
print(kindyn.potentialEnergy())
print(kindyn.jacobianCenterOfMass(True))

c = cas_kin_dyn.CollisionHandler(kindyn)

q = np.random.randn(kindyn.nq())
d = np.zeros((c.numPairs(),))
J = np.zeros((c.numPairs(), kindyn.nv()), order="F")
