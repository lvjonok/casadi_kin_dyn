import casadi_kin_dyn.casadi_kin_dyn as cas_kin_dyn
import casadi

urdf = open("cart_pole.urdf", "r").read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

kindyn.nq()
print(kindyn.potentialEnergy()) 
