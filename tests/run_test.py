import casadi_kin_dyn.casadi_kin_dyn as cas_kin_dyn

urdf = open("cart_pole.urdf", "r").read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

print(kindyn.nq())
print(kindyn.potentialEnergy())
print(kindyn.jacobian("mass", cas_kin_dyn.CasadiKinDyn.LOCAL))
print(kindyn.jacobianTimeVariation("mass", cas_kin_dyn.CasadiKinDyn.LOCAL))
print(kindyn.jacobianCenterOfMass(True))
