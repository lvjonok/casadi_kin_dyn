# casadi_kin_dyn

Package for generation of symbolic (SX) expressions of robot kinematics and dynamics. Based on URDF and Pinocchio.

## What has changed?

In comparison to the original [casadi_kin_dyn](https://github.com/ADVRHumanoids/casadi_kin_dyn) repository, such changes were made:

- Migration to `cmeel`-based packaging
- Use `pre-commit` to prettify repository
- Added `torque`, `potentialEnergy`, `kineticEnergy` regressors
- Added `jacobian time derivative` mapping
- Added `jacobian of CoM` mapping

# v1.6.7

- Added ability to set `root_joint` to `FreeFlyer` joint

# v1.6.8

- `root_joint` is now transformed to enum that can be freely extended with Pinocchio JointModel types.

# v1.6.9

- One can now fix `floating` joints in the model by passing a list of `[x, y, z, qvx, qvy, qvz, qs]`.

```python
import casadi_kin_dyn.casadi_kin_dyn as cas_kin_dyn

kindyn = cas_kin_dyn.CasadiKinDyn(
    urdf,
    root_joint=cas_kin_dyn.CasadiKinDyn.JointType.OMIT,
    fixed_joints={"floating_base_joint": np.array([0, 0, 0, 0, 0, 0, 1])},
)
```

# v1.6.10

- Derivatives of centroidal dynamics are exposed from `pinocchio::computeCentroidalDynamicsDerivatives`
