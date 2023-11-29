# casadi_kin_dyn

Package for generation of symbolic (SX) expressions of robot kinematics and dynamics. Based on URDF and Pinocchio.

## What has changed?

In comparison to the original [casadi_kin_dyn](https://github.com/ADVRHumanoids/casadi_kin_dyn) repository, such changes were made:

- Migration to `cmeel`-based packaging
- Use `pre-commit` to prettify repository
- Added `torque`, `potentialEnergy`, `kineticEnergy` regressors
- Added `jacobian time derivative` mapping
- Added `jacobian of CoM` mapping
