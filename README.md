# State Estimation for Fixed-Wing UAV

Comparison of Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), and Particle Filter (PF) for aircraft state estimation.

## Overview

This repository provides a fillable implementation outline for three state estimation algorithms applied to fixed-wing UAV dynamics. The 13-dimensional state includes position, velocity, attitude (quaternion), and angular rates.

## Setup

**Requirements:**
- Python 3.8+
- NumPy, SciPy, Matplotlib
- CasADi (for dynamics model)
- Jupyter Notebook

**Installation:**
```bash
git clone git@github.com:Path-Integral-Control/Estimation.git
cd Estimation
pip install numpy scipy matplotlib casadi jupyter
```

## Repository Structure

- `dynamics.ipynb`: Aircraft dynamics model (symbolic + numerical)
- `estimation_outline.ipynb`: Main assignment notebook (fillable template)
- `data/`: Trajectory CSV files (format described below)
- `README.md`: This file

## Dynamics Model

The `dynamics.ipynb` notebook provides the fixed-wing aircraft dynamics based on the Night Vapor platform. After running the notebook, the following functions are available:

**Available Functions:**
- `f(x, u)`: Continuous-time dynamics, returns 13D state derivative
- `F(x, u)`: Jacobian of dynamics with respect to state (13x13)
- `G(x, u)`: Jacobian of dynamics with respect to control (13x3)
- `param_values`: Dictionary of aircraft parameters

**State Vector (13D):**
```
[px, py, pz,                    # Position (world frame) [m]
 u, v, w,                       # Velocity (body frame) [m/s]
 qw, qx, qy, qz,                # Quaternion (world to body)
 omega_x, omega_y, omega_z]     # Angular velocity (body frame) [rad/s]
```

**Control Vector (3D):**
```
[throttle, elevator, rudder]    # Normalized control inputs [-1, 1]
```

## Trajectory Data

Trajectory files are CSV format with 17 columns:
```
time, px, py, pz, u, v, w, qw, qx, qy, qz, omega_x, omega_y, omega_z, throttle, elevator, rudder
```

**Data Source:** Trajectories are generated externally using the [auav_pylon_2025](https://github.com/CogniPilot/auav_pylon_2025) simulation environment.

**Measurement Model:** The estimators use 10 simulated sensor measurements:
- GPS position (3): px, py, pz
- Doppler velocity (3): u, v, w (body frame, noisy)
- IMU gyroscope (3): omega_x, omega_y, omega_z
- Pitot tube (1): airspeed magnitude

## Usage

1. Place trajectory CSV in `data/` folder
2. Open `estimation_outline.ipynb` and follow instructions
3. Complete the marked sections (`# YOUR CODE HERE`)
4. Run all cells to generate comparison results

## Assignment Tasks

Students must implement the following sections in `estimation_outline.ipynb`:

### Extended Kalman Filter (EKF)
- Initialize state estimate and covariance
- Prediction step: propagate state and covariance using linearized dynamics
- Update step: compute Kalman gain and update estimate with measurements

### Unscented Kalman Filter (UKF)
- Generate sigma points from current estimate and covariance
- Propagate sigma points through nonlinear dynamics
- Compute predicted mean and covariance
- Update with measurements using unscented transform

### Particle Filter (PF)
- Initialize particle cloud from prior distribution
- Prediction: propagate particles through dynamics with process noise
- Update: compute particle weights based on measurement likelihood
- Resample particles to avoid degeneracy

### Particle Filter Variants (Optional)
- Auxiliary Particle Filter
- Regularized Particle Filter
- Rao-Blackwellized Particle Filter

## Expected Outcomes

Upon completion, the notebook should produce:

1. **State estimation plots**: Comparison of EKF/UKF/PF estimates vs ground truth
   - Position tracking (3D trajectory)
   - Velocity tracking (body frame components)
   - Attitude tracking (Euler angles from quaternion)
   - Angular rate tracking

2. **Performance metrics**:
   - Root Mean Square Error (RMSE) for each state component
   - Computation time comparison
   - Filter consistency analysis (innovation statistics)

3. **Analysis**: Discussion of trade-offs between filters
   - Accuracy vs computational cost
   - Robustness to nonlinearity

## References

- Aircraft simulation: [CogniPilot auav_pylon_2025](https://github.com/CogniPilot/auav_pylon_2025)
- Dynamics formulation: Night Vapor fixed-wing platform
