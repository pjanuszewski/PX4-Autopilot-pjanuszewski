# PX4-Autopilot with Control Toolbox Integration

## Overview
This repository integrates the **Control Toolbox (CT)** library into the **PX4 Autopilot** environment. The main goal is to demonstrate the feasibility of running Control Toolbox functions, such as **LQR computation** (Riccati equation), during the runtime of PX4 simulations.

The `lqr_runtime` branch successfully demonstrates the real-time calculation of the **LQR K matrix** during simulation, though it does not yet produce the desired trajectory-following behavior for the drone.

## Features
- **LQR Runtime Calculation**:
  - The `lqr_runtime` branch proves that the **Riccati equation** and **LQR K matrix** can be computed in real time during PX4 simulations.
  - This setup can serve as a reference for implementing LQR or other control methods in PX4.
  - The `lqr_about_trajectory` branch includes efforts to achieve precise trajectory-following and demonstrates that LQR can be used at a high frequency during runtime based on current states, but the simulation does not yet meet the desired results.

- **Control Toolbox Integration**:
  - **Control Toolbox** is used as a submodule. After running `make distclean`, reinitialize the submodule with:
    ```bash
    git submodule update --init --recursive
    ```

## Getting Started
1. **Clone the Repository**:
    ```bash
    git clone --recurse-submodules https://github.com/your-username/PX4-Autopilot.git
    cd PX4-Autopilot
    ```

2. **Build PX4**:
    ```bash
    make px4_sitl_default
    ```

3. **If You Run `make distclean`**:
   - After running `make distclean`, you will need to reinitialize the Control Toolbox submodule:
     ```bash
     git submodule update --init --recursive
     ```

## Using the LQR Functionality
- LQR-related functions have been copied to the `src/modules/mc_att_control/` folder and modified.
- The functions demonstrate the computation of **LQR matrices** during simulation but require further adaptation for specific control tasks.
- **Note**: Modifications are needed due to PX4’s stricter compiler standards. Common issues include uninitialized variables or handling floating-point (`wfloat`) warnings.

## Known Issues
- The current implementation in the `lqr_about_trajectory` branch does not achieve precise trajectory-following for the drone.
- **Strict compiler standards** in PX4 require adaptations of the Control Toolbox functions (e.g., handling uninitialized variables or floating-point (`wfloat`) warnings).
- Users may need to make additional modifications when integrating LQR or other control methods.

## Future Plans
- **Refine LQR Implementation**: Continue improving the LQR calculations for better control performance.
- **Model Predictive Control (MPC)**: The long-term goal is to integrate **MPC** into the PX4 environment, leveraging the Control Toolbox.

## Contributing
Contributions and feedback are welcome! If you have insights or improvements, feel free to open an issue or submit a pull request.

## License
This project is licensed under multiple licenses due to the inclusion of code from different sources:

- **PX4 Autopilot**: Licensed under the **BSD 3-Clause License**.

- **Control Toolbox**: Licensed under the **BSD 2-Clause License**.

## Disclaimer
This repository is a modified version of the original PX4 Autopilot, integrating the Control Toolbox for research and development purposes. It is not guaranteed to be stable or production-ready.
