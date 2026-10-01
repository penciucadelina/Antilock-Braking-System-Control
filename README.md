# ABS Slip Control using System Identification and Model Predictive Control (MPC)

## Overview
This project presents the modeling, identification, and control of an Anti-lock Braking System (ABS) in MATLAB/Simulink. 
The main objective was to regulate wheel slip in the optimal adhesion region during braking, preventing wheel lock while preserving vehicle stability and steering capability. The project follows a complete control-engineering workflow, from simulation-based data acquisition and system identification to advanced constrained predictive control and closed-loop validation.

## Control Objective
The target was to maintain the slip ratio at the optimal value of **0.8**, ensuring maximum road adhesion, while keeping the braking command within the physical actuator constraints $u \in [0, 1]$ (where 0 means no braking and 1 means full braking).

## Main Contributions
* Acquired braking and slip data from the **INTECO ABS** simulation environment.
* Built a highly accurate second-order discrete state-space model from simulation data.
* Evaluated identification methods and successfully applied the **Output-Error (OE)** method, further refined with the **Prediction Error Minimization (PEM)** algorithm.
* Designed and implemented a **Model Predictive Controller (MPC)** for optimal slip tracking.
* Applied physical actuator limits and rate-of-change penalties to the brake command to ensure smooth and realistic actuation.
* Validated the final control solution in both MATLAB (script-based) and Simulink (block-based) environments.

## Workflow
1. **Data Acquisition:** Extracting open-loop step response data from the INTECO ABS model in Simulink.
2. **System Identification:** Processing input-output slip data using OE and PEM algorithms.
3. **State-Space Conversion:** Extracting the A, B, C, D matrices for predictive control design.
4. **MPC Design:** Tuning prediction horizons, control horizons, and cost function weights.
5. **Closed-Loop Validation:** Simulating the controlled system in MATLAB and the full nonlinear ABS model in Simulink.

## System Identification
The controller design starts from ABS simulation data exported from Simulink:
* **Input:** Braking signal (step input)
* **Output:** Measured wheel slip

The system was identified using the Output-Error (OE) structure of order [2 2 0] and fine-tuned using the PEM algorithm. The resulting discrete mathematical model achieved a **fit of 85.14%** to the validation data. This high-fidelity model was then converted into a state-space representation, making it directly suitable for MPC design by providing the internal dynamic matrices.

## Model Predictive Control (MPC) Strategy
MPC was chosen as the control strategy because it offers the best overall balance between tracking accuracy, smooth control action, and constraint handling. For an ABS application, this is critical because the braking command must remain physically feasible while the slip ratio must stay in the optimal adhesion region.

### MPC Design Parameters
The final MPC controller was configured with:
* **Slip reference:** 0.8
* **Prediction horizon (P):** 30 samples (~0.27s)
* **Control horizon (M):** 3 samples
* **Brake constraints:** [0, 1]
* **Output Weight:** 2 (prioritizes strict slip tracking)
* **Manipulated Variable Rate Weight:** 0.3 (penalizes abrupt brake variations, ensuring a smooth mechanical response)

## Results
The project demonstrated strong closed-loop performance in both MATLAB and Simulink validations:
* **Fast Response:** Settling time for the discrete model was under 0.4 seconds. In the full Simulink closed-loop, the system stabilized at the reference in approximately 0.85 seconds.
* **Minimal Overshoot:** The peak deviation was kept under 5%.
* **Constraint Adherence:** The MPC command correctly saturated at the maximum physical limit without causing instability.
* **Overall Performance:** The controlled system accurately tracked the 0.7-0.8 slip target without wheel lock, performing significantly better than an uncontrolled emergency braking scenario.

## Tools and Technologies
* **MATLAB & Simulink** (R2023b)
* System Identification Toolbox
* Model Predictive Control Toolbox
* INTECO ABS Laboratory System Simulator
