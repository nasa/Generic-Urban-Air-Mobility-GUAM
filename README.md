# NASA TTT-Autonomous Systems (AS): Intelligent Contingency Management (ICM)  
## Generic UAM Simulation  
### Version 1.1  

---

### Point of Contact:  
**Michael J. Acheson**  
NASA Langley Research Center (LaRC)  
Dynamics Systems and Control Branch (D-316)  
Email: michael.j.acheson@nasa.gov  

---

### Versions:
**Version 1.1** - 10.11.2023, MJA:  
- Incorporates expanded polynomial aero-propulsive database, trim tables, and gain scheduled baseline controller.  
- Includes trim scripts (see `./vehicles/Lift+Cruise/Trim/trim_helix.x`) and linearization scripts (see `./vehicles/Lift+Cruise/Control/ctrl_scheduler_GUAM.m`).  
- Adds piece-wise Bezier RefInput reference trajectory capability, and a simulation output animation script (`./utilities/Animate_SimOut.m`).  
- Enables a user-defined output script that allows specifying output variables (e.g., `./vehicles/Lift+Cruise/setup/User_SimOut/mySimOutFunc_Animate.m`).  

---

### Running a Simulation Example:
To run a simulation example case, execute the `RUNME.m` script at the top level!

---

### Simulation Overview:
This simulation is a generic UAM simulation, including a model representative of a NASA Lift+Cruise vehicle configuration.

#### Key Simulation Components:
1. **Simulation Architecture**:
   - Supports common rigid body 6-DOF frames of reference (e.g., Earth Centered Inertial, Earth Centered Earth Fixed (ECEF), North-East-Down (NED), Navigation, Velocity, Wind, Stability, and Body).
   - Contains most aerospace signals/quantities of typical interest.

2. **Generic Architecture**:
   - Supports swapping in and out aircraft models, sensors, actuator models, control algorithms, etc.

3. **Desired Trajectory or RefInputs**:
   - Supports ramps, timeseries, piece-wise Bezier curves, and doublets.

4. **Nominal Gain Scheduled (LQRi) Baseline Controller**:
   - Operates in the heading frame (NED frame rotated by the heading angle).

---

### Demonstration Trajectory Flights:
Found in the `Exec_Scripts` folder:
1. Simple sinusoidal input case (`./Exec_Scripts/exam_TS_Sinusoidal_traj.m`).
2. Basic lifting hover and transition to forward flight (`./Exec_Scripts/exam_TS_Hover2Cruise_traj.m`).
3. Cruise climbing right hand turn (`./Exec_Scripts/exam_TS_Cruise_Climb_Turn_traj.m`).
4. Takeoff, climbing transition to cruise and descending deceleration to landing using ramps.
5. Piece-wise Bezier curve trajectories:
   - Cruise descent and deceleration.
   - Hover climb and acceleration.

These demonstration trajectories can be performed by executing `RUNME.m` at the top level folder or by adding the `Exec_Scripts` folder to the MATLAB path and running the associated execution example script. 

---

### Simulation Input and Output:
- Output data is provided by the MATLAB logged signal `logsout{1}`.
- Analysis scripts make use of the output data assigned to a SimOut structure: `>>SimOut = logsout{1}.Values`.
- Simulation input (fixed) parameters are provided in a large structure `SimIn`, whereas desired tunable simulation parameters are provided using the large structure `SimPar`.

---

### User Variants and Configuration:
- Users can select subsystem variants through an input structure `userStruct.variants` and call the `simSetup.m` script to prepare the simulation for execution.
- Variants can be changed using the `userStruct` to specify various switches (see `./setup/setupVariantStruct.m` and `./setup/setupSwitches.m`).

Example of available `userStruct` options:
- userStruct.variants:
- refInputType: Timeseries
- vehicleType: LiftPlusCruise
- expType: DEFAULT
- atmosType: US_STD_ATMOS_76
- turbType: None
- ctrlType: BASELINE
- actType: FirstOrder
- propType: None
- fmType: Polynomial
- eomType: Simple
- sensorType: None


---

### Actuator Type Variants:
Available in the `ActuatorEnum.m` file:
classdef ActuatorEnum < Simulink.IntEnumType
enumeration
None(1)
FirstOrder(2)
SecondOrder(3)
FirstOrderFailSurf(4)
end
end


---

### Trajectories and Maneuvers:
- `userStruct.variant.refInputType` option selects between FOUR_RAMP, ONE_RAMP, TIMESERIES, BEZIER, and DEFAULT (doublets) options.
- Example: FOUR_RAMP in `Exec_Scripts/exam_RAMP.m`.

---

### Aero-Propulsive Model Options:
Two options available:
1. **Low-fidelity strip-theory model** (`SFunction` variant).
2. **Mid-fidelity polynomial model** (`Polynomial` variant).

References:
1. Cook, J. W., and Hauser, J., "A Strip Theory Approach to Dynamic Modeling of eVTOL Aircraft," AIAA SciTech 2021 Forum, AIAA Paper 2021-1720, Jan. 2021. [DOI](https://doi.org/10.2514/6.2021-1720).
2. Simmons, B. M., Buning, P. G., and Murphy, P. C., “Full-Envelope Aero-Propulsive Model Identification for Lift+Cruise Aircraft Using Computational Experiments,” AIAA AVIATION 2021 Forum, AIAA Paper 2021-3170, Aug. 2021. [DOI](https://doi.org/10.2514/6.2021-3170).

---

### Simulation Execution Performance:
Viewed via Simulink scopes in the `VehicleSimulation/Vehicle Generalized Control/Lift+Cruise/Control/Baseline` block.

---

### Trimming and Gain Scheduling:
- Trimming: `./vehicles/Lift+Cruise/Trim/trim_helix.m`.
- Gain Scheduling: `./vehicles/Lift+Cruise/control/ctrl_scheduler_GUAM.m`.

---

### Notices:
**Copyright 2024 United States Government**  
All Rights Reserved.

**Third-Party Software**:
- MATLAB®/SIMULINK® by The MathWorks, Inc.
- Users must supply their own licenses: [MathWorks](https://www.mathworks.com).

**Disclaimers**:
- **No Warranty**: Provided "as is" without any warranty.
- **Waiver and Indemnity**: Users waive claims against the U.S. Government and its contractors.

---
