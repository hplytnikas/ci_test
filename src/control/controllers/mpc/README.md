## MPC

Model Predictive Control.

Used in the 2024 season mainly for AutoX and Trackdrive Formula Student disciplines.
By incorporating a model and iteratively (in real time) optimizing the control inputs for a time-horizon, the controller decides on the lateral and longitudinal control inputs that are optimal also for the near future.

MPC usually consists of:
- Model
- Constraints --> Safety and recursive feasibility
- Cost --> Desired Behavior and
- Solver

In the case of DV-control in 2024, we have the following folder structure:

mpc
├── mpc_controller *(where the magic happens)*
├── mpc_geometry *(a class that handles geometric interpolations for the MPC)*
├── mpc_visualisation_node *(just visualization stuff)*
└── solvers *(here one can add other solvers that inherit from the common solver)*
    ├── acados_solver
    ├── embotech_solver
    ├── mpc_solver_common *(abstract interface that defines a solver)*
    └── codegenerated_solvers *(submodule with generated solvers)*
