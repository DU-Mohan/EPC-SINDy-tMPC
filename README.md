# EPC-SINDy–tMPC microgrid demo

Simulation demo accompanying the submitted manuscript **“Physics-Consistent Nonlinear Identification and Robust Control for Frequency and Voltage Regulation in Inverter-Based Microgrids.”**

The paper is currently under review. Key control and measurement blocks are provided as protected Simulink models (`.slxp`), and MATLAB algorithms are provided as P-code (`.p`). Their editable source implementations will be officially released after publication.

## Main files

- `ini.m`: initialization and simulation parameters.
- `model.slx`: microgrid simulation model.
- `run_model.mlx`: MATLAB live script for identification, control simulations, and plotting.
- `utils/`: supporting functions, shared library, and protected models.
- `data/`: supplied RL weights; simulation data and identified parameters are generated locally.
- `figures/`: figures generated locally by the live script.

## Run

Open `run_model.mlx` in MATLAB's Live Editor and run its sections from top to bottom. The script runs `ini.m` and sets the required paths. RL uses the supplied weights; retraining is not required.

The workflow generates the manuscript's **Figures 5 and 6**, exported as `figures/tmpc.pdf` and `figures/benchmark.pdf`, respectively. The first simulation can take several minutes to compile.

Generated simulation data, figures, and build caches are excluded from Git. Keep `data/trained_rl.mat` and the two support MAT files in `utils/interfaces` when sharing the demo.

## Requirements

Tested on **Windows 64-bit with MATLAB/Simulink R2024a Update 3**. Required toolboxes: Simscape, Simscape Electrical (Specialized Power Systems), Control System Toolbox, Optimization Toolbox, and Signal Processing Toolbox. A configured C/C++ compiler is needed for the simulation builds. Other releases and platforms have not been validated.

Keep the folder structure intact and avoid adding another copy of the project to the MATLAB path. The supplied protected models have fixed sample times, prediction horizons, and model dimensions.

License: [MIT](LICENSE).
