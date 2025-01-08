# 5barLinkage

- This repository contains a Julia script that derives and simulates the nonlinear state-space equations of motion of a 5 bar linkage mechanism, actuated at its base. Aim is to output data to be used by SINDy-PI

The derivation of the dynamic model is detailed in [the jpg.](./hand_derivation.jpg)
Take a look at src/main.jl for the step by step implementation.

## Preliminaries
[Install Julia](https://docs.julialang.org/en/v1/manual/installation/)

From a command prompt, run Julia

```bash
julia
```

## To run the code from cmd prompt:
Clone the repository and move to its directory.

Run the script:

```bash
julia main.jl
```
This can be run from VSCode, but make sure to add the Julia extension to VSCode first.

This commented script runs through the model derivation making use of the Julia Symbolics.jl toolbox
Then it simulates the unconstrained system's free response (no motor torque). This is just a swinging 4-link pendulum:

 <img src="./plots/unconstrained_system.gif" alt="pendulum_response_gif" width="350"/>
 
Then it simulates the constrained linkage system in response to chosen motor torque profile.

<img src="./plots/constrained_system.gif" alt="constrained_motion_gif" width="480"/>

## Next steps...
- put in correct linkage parameters.
- Generate $[\dot{X}]$  data array. I think using Acausal filtering technique such as Savitzky Golay algorithm.
