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
This can be run from VSCode, but make sure to add the Julia extension first.

This commented script runs through the model derivation and simulates the system's free response (no motor torque) from chosen initial conditions  

<img src="./anims/rotary_pendulum_anim.gif" alt="response_gif" width="480"/> <img src="./plots/response.png" alt="pendulum response" width="350"/>

## Next steps...
- put in correct linkage parameters.
- Generate $[\dot{X}]$  data array. I think using Acausal filtering technique such as Savitzky Golay algorithm.
