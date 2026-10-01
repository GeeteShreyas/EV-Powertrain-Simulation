# EV Powertrain & Battery Performance Simulation

MATLAB-based simulation of an electric vehicle powertrain using the WLTC Class 3 driving cycle.

## Project Overview

This project estimates vehicle forces, wheel power, battery power, energy consumption, and battery state of charge (SOC) during a WLTC Class 3 driving cycle.

It also investigates how changes in vehicle mass, aerodynamic drag coefficient, and rolling resistance affect energy consumption.

## Files

- `EV_powertrain_model.m` — Main MATLAB simulation script
- `WLTC_Class3.csv` — WLTC Class 3 driving-cycle input data
- `Results/` — Generated simulation plots

## Main Outputs

- Vehicle speed and acceleration
- Total driving force
- Battery power demand and regenerative braking power
- Battery state of charge
- Total energy consumption
- Sensitivity analysis for mass, drag coefficient, and rolling resistance

## Model Assumptions

- The vehicle follows the WLTC Class 3 speed profile exactly.
- Road gradient is assumed to be zero (flat road).
- Air density is assumed constant.
- Drivetrain efficiency is constant during traction.
- Regenerative braking is applied during negative wheel-power demand.
- Auxiliary loads such as HVAC, lights, and infotainment are not included.
- Battery voltage and internal resistance effects are not modeled.


## Governing Equations

The total tractive force is calculated as:

Ftotal = Frolling + Faerodynamic + Facceleration

Where:

- Rolling resistance: `Frolling = m × g × Crr`
- Aerodynamic drag: `Faerodynamic = 0.5 × ρ × Cd × A × v²`
- Acceleration force: `Facceleration = m × a`
- Wheel power: `Pwheel = Ftotal × v`

Battery power is calculated using drivetrain efficiency during traction and regenerative efficiency during braking.

## Key Findings

- Higher vehicle mass increases acceleration energy demand.
- Aerodynamic drag becomes more important at higher speeds.
- Rolling resistance affects energy use throughout the driving cycle.
- Regenerative braking recovers part of the vehicle’s kinetic energy during deceleration.
