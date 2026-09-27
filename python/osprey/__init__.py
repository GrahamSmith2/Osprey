"""Osprey USV manoeuvring model, rudder sizing and Lua control bridge.

Python port of the MATLAB model in the repo root (params/, model/, analysis/).
See python/README.md for usage and python/PORTING.md for what changed.
"""
from .params import (AS_BUILT, PROPOSED, DEG, INCH, KGCM, Rudder, Vessel,
                     Environment, Actuators, Params, build_params,
                     sample_uncertainty, UNCERTAINTY)
from .sim import simulate, trim_state, Fault, SimResult

__all__ = ["AS_BUILT", "PROPOSED", "DEG", "INCH", "KGCM", "Rudder", "Vessel",
           "Environment", "Actuators", "Params", "build_params",
           "sample_uncertainty", "UNCERTAINTY", "simulate", "trim_state",
           "Fault", "SimResult"]
