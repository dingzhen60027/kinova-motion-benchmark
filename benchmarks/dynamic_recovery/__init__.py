"""Closed-loop motion recovery benchmark for an in-motion Kinova Gen3.

Every task starts from a predefined moving robot state. Independently moving
obstacles disturb the executing trajectory, and the evaluated method replans
online while the simulation advances by the method's measured computation
time. The robot model, collision geometry, trajectory format and MuJoCo
exporter are the frozen ``generic_motion_v1`` components.
"""
BENCHMARK = "kinova7_dynamic_recovery_v1"
