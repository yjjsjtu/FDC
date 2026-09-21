"""
DummyX2 Robot - Real-time Torque Compensation using Pinocchio

This program implements real-time dynamics-based torque compensation for DummyX2:
- Uses Pinocchio for inverse dynamics computation
- Controls RdriveS1 motors via CAN
- Supports harmonic reducers with configurable ratios
- Provides gravity compensation and full dynamics compensation
"""

Command
`python3 dynamics_test.py`