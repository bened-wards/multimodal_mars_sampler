from rotorcraft import ConventionalRotorcraft, CoaxialRotorcraft, TiltRotorcraft
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions
import matplotlib.pyplot as plt
import numpy as np
import csv
import logging

MASS_LIMIT = 50
mission_scenario = FlightMissionScenario(
        hover_time=3*60, forward_flight_distance=600, climb_height=50, 
        climb_rate=10, descent_rate=10)
design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
design_assumptions = DesignAssumptions()
coaxial = CoaxialRotorcraft("Coaxial helicopter", 2, 4, mission_scenario, design_constraints, design_assumptions, logging.DEBUG)
quad_coaxial = CoaxialRotorcraft("Quad coaxial", 8, 4, mission_scenario, design_constraints, design_assumptions, logging.DEBUG)
design_mass = 30
payload = coaxial.calc_and_verify_initial_design(design_mass)
print(f"{coaxial.name}: {payload:.2f}kg. Diameter: {coaxial.total_diameter:.3f}m")

payload = quad_coaxial.calc_and_verify_initial_design(design_mass)
print(f"{quad_coaxial.name}: {payload:.2f}kg. Diameter: {quad_coaxial.total_diameter:.3f}m")
