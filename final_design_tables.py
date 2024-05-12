from rotorcraft import ConventionalRotorcraft, CoaxialRotorcraft, TiltRotorcraft, Rotorcraft
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

def print_chars(aircraft: Rotorcraft):
    print("CHARACTERISTICS")
    print(aircraft.name)
    print(f"{aircraft._rotor_radius:.3f}")
    print(f"{aircraft.total_diameter:.3f}")
    print(f"{aircraft._no_rotors}")
    print(f"{aircraft._no_blades}")
    print(f"{aircraft._blade_area * aircraft._no_blades * aircraft._no_rotors:.3f}")
    disk_area = aircraft._rotor_radius**2 * np.pi * aircraft._no_nonoverlapping_rotors
    print(f"{disk_area:.3f}")
    print(f"{aircraft._design_mass / disk_area:.3f}")
    print(f"{design_constraints.TIP_SPEED_LIMIT:.2f}")
    print(f"{aircraft.motor_rpm_hover:.0f}")
    print(f"{aircraft._motor_power_spec/1000:.2f}")
    print(f"{aircraft._solar_panel_area:.3f}")
    print(f"{aircraft._energy_required_Wh:.3f}")

def print_mass(aircraft: Rotorcraft):
    print("MASS")
    print(aircraft.name)
    print(aircraft._design_mass)
    print(f"{aircraft._design_mass*design_constraints.CONTINGENCY_WEIGHT_FACTOR:.2f}")
    print(f"{aircraft._motor_mass:.2f}")
    print(f"{aircraft._battery_mass:.2f}")
    print(f"{aircraft._solar_panel_mass:.2f}")
    print(f"{aircraft._rotor_mass:.2f}")
    print(f"{aircraft._structure_mass:.2f}")
    print(f"{aircraft._ground_mobility_mass:.2f}")
    print(f"{aircraft._flight_electronics_mass:.2f}")
    print(f"{aircraft._empty_mass:.2f}")
    print(f"{aircraft._payload:.2f}")

coaxial2 = CoaxialRotorcraft("Coaxial helicopter - 2", 2, 2, mission_scenario, design_constraints, design_assumptions)
quad_coaxial4 = CoaxialRotorcraft("Quad coaxial - 4", 8, 4, mission_scenario, design_constraints, design_assumptions)
quad_coaxial2 = CoaxialRotorcraft("Quad coaxial - 2", 8, 2, mission_scenario, design_constraints, design_assumptions)


# minimise design mass for minimum payload
quad_design_mass = 17.27
quad_payload = quad_coaxial2.calc_and_verify_initial_design(quad_design_mass)
print(f"{quad_coaxial2.name} ({quad_design_mass:.2f}kg): {quad_payload:.4f}kg. Diameter: {quad_coaxial2.total_diameter:.3f}m")
print_chars(quad_coaxial2)
print_mass(quad_coaxial2)

# midpoint
design_mass = 30
coaxial_payload = coaxial2.calc_and_verify_initial_design(design_mass)
quad_payload = quad_coaxial4.calc_and_verify_initial_design(design_mass)
print(f"{coaxial2.name} ({design_mass:.2f}kg): {coaxial_payload:.2f}kg. Diameter: {coaxial2.total_diameter:.3f}m")
print(f"{quad_coaxial4.name} ({design_mass:.2f}kg): {quad_payload:.2f}kg. Diameter: {quad_coaxial4.total_diameter:.3f}m")
print_chars(coaxial2)
print_chars(quad_coaxial4)
print_mass(coaxial2)
print_mass(quad_coaxial4)

# maximise payload
design_mass = 50
quad_payload = quad_coaxial4.calc_and_verify_initial_design(design_mass)
print(f"{quad_coaxial4.name} ({design_mass:.2f}kg): {quad_payload:.2f}kg. Diameter: {quad_coaxial4.total_diameter:.3f}m")
print_chars(quad_coaxial4)
print_mass(quad_coaxial4)


