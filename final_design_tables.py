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
coaxial = CoaxialRotorcraft("Coaxial helicopter", 2, 4, mission_scenario, design_constraints, design_assumptions)
quad_coaxial = CoaxialRotorcraft("Quad coaxial", 8, 4, mission_scenario, design_constraints, design_assumptions)

def print_chars(aircraft: Rotorcraft):
    return
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

# minimise design mass for minimum payload
coaxial_design_mass = 20.38
coaxial_payload = coaxial.calc_and_verify_initial_design(coaxial_design_mass)
quad_design_mass = 18.92
quad_payload = quad_coaxial.calc_and_verify_initial_design(quad_design_mass)
print(f"{coaxial.name} ({coaxial_design_mass:.2f}kg): {coaxial_payload:.4f}kg. Diameter: {coaxial.total_diameter:.3f}m")
print(f"{quad_coaxial.name} ({quad_design_mass:.2f}kg): {quad_payload:.4f}kg. Diameter: {quad_coaxial.total_diameter:.3f}m")
print_chars(coaxial)
print_chars(quad_coaxial)
print_mass(coaxial)
print_mass(quad_coaxial)

# midpoint
design_mass = 30
coaxial_payload = coaxial.calc_and_verify_initial_design(design_mass)
quad_payload = quad_coaxial.calc_and_verify_initial_design(design_mass)
print(f"{coaxial.name} ({coaxial_design_mass:.2f}kg): {coaxial_payload:.2f}kg. Diameter: {coaxial.total_diameter:.3f}m")
print(f"{quad_coaxial.name} ({quad_design_mass:.2f}kg): {quad_payload:.2f}kg. Diameter: {quad_coaxial.total_diameter:.3f}m")
print_chars(coaxial)
print_chars(quad_coaxial)
print_mass(coaxial)
print_mass(quad_coaxial)

# maximise payload
design_mass = 50
coaxial_payload = coaxial.calc_and_verify_initial_design(design_mass)
quad_payload = quad_coaxial.calc_and_verify_initial_design(design_mass)
print(f"{coaxial.name} ({coaxial_design_mass:.2f}kg): {coaxial_payload:.2f}kg. Diameter: {coaxial.total_diameter:.3f}m")
print(f"{quad_coaxial.name} ({quad_design_mass:.2f}kg): {quad_payload:.2f}kg. Diameter: {quad_coaxial.total_diameter:.3f}m")
print_chars(coaxial)
print_chars(quad_coaxial)
print_mass(coaxial)
print_mass(quad_coaxial)


