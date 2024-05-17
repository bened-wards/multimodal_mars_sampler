from rotorcraft import ConventionalRotorcraft, CoaxialRotorcraft, TiltRotorcraft, Rotorcraft
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions
import matplotlib.pyplot as plt
import numpy as np
import csv
import logging

def print_chars_with_error_bars(aircraft: Rotorcraft):
    print("CHARACTERISTICS")
    print(aircraft.name)
    print(f"{0.9 * aircraft._rotor_radius:.3f} : {aircraft._rotor_radius:.3f} : {1.1 * aircraft._rotor_radius:.3f}")
    print(f"{0.9 * aircraft.total_diameter:.3f} : {aircraft.total_diameter:.3f} : {1.1 * aircraft.total_diameter:.3f}")
    print(f"{aircraft._no_rotors}")
    print(f"{aircraft._no_blades}")
    print(f"{aircraft._blade_area * aircraft._no_blades * aircraft._no_rotors:.3f}")
    disk_area = aircraft._rotor_radius**2 * np.pi * aircraft._no_nonoverlapping_rotors
    print(f"{0.9 * disk_area:.3f}: {disk_area:.3f} : {1.1 * disk_area:.3f}")
    print(f"{0.9 * aircraft._design_mass / disk_area:.3f} : {aircraft._design_mass / disk_area:.3f} : {1.1 * aircraft._design_mass / disk_area:.3f}")
    print(f"{0.9 * aircraft._design_constraints.TIP_SPEED_LIMIT:.2f} : {aircraft._design_constraints.TIP_SPEED_LIMIT:.2f} : {1.1 * aircraft._design_constraints.TIP_SPEED_LIMIT:.2f}")
    print(f"{0.9 * aircraft.motor_rpm_hover:.0f} : {aircraft.motor_rpm_hover:.0f} : {1.1 * aircraft.motor_rpm_hover:.0f}")
    print(f"{0.9 * aircraft._motor_power_spec/1000:.2f} : {aircraft._motor_power_spec/1000:.2f} : {1.1 * aircraft._motor_power_spec/1000:.2f}")
    print(f"{0.9 * aircraft._solar_panel_area:.3f} : {aircraft._solar_panel_area:.3f} : {1.1 * aircraft._solar_panel_area:.3f}")
    print(f"{0.9 * aircraft._energy_required_Wh:.3f} : {aircraft._energy_required_Wh:.3f} : {1.1 * aircraft._energy_required_Wh:.3f}")

def print_mass_with_error_bars(aircraft: Rotorcraft):
    print("MASS")
    print(aircraft.name)
    print(f"{aircraft._design_mass:.3f}")
    contingency = aircraft._design_mass*aircraft._design_constraints.CONTINGENCY_WEIGHT_FACTOR
    print(f"{0.9 * contingency:.3f} : {contingency:.3f} : {1.1 * contingency:.3f}")
    print(f"{0.9 * aircraft._motor_mass:.3f} : {aircraft._motor_mass:.3f} : {1.1 * aircraft._motor_mass:.3f}")
    print(f"{0.9 * aircraft._battery_mass:.3f} : {aircraft._battery_mass:.3f} : {1.1 * aircraft._battery_mass:.3f}")
    print(f"{0.9 * aircraft._solar_panel_mass:.3f} : {aircraft._solar_panel_mass:.3f} : {1.1 * aircraft._solar_panel_mass:.3f}")
    print(f"{0.9 * aircraft._rotor_mass:.3f} : {aircraft._rotor_mass:.3f} : {1.1 * aircraft._rotor_mass:.3f}")
    print(f"{0.9 * aircraft._structure_mass:.3f} : {aircraft._structure_mass:.3f} : {1.1 * aircraft._structure_mass:.3f}")
    print(f"{0.9 * aircraft._ground_mobility_mass:.3f} : {aircraft._ground_mobility_mass:.3f} : {1.1 * aircraft._ground_mobility_mass:.3f}")
    print(f"{0.9 * aircraft._flight_electronics_mass:.3f} : {aircraft._flight_electronics_mass:.3f} : {1.1 * aircraft._flight_electronics_mass:.3f}")
    print(f"{0.9 * aircraft._empty_mass + contingency:.3f} : {aircraft._empty_mass + contingency:.3f} : {1.1 * aircraft._empty_mass + contingency:.3f}")
    print(f"{0.9 * aircraft._payload:.3f} : {aircraft._payload:.3f} : {1.1 * aircraft._payload:.3f}")

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
    print(f"{aircraft._design_constraints.TIP_SPEED_LIMIT:.2f}")
    print(f"{aircraft.motor_rpm_hover:.0f}")
    print(f"{aircraft._motor_power_spec/1000:.2f}")
    print(f"{aircraft._solar_panel_area:.3f}")
    print(f"{aircraft._energy_required_Wh:.3f}")

def print_mass(aircraft: Rotorcraft):
    print("MASS")
    print(aircraft.name)
    print(f"{aircraft._design_mass:.3f}")
    contingency = aircraft._design_mass*aircraft._design_constraints.CONTINGENCY_WEIGHT_FACTOR
    print(f"{contingency:.3f}")
    print(f"{aircraft._motor_mass:.3f}")
    print(f"{aircraft._battery_mass:.3f}")
    print(f"{aircraft._solar_panel_mass:.3f}")
    print(f"{aircraft._rotor_mass:.3f}")
    print(f"{aircraft._structure_mass:.3f}")
    print(f"{aircraft._ground_mobility_mass:.3f}")
    print(f"{aircraft._flight_electronics_mass:.3f}")
    print(f"{aircraft._empty_mass + contingency:.3f}")
    print(f"{aircraft._payload:.3f}")

MSH_ROTOR_AREA_FACTOR = 0.1424
MASS_LIMIT = 50

mission_scenario = FlightMissionScenario(
    hover_time=2*60, forward_flight_speed=30, forward_flight_distance=1000, 
    climb_height=200, climb_rate=10, descent_rate=10)
design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=1000)
design_assumptions = DesignAssumptions()
design_assumptions.BLADE_LOADING = 0.11 # NASA MSH paper
design_assumptions.DRAG_COEF_MEAN = 0.05 # read from graph in NASA MSH paper
design_constraints.MACH_TIP_LIMIT = 0.7
design_constraints.MACH_ADVANCING_TIP_LIMIT = 0.83
design_assumptions.AVIONICS_POWER = 0
design_assumptions.SAMPLING_MECHANISM_POWER = 0
design_assumptions.SAMPLING_TIME = 0
design_assumptions.GROUND_MOBILITY_POWER = 0
design_assumptions.GROUND_MOBILITY_TIME = 0
design_assumptions.GROUND_MOBILITY_MASS_PROPORTION = 0

# coaxial specific
design_constraints.CONTINGENCY_WEIGHT_FACTOR = 0.177462289
design_assumptions.ELECTRONICS_MASS = 2.674

coaxial_design_mass = 18.032
coaxial = CoaxialRotorcraft("MSH Coaxial", 2, 4, mission_scenario, design_constraints, design_assumptions)
coaxial._rotor_blade_area_factor = MSH_ROTOR_AREA_FACTOR
coaxial_payload = coaxial.calc_and_verify_initial_design(coaxial_design_mass)
print(f"{coaxial.name} ({coaxial_design_mass:.2f}kg): {coaxial_payload:.4f}kg. Diameter: {coaxial.total_diameter:.3f}m")
print_chars(coaxial)
print_mass(coaxial)

# hex specific
design_constraints.CONTINGENCY_WEIGHT_FACTOR = 0.177046767
design_assumptions.ELECTRONICS_MASS = 2.644

hex_design_mass = 17.662
hex = ConventionalRotorcraft("MSH Hex", 6, 4, mission_scenario, design_constraints, design_assumptions)
hex._rotor_blade_area_factor = MSH_ROTOR_AREA_FACTOR
hex_payload = hex.calc_and_verify_initial_design(hex_design_mass)
print(f"{hex.name} ({hex_design_mass:.2f}kg): {hex_payload:.4f}kg. Diameter: {hex.total_diameter:.3f}m")
print_chars(hex)
print_mass(hex)
