import numpy as np

# constants
MARS_GRAVITY = 3.71
SPEED_OF_SOUND = 240 # NASA MSH has this as 233.1
DENSITY = 0.0175
SOL_SECONDS = 24 * 60 * 60 + 39 * 60 + 35.244

# constraints
MASS_LIMIT = 50 # kg
CONTINGENCY_WEIGHT_FACTOR = 0.2 # 20%

HOVER_THRUST_CONDITION = 1.5 # 150%

MACH_BLADE_TIP_LIMIT = 0.8 

MAX_DIAMETER = 4.35 # metres

HOVER_TIME = 5 * 60 # seconds

# assumptions / design decisions
blade_loading = 0.175 # coefficient of thrust / solidity. From ROAMX rotor
drag_coef_mean = 1.02 / 28.25 # from Table 4 ROAMX motor paper: https://ntrs.nasa.gov/api/citations/20240000766/downloads/1689_Koning_Final_011924.pdf
figure_of_merit = 0.65 # from Table 3 ROAMX motor paper: https://ntrs.nasa.gov/api/citations/20240000766/downloads/1689_Koning_Final_011924.pdf

induced_power_factor = 1.2 # from NASA MSH paper

# assumptions from NASA MSH paper
motor_efficiency = 0.8 # 80%
rotor_servo_power = 0.15 # 15%
propulsive_efficiency = motor_efficiency * (1 - rotor_servo_power)

avionics_power = 35 # W - NASA MSH paper
solar_panel_power_density = 21.9 # W/m^2 - NASA MSH paper based on MH
solar_panel_mass_density = 2.0 # kg/m^2
usable_battery = 0.7 # 10-80% depth-of-discharge
battery_density = 218.5 # Wh/kg - NASA MSH paper - JPL technology forecast TODO: see if there is better forecast
fixed_mass = 6 # TODO: make this something reasonable


def hover_design(no_rotors, no_blades = 2):
    # STEP 1: total mass
    total_mass = MASS_LIMIT * (1 - CONTINGENCY_WEIGHT_FACTOR)

    # STEP 2: thrust
    thrust = HOVER_THRUST_CONDITION * total_mass * MARS_GRAVITY
    thrust_per_rotor = thrust / no_rotors

    # STEP 3: rotor radius
    tip_speed = SPEED_OF_SOUND * MACH_BLADE_TIP_LIMIT
    blade_area = thrust_per_rotor / (DENSITY * blade_loading * tip_speed**2)
    # TODO: find relationship between blade area and rotor radius
    rotor_radius = blade_area / 2 
    disk_area = rotor_radius**2 * np.pi

    if rotor_radius * no_blades > MAX_DIAMETER:
        raise ValueError("Cannot create required thrust to fit in aeroshell.")

    # STEP 5: thrust power requirements
    thrust_power = induced_power_factor * thrust * np.sqrt(thrust / (2 * DENSITY * disk_area)) + DENSITY * blade_area * tip_speed**3 * drag_coef_mean / 8
    thrust_power_per_rotor = induced_power_factor * thrust_per_rotor * np.sqrt(thrust_per_rotor / (2 * DENSITY * disk_area)) + DENSITY * blade_area * tip_speed**3 * drag_coef_mean / 8

    # STEP 6: torque
    rotational_speed = tip_speed / rotor_radius
    torque_per_rotor = thrust_power_per_rotor / propulsive_efficiency / rotational_speed
    torque = torque_per_rotor * no_rotors

    # STEP 7: energy per sol
    avionics_energy = avionics_power * SOL_SECONDS
    sleep_energy = 15 * total_mass**(1/3) # TODO: is this correct interpretation from NASA MSH paper? 
    energy = thrust_power * HOVER_TIME + sleep_energy + avionics_energy
    total_power = energy / SOL_SECONDS

    # STEP 8: mass
    motor_mass = 0.076 * torque**0.86 # kg - NASA MSH (based on MH)

    solar_panel_area = total_power / solar_panel_power_density # m^2
    solar_panel_mass = solar_panel_area * solar_panel_mass_density # kg

    energy_Wh = energy / (60*60)
    energy_required_Wh = energy_Wh / usable_battery # TODO: did NASA MSH paper get this wrong and not actually consider usable_battery?
    battery_mass = energy_required_Wh / battery_density # NASA MSH paper

    rotor_mass = 1.1 * blade_area # NASA MSH paper. TODO: find rotor material density to make this function of rotor radius?
    # TODO shall we consider things like hub, shaft, support arms, fuselage -> or just include in fixed_mass

    empty_mass = motor_mass + solar_panel_mass + battery_mass + rotor_mass + fixed_mass

    # STEP 9: payload
    payload = total_mass - empty_mass
    if payload < 0:
        raise ValueError(f"Payload is less than zero: {payload:.2f}! Cannot lift anything.")
