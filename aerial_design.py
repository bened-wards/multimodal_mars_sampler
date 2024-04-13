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

MACH_FORWARD_FLIGHT_LIMIT = 0.95

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
fixed_mass = 6 # TODO: make this something reasonable - maybe function of total mass (heavier drone requires more structure?)


def hover_design(design_mass, no_rotors, no_blades = 2):
    # STEP 1: total mass
    total_mass = design_mass * (1 - CONTINGENCY_WEIGHT_FACTOR)

    # STEP 2: thrust
    thrust = HOVER_THRUST_CONDITION * total_mass * MARS_GRAVITY
    thrust_per_rotor = thrust / no_rotors

    # STEP 3: rotor radius
    tip_speed = SPEED_OF_SOUND * MACH_BLADE_TIP_LIMIT
    blade_area = thrust_per_rotor / (DENSITY * blade_loading * tip_speed**2)
    # TODO: find relationship between blade area and rotor radius:
    rotor_radius = blade_area / 4
    disk_area = rotor_radius**2 * np.pi

    if rotor_radius * 2 * no_rotors > MAX_DIAMETER:
        raise ValueError("Cannot create required thrust to fit in aeroshell.")

    # STEP 5: thrust power requirements
    thrust_power_per_rotor = induced_power_factor * thrust_per_rotor * np.sqrt(thrust_per_rotor / (2 * DENSITY * disk_area)) + DENSITY * blade_area * tip_speed**3 * drag_coef_mean / 8
    thrust_power = thrust_power_per_rotor * no_rotors

    # STEP 6: torque
    rotational_speed = tip_speed / rotor_radius
    torque = thrust_power / propulsive_efficiency / rotational_speed

    # STEP 6.5(haha): Flight performance
    
    # Forward flight speed
    advancing_blade_speed = MACH_FORWARD_FLIGHT_LIMIT*SPEED_OF_SOUND
    forward_velocity = advancing_blade_speed - tip_speed

    # Hover power
    hover_power = thrust_power / HOVER_THRUST_CONDITION / propulsive_efficiency

    # Forward flight power
    f_flight_power = 1.2*hover_power # TODO: Not sure if this is actually the case just put it in for now

    # Take off power requirement: Climb rate to initial 10m altitude gives time for climb -> multiply full power by time to climb gives power consumption for climb
    climb_rate = 2 #m/s
    takeoff_alt = 20 #m
    takeoff_time = takeoff_alt / climb_rate
    takeoff_power = ( thrust_power / propulsive_efficiency ) * takeoff_time # Joules (I think if were using Watts for Power? I'm not too sure on battery drain stuff might need some help with that)

    # Landing power requirement
    landing_power = 0.9*takeoff_power # TODO: Do some research into this, not really sure how to do it

    # Battery power
    battery_mass_proportion = 0.3 # TODO: This is a guess (Maybe a variable for optimisation?)
    battery_mass = total_mass*battery_mass_proportion
    battery_power = battery_density*battery_mass * 60*60 # Joules

    # Hover flight time: 
    usable_battery_power = battery_power*usable_battery
    hover_time = (usable_battery_power-takeoff_power-landing_power) / hover_power

    # Forward flight:
    forward_flight_time = (usable_battery_power-takeoff_power-landing_power) / f_flight_power
    flight_distance = forward_velocity * forward_flight_time

    # STEP 7: energy per sol
    avionics_energy = avionics_power * SOL_SECONDS
    sleep_energy = 15 * total_mass**(1/3) # TODO: is this correct interpretation from NASA MSH paper? 
    energy = hover_power * HOVER_TIME + sleep_energy + avionics_energy  # TODO: If we think my method above is better then HOVER_time becomes hover_time and we can determine the time to charge battery for flight etc
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
    

# analysis (for each design)

# initial verification using design_mass = MASS_LIMIT

# payload efficiency tradeoff
# - vary design_mass and calculate payload/design_mass

# payload vs hover/range tradeoff
# - fix design_mass (maybe at MASS_LIMIT, maybe at maximum payload efficiency) 
# - decrease the proportion of payload for sampling and move into battery to provide more energy/power
# - back calculate HOVER_TIME or FORWARD_FLIGHT_RANGE from total_power
