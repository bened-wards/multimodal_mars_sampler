from rotorcraft import ConventionalRotorcraft, CoaxialRotorcraft, TiltRotorcraft
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions
import matplotlib.pyplot as plt
import numpy as np
import csv

def msh_test():
    print("Hex Mars Science Helicopter")
    # mission setup
    MASS_LIMIT = 17.66
    mission_scenario = FlightMissionScenario(
        hover_time=2*60, forward_flight_speed=30, forward_flight_distance=1000, 
        climb_height=200, climb_rate=10, descent_rate=10)
    design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
    design_assumptions = DesignAssumptions()
    design_assumptions.BLADE_LOADING = 0.11
    design_constraints.MACH_TIP_LIMIT = 0.7
    design_constraints.MACH_ADVANCING_TIP_LIMIT = 0.83
    design_assumptions.ELECTRONICS_MASS = 2.64

    # analysis (for each design)
    hex = ConventionalRotorcraft(6, 4, mission_scenario, design_constraints, design_assumptions)

    # initial verification using design_mass = MASS_LIMIT
    payload = hex.calc_and_verify_initial_design(MASS_LIMIT)

def msh_coaxial_test():
    print("Coaxial Mars Science Helicopter")
    # mission setup
    MASS_LIMIT = 18.03
    mission_scenario = FlightMissionScenario(
        hover_time=2*60, forward_flight_speed=30, forward_flight_distance=1000, 
        climb_height=200, climb_rate=10, descent_rate=10)
    design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
    design_assumptions = DesignAssumptions()
    design_assumptions.BLADE_LOADING = 0.11
    design_constraints.MACH_TIP_LIMIT = 0.7
    design_constraints.MACH_ADVANCING_TIP_LIMIT = 0.83
    design_assumptions.ELECTRONICS_MASS = 2.64

    # analysis (for each design)
    coaxial = CoaxialRotorcraft(2, 4, mission_scenario, design_constraints, design_assumptions)

    # initial verification using design_mass = MASS_LIMIT
    payload = coaxial.calc_and_verify_initial_design(MASS_LIMIT)

def advanced_mh_test():
    print("Advanced Mars Helicopter")
    # mission setup
    MASS_LIMIT = 4.6
    mission_scenario = FlightMissionScenario(
        hover_time=2*60, forward_flight_distance=2000, climb_height=200, 
        climb_rate=2, descent_rate=2)
    design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
    design_assumptions = DesignAssumptions()
    design_assumptions.BLADE_LOADING = 0.115
    design_assumptions.ELECTRONICS_MASS = 0.6

    # analysis (for each design)
    coaxial = CoaxialRotorcraft(2, 2, mission_scenario, design_constraints, design_assumptions)

    # initial verification using design_mass = MASS_LIMIT
    payload = coaxial.calc_and_verify_initial_design(MASS_LIMIT)

def mission_setup(max_diameter=4.35) -> tuple[FlightMissionScenario, DesignConstraints, DesignAssumptions]:
    # don't really need hover apart from identification of sampling/landing locations
    # 3 minutes used in consideration of this and additional redundancy around takeoff/landing
    our_mission_scenario = FlightMissionScenario(
            hover_time=3*60, forward_flight_distance=600, climb_height=50, 
            climb_rate=10, descent_rate=10)
    MASS_LIMIT = 50
    design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=max_diameter)
    design_assumptions = DesignAssumptions()
    return our_mission_scenario, design_constraints, design_assumptions

def perform_analyses(write_csv=False):
    mission_scenario, design_constraints, design_assumptions = mission_setup()
    design_constraints.MAX_DIAMETER = 1000
    coaxial = CoaxialRotorcraft("Coaxial - Ingenuity-like", 2, 4, mission_scenario, design_constraints, design_assumptions)
    quad = ConventionalRotorcraft("Quadcopter", 4, 4, mission_scenario, design_constraints, design_assumptions)
    hex = ConventionalRotorcraft("Hexcopter", 6, 4, mission_scenario, design_constraints, design_assumptions)
    octo = ConventionalRotorcraft("Octocopter", 8, 4, mission_scenario, design_constraints, design_assumptions)
    quad_coaxial = CoaxialRotorcraft("Quad coaxial", 8, 4, mission_scenario, design_constraints, design_assumptions)
    tiltrotor = TiltRotorcraft("Hex-tiltrotor", 6, 4, 4, mission_scenario, design_constraints, design_assumptions)
    aircrafts = [coaxial, quad, hex, octo, quad_coaxial, tiltrotor]
    # aircrafts = [hex, tiltrotor]
    
    # design_mass = 20
    # for aircraft in aircrafts:
    #     aircraft_analysis(aircraft, design_mass)

    if write_csv:
        design_masses = [20, 25, 30, 35, 40, 45, 50]
        with open("data.csv", "w", newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            header_str = "aircraft,no_rotors,no_nontilting_rotors,no_blades,max_thrust_requirement,rotor_radius,max_thrust_power,hover_power,f_flight_power,motor_rpm_hover,motor_power,motor_power_spec,motor_torque,max_forward_velocity,total_energy,flight_energy,ground_energy,sampling_energy,sleep_energy,design_mass,contingency_mass,motor_mass,battery_mass,solar_panel_mass,rotor_mass,structure_mass,ground_mobility_mass,flight_elec_mass,total_empty_mass,payload"
            header = header_str.split(',')
            csv_writer.writerow(header)
            for aircraft in aircrafts:
                for design_mass in design_masses:
                    aircraft.calc_and_verify_initial_design(design_mass)
                    csv_writer.writerow(aircraft.get_csv_summary())

def aircraft_analysis(aircraft, design_mass):
    # initial verification using design_mass = MASS_LIMIT
    payload = aircraft.calc_and_verify_initial_design(design_mass)

    # payload efficiency tradeoff
    # - vary design_mass and calculate payload/design_mass
    design_masses, payload_efficiences = aircraft.payload_efficiency_analysis()

    # payload vs hover/range tradeoff
    # - fix design_mass (maybe at MASS_LIMIT, maybe at maximum payload efficiency) 
    # - decrease the proportion of payload for sampling and move into battery to provide more energy/power
    # - back calculate HOVER_TIME or FORWARD_FLIGHT_RANGE from total_power
    MIN_PAYLOAD = 2
    valid_payloads, hover_times, ranges = aircraft.trade_payload_for_battery(design_mass, MIN_PAYLOAD)

    mass_max_payload_efficiency = design_masses[np.argmax(payload_efficiences)]
    valid_payloads, hover_times, ranges = aircraft.trade_payload_for_battery(mass_max_payload_efficiency, MIN_PAYLOAD)

    plt.figure()
    plt.plot(design_masses, np.array(payload_efficiences)*100)
    plt.title(f"Payload efficiency vs payload: {aircraft.name}")
    plt.xlabel("Payload (kg)")
    plt.ylabel("Payload efficiency (%)")
    plt.grid(True)

    plt.figure()
    for payload, rng, hover_time in zip(valid_payloads, ranges, hover_times):
        plt.plot([0, hover_time/60], [rng/1000, 0], label=f"Payload {payload:.2f}")
    plt.xlabel('Hover Time')
    plt.ylabel('Range')
    plt.title(f'Hover Time vs Range for Different Payloads: {aircraft.name}')
    plt.legend()
    plt.grid(True) 
    plt.show()
    

if __name__ == "__main__":
    # advanced_mh_test()

    # msh_test()

    # msh_coaxial_test()

    perform_analyses(True)

    