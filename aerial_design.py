from rotorcraft import ConventionalRotorcraft, CoaxialRotorcraft, TiltRotorcraft
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions
import matplotlib.pyplot as plt
import numpy as np

def msh_test():
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

if __name__ == "__main__":
    # advanced_mh_test()

    # msh_test()

    msh_coaxial_test()

    # # mission setup
    # MASS_LIMIT = 20
    # mission_scenario = FlightMissionScenario(
    #     hover_time=5*60, forward_flight_distance=600, climb_height=50, 
    #     climb_rate=2, descent_rate=2)
    # design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
    # design_assumptions = DesignAssumptions()

    # # analysis (for each design)
    # quad = ConventionalRotorcraft(4, 2, mission_scenario, design_constraints, design_assumptions)

    # # initial verification using design_mass = MASS_LIMIT
    # payload = quad.calc_and_verify_initial_design(MASS_LIMIT)

    # # payload efficiency tradeoff
    # # - vary design_mass and calculate payload/design_mass
    # design_masses, payload_efficiences = quad.payload_efficiency_analysis()

    # # payload vs hover/range tradeoff
    # # - fix design_mass (maybe at MASS_LIMIT, maybe at maximum payload efficiency) 
    # # - decrease the proportion of payload for sampling and move into battery to provide more energy/power
    # # - back calculate HOVER_TIME or FORWARD_FLIGHT_RANGE from total_power
    # MIN_PAYLOAD = 2
    # valid_payloads, extra_hover_times, extra_f_flight_distances = quad.trade_payload_for_battery(MASS_LIMIT, MIN_PAYLOAD)

    # mass_max_payload_efficiency = design_masses[np.argmax(payload_efficiences)]
    # valid_payloads, extra_hover_times, extra_f_flight_distances = quad.trade_payload_for_battery(mass_max_payload_efficiency, MIN_PAYLOAD)

    # plt.figure()
    # plt.plot(design_masses, payload_efficiences)
    # plt.show()

    