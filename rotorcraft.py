import numpy as np

import mars_constants
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions

class Rotorcraft:

    def __init__(self, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions):
        self._mission_scenario = mission_scenario
        self._design_constraints = design_constraints
        self._design_assumptions = design_assumptions

        self._no_rotors = no_rotors
        self._no_blades = no_blades

    ##########################################################
    ##### ANALYSES
    ##########################################################

    def calc_and_verify_initial_design(self, design_mass=None):
        print("\n\n-----\nNew analysis: calculation and verification of initial design\n-----\n")
        self._design_mass = self.dc.MASS_LIMIT if not design_mass else design_mass
        print(f"Design mass of rotorcraft is: {self._design_mass:.2f}kg")

        self._max_thrust = self.calc_max_thrust(self._design_mass)
        print(f"Maximum thrust required is: {self._max_thrust:.2f}N")

        self._rotor_radius = self.calc_rotor_radius(self.max_thrust_per_rotor)
        print(f"Rotor radius required to produce maximum thrust is {self._rotor_radius:.2f}m")
        # TODO figure out the geometry better
        if self._rotor_radius * 2 > self.dc.MAX_DIAMETER:
            raise ValueError(f"Cannot create required thrust to fit in aeroshell. Rotor radius: {self._rotor_radius:.2f}, aeroshell diameter: {self.dc.MAX_DIAMETER}")
        
        self._max_thrust_power = self.calc_max_thrust_power(self.hover_thrust_per_rotor, self.blade_area)
        print(f"Power from motors at maximum thrust is: {self._max_thrust_power:.2f}W")

        self._torque = self.calc_torque(self._max_thrust_power, self._rotor_radius)
        print(f"Maximum torque required at maximum thrust is: {self._torque:.2f}N.m")

        self._energy_per_sol = self.calc_energy_per_sol()
        print(f"Energy required for rotorcraft per sol is: {self._energy_per_sol/1e6:.3f}MJ")
        self._empty_mass = self.calc_empty_mass(self._torque, self._energy_per_sol)
        print(f"Empty mass of the rotorcraft is: {self._empty_mass:.2f}kg")
        
        self._payload = self.calc_payload(self.total_available_mass, self._empty_mass)
        print(f"Achievable payload is: {self._payload:.2f}kg")
        if self._payload < 0:
            raise ValueError(f"Payload is less than zero: {self._payload:.2f}! Cannot lift anything.")
        return self._payload

    def payload_efficiency_analysis(self, design_masses=None):
        if design_masses is None:
            design_masses = [10, 15, 20, 25, 30, 35, 40, 45, 50]
        
        payloads = []
        for design_mass in design_masses:
            try:
                payloads.append(self.calc_and_verify_initial_design(design_mass))
            except ValueError:
                payloads.append(0)
        
        payload_efficiencies = [payload/design_mass for payload, design_mass in zip(payloads, design_masses)]
        return design_masses, payload_efficiencies

    def trade_payload_for_battery(self, design_mass, min_payload):
        max_payload = self.calc_and_verify_initial_design(design_mass)
        valid_payloads = list(range(int(max_payload), int(min_payload)-1,-1))
        valid_payloads.insert(0, max_payload)
        extra_masses = [max_payload - reduced for reduced in valid_payloads]
        extra_energies = [self.calc_energy_from_battery_mass(extra_mass) for extra_mass in extra_masses]
        extra_hover_times = [self.hover_time_from_energy(extra_energy) for extra_energy in extra_energies]
        extra_f_flight_distances = [self.f_flight_distance_from_energy(extra_energy) for extra_energy in extra_energies]

        return valid_payloads, extra_hover_times, extra_f_flight_distances
    
    ##########################################################
    ##### CALCULATIONS OF PARAMETERS
    ##########################################################

    def calc_max_thrust(self, design_mass):
        """Calculate thrust based on total design mass, not mass available for componentry (e.g. 50kg, with 40kg left for components)"""
        return self.dc.HOVER_THRUST_CONDITION * design_mass * mars_constants.GRAVITY
    
    def calc_blade_area(self, thrust_per_rotor):
        return thrust_per_rotor / (mars_constants.DENSITY * self.da.BLADE_LOADING * self.dc.TIP_SPEED_LIMIT**2)
    
    def calc_tip_speed(self, thrust_per_rotor, blade_area):
        return np.sqrt(thrust_per_rotor / (mars_constants.DENSITY * self.da.BLADE_LOADING * blade_area))

    def calc_rotor_radius(self, thrust_per_rotor):
        blade_area = self.calc_blade_area(thrust_per_rotor)
        rotor_radius = 2.8978 * blade_area ** 0.5
        return rotor_radius

    def calc_thrust_power(self, thrust_per_rotor, blade_area, induced_power_factor, tip_speed):
        induced_power = induced_power_factor * thrust_per_rotor * \
            np.sqrt(thrust_per_rotor / (2 * mars_constants.DENSITY * self.disk_area))
        profile_power = mars_constants.DENSITY * blade_area * tip_speed**3 * self.da.DRAG_COEF_MEAN / 8
        print(f"Induced power is {induced_power:.2f}W, profile power is {profile_power:.2f}W")
        thrust_power_per_rotor =  induced_power + profile_power
        return thrust_power_per_rotor * self._no_rotors
    
    def calc_max_thrust_power(self, max_thrust_per_rotor, blade_area):
        """Assuming ok to use hover induced power factor since thrust has increased for induced power.
        Using max tip speed limit governed by MACH limit of 0.8"""
        print("Max thrust power calculations:")
        return self.calc_thrust_power(max_thrust_per_rotor, blade_area, self.induced_power_factor_hover, self.dc.TIP_SPEED_LIMIT)        
    
    def calc_torque(self, thrust_power, rotor_radius):
        """From Ronan's aerodynamics notes"""
        rotational_speed = self.dc.TIP_SPEED_LIMIT / rotor_radius
        return thrust_power / self.da.PROPULSIVE_EFFICIENCY / rotational_speed        
    
    def calc_energy_per_sol(self):
        print("----\nEnergy calculations\n----")
        print(f"Forward velocity used for mission: {self.forward_velocity:.2f}m/s")
        flight_energy = self._mission_scenario.get_single_flight_energy(self.hover_power, self.forward_velocity, self.f_flight_power, self.da.AVIONICS_POWER)
        print(f"Single flight: {flight_energy/1e6:.3f}MJ")
        sampling_mechanism_energy = self.da.SAMPLING_MECHANISM_POWER * self.da.SAMPLING_TIME
        print(f"Sampling mechanism: {sampling_mechanism_energy/1e6:.3f}MJ")

        # TODO -> avionics only used during mission time?
        # avionics_energy = self.da.AVIONICS_POWER * flight_time
        # print(f"Avionics: {avionics_energy/1e6:.3f}MJ")

        sleep_energy = 0.518 * self._design_mass**(1/3) * mars_constants.SOL_SECONDS
        print(f"Sleeping: {sleep_energy/1e6:.3f}MJ")
        return flight_energy + sampling_mechanism_energy + sleep_energy 
    
    def calc_empty_mass(self, torque, energy):
        print("----\nMass calculations\n----")
        motor_mass = 0.076 * torque**0.86 # kg - NASA MSH (based on MH)
        print(f"Motor: {motor_mass:.2f}kg")
        total_power = energy / mars_constants.SOL_SECONDS
        solar_panel_area = total_power / self.da.SOLAR_PANEL_POWER_DENSITY # m^2
        solar_panel_mass = solar_panel_area * self.da.SOLAR_PANEL_MASS_DENSITY # kg
        print(f"Solar panel: {solar_panel_mass:.2f}kg")
        energy_Wh = energy / (60*60)
        energy_required_Wh = energy_Wh / self.da.USABLE_BATTERY_PERC # TODO: did NASA MSH paper get this wrong and not actually consider USABLE_BATTERY_PERC?
        battery_mass = energy_required_Wh / self.da.BATTERY_DENSITY # NASA MSH paper
        print(f"Battery: {battery_mass:.2f}kg")
        # TODO decide between these methods
        # rotor_mass = 1.1 * blade_area # NASA MSH paper
        rotor_mass = (0.168/0.72) * self._rotor_radius * self._no_blades # ROAMX blade correlation between mass and radius
        print(f"Rotors: {rotor_mass:.2f}kg")
        # TODO could consider things like hub, shaft, support arms, fuselage -> however I think this is overspecifying and hence scaling with design mass
        structure_mass = 1/3 * self._design_mass # based on MSH paper designs
        print(f"Structure: {structure_mass:.2f}kg")
        flight_electronics_mass = self.da.ELECTRONICS_MASS
        print(f"Flight electronics: {flight_electronics_mass:.2f}kg")
        return motor_mass + solar_panel_mass + battery_mass + rotor_mass + structure_mass + flight_electronics_mass
    
    def calc_energy_from_battery_mass(self, battery_mass):
        """battery_mass in kg, energy returned in joules"""
        return battery_mass * self.da.BATTERY_DENSITY * 60 * 60
    
    def calc_payload(self, available_mass, empty_mass):
        return available_mass - empty_mass
    
    def hover_time_from_energy(self, energy):
        return energy / self.hover_power
    
    def f_flight_distance_from_energy(self, energy):
        return energy / self.forward_velocity
    
    ##########################################################
    ##### GETTERS AND SETTERS
    ##########################################################

    @property
    def dc(self):
        return self._design_constraints
    
    @property 
    def da(self):
        return self._design_assumptions

    @property
    def total_available_mass(self):
        self._total_available_mass = self._design_mass * (1 - self.dc.CONTINGENCY_WEIGHT_FACTOR)
        return self._total_available_mass
    
    @property
    def max_thrust(self):
        return self._max_thrust

    @property
    def max_thrust_per_rotor(self):
        self._max_thrust_per_rotor = self._max_thrust / self._no_rotors
        return self._max_thrust_per_rotor
    
    @property
    def hover_thrust(self):
        return self._max_thrust / self.dc.HOVER_THRUST_CONDITION
    
    @property
    def hover_thrust_per_rotor(self):
        return self.max_thrust_per_rotor / self.dc.HOVER_THRUST_CONDITION
    
    @property
    def blade_area(self):
        return 0.119087 * self._rotor_radius ** 2 * self._no_blades
    
    @property
    def disk_area(self):
        return self._rotor_radius**2 * np.pi
    
    @property
    def thrust_power_per_rotor(self):
        self._thrust_power_per_rotor = self._max_thrust_power / self._no_rotors
        return self._thrust_power_per_rotor
    
    @property
    def hover_power(self):
        print("Hover power calculations:")
        hover_tip_speed = self.calc_tip_speed(self.hover_thrust_per_rotor, self.blade_area)
        return self.calc_thrust_power(self.hover_thrust_per_rotor, self.blade_area, self.induced_power_factor_hover, hover_tip_speed)
    
    @property
    def f_flight_power(self):
        """Using hover thrust power because induced power factor increases induced power.
        Using increased advancing speed tip limit to increase profile power -> probably conservative because of retreating tip speed"""
        print("Forward flight power calculations")
        return self.calc_thrust_power(self.hover_thrust_per_rotor, self.blade_area, self.induced_power_factor_forward, self.dc.ADVANCING_TIP_SPEED_LIMIT)
    
    @property
    def forward_velocity(self):    
        return self.dc.ADVANCING_TIP_SPEED_LIMIT - self.dc.TIP_SPEED_LIMIT

    @property
    def induced_power_factor_hover(self):
        raise TypeError("In abstract base class. Instantiate type of aircraft.")
    
    @property
    def induced_power_factor_forward(self):
        raise TypeError("In abstract base class. Instantiate type of aircraft.")

    @property
    def total_power(self):
        return self._energy_per_sol / mars_constants.SOL_SECONDS


class ConventionalRotorcraft(Rotorcraft):

    def __init__(self, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions):
        
        super().__init__(no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions)

    # from NASA MSH paper and other papers listed in Notion
    @property
    def induced_power_factor_hover(self):
        return 1.2
    
    @property
    def induced_power_factor_forward(self):
        return 2.0

class CoaxialRotorcraft(Rotorcraft):

    def __init__(self, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions):
        
        super().__init__(no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions)

    # from NASA MSH paper and other papers listed in Notion
    @property
    def induced_power_factor_hover(self):
        return 1.1
    
    @property
    def induced_power_factor_forward(self):
        return 1.6

class TiltRotorcraft(Rotorcraft):

    def __init__(self, no_rotors, no_nontilt_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions):
        
        super().__init__(no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions)

        self._no_nontilt_rotors = no_nontilt_rotors

    def calc_max_thrust(self, design_mass):
        """Calculate thrust based on total design mass, not mass available for componentry (e.g. 50kg, with 40kg left for components)"""
        tiltrotor_multiplier = self._no_rotors / self.no_nontilt_rotors
        return self.dc.HOVER_THRUST_CONDITION * design_mass * mars_constants.GRAVITY * tiltrotor_multiplier
    
    # from NASA MSH paper and other papers listed in Notion
    @property
    def induced_power_factor_hover(self):
        return 1.2
    
    # TODO update this
    @property
    def induced_power_factor_forward(self):
        return 2.0
    
    # TODO -> update this with forward velocity of tiltrotor
    @property
    def forward_velocity(self):    
        return 1
