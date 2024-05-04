import numpy as np
import logging

import mars_constants
from mission_design import FlightMissionScenario, DesignConstraints, DesignAssumptions

class Rotorcraft:

    def __init__(self, name, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions,
                 log_level=logging.WARNING):
        self.name = name 

        self._mission_scenario = mission_scenario
        self._design_constraints = design_constraints
        self._design_assumptions = design_assumptions

        self._no_rotors = no_rotors
        self._no_blades = no_blades

        # to calculate total diameter
        self._no_nonoverlapping_rotors = self._no_rotors

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(log_level)
        formatter = logging.Formatter('%(message)s')
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

    ##########################################################
    ##### ANALYSES
    ##########################################################

    def calc_and_verify_initial_design(self, design_mass=None):
        self.logger.info(f"\n{self.name}\n-----\nNew analysis: calculation and verification of initial design\n-----\n")
        self._design_mass = self.dc.MASS_LIMIT if not design_mass else design_mass
        self.logger.info(f"Design mass of rotorcraft is: {self._design_mass:.2f}kg")

        self._max_thrust = self.calc_max_thrust(self._design_mass)
        self.logger.info(f"Maximum thrust required is: {self._max_thrust:.2f}N")

        self._rotor_radius = self.calc_rotor_radius(self.max_thrust_per_rotor)
        self.logger.info(f"Rotor radius required to produce maximum thrust is {self._rotor_radius:.2f}m")
        
        if self.total_diameter > self.dc.MAX_DIAMETER:
            raise ValueError(f"Cannot create required thrust to fit in aeroshell. Rotor radius: {self._rotor_radius:.2f} results in diameter {self.total_diameter:.2f}m aeroshell diameter: {self.dc.MAX_DIAMETER}")
                
        self._max_thrust_power = self.calc_max_thrust_power(self.hover_thrust_per_rotor, self.rotor_area)
        self.logger.info(f"Power required for maximum thrust is: {self._max_thrust_power:.2f}W")

        self._torque = self.calc_torque(self._max_thrust_power, self._rotor_radius)
        self.logger.info(f"Maximum torque required at maximum thrust is: {self._torque:.2f}N.m")

        if self.max_forward_velocity < self._mission_scenario.FORWARD_FLIGHT_SPEED:
            raise ValueError(f"Cannot travel at desired speed for mission of {self._mission_scenario.FORWARD_FLIGHT_SPEED:.2f}m/s. Can only reach: {self.max_forward_velocity:.2f}m/s")
        self.logger.info(f"Maximum forward velocity of aircraft is: {self.max_forward_velocity:.2f}m/s")

        self._energy_per_sol = self.calc_energy_per_sol()
        self.logger.info(f"Energy required for rotorcraft per sol is: {self._energy_per_sol/1e6:.3f}MJ")
        self._empty_mass = self.calc_empty_mass(self._torque, self._energy_per_sol)
        self.logger.info(f"Empty mass of the rotorcraft is: {self._empty_mass:.2f}kg")
        
        self._payload = self.calc_payload(self.total_available_mass, self._empty_mass)
        self.logger.info(f"Achievable payload is: {self._payload:.2f}kg")
        if self._payload < 0:
            raise ValueError(f"Payload is less than zero: {self._payload:.2f}! Cannot lift anything.")
        return self._payload

    def payload_efficiency_analysis(self, design_masses=None):
        if design_masses is None:
            design_masses = [10, 15, 20, 25, 30, 35, 40, 45, 50]
        
        payloads = []
        valid_design_masses = []
        for design_mass in design_masses:
            try:
                payloads.append(self.calc_and_verify_initial_design(design_mass))
                valid_design_masses.append(design_mass)
            except ValueError:
                continue
        
        payload_efficiencies = [payload/design_mass for payload, design_mass in zip(payloads, valid_design_masses)]
        return valid_design_masses, payload_efficiencies
    
    def number_of_blades_analysis(self, design_masses=None):
        if design_masses is None:
            design_masses = [10, 15, 20, 25, 30, 35, 40, 45, 50]
        
        payloads = []
        rotor_radiuses = []
        valid_design_masses = []
        for design_mass in design_masses:
            try:
                payloads.append(self.calc_and_verify_initial_design(design_mass))
                rotor_radiuses.append(self._rotor_radius)
                valid_design_masses.append(design_mass)
            except ValueError:
                continue
        
        payload_efficiencies = [payload/design_mass for payload, design_mass in zip(payloads, design_masses)]
        return valid_design_masses, payload_efficiencies, rotor_radiuses

    def trade_payload_for_battery(self, design_mass, min_payload):
        max_payload = self.calc_and_verify_initial_design(design_mass)
        valid_payloads = list(range(int(min_payload), int(max_payload)+1))
        valid_payloads.append(max_payload)
        battery_masses = [max_payload - reduced + self._battery_mass for reduced in valid_payloads]
        energies = [self.calc_energy_from_battery_mass(extra_mass) for extra_mass in battery_masses]
        hover_times = [self.hover_time_from_energy(extra_energy) for extra_energy in energies]
        f_flight_distances = [self.f_flight_distance_from_energy(extra_energy) for extra_energy in energies]

        return valid_payloads, hover_times, f_flight_distances
    
    def get_csv_summary(self):
        """aircraft,no_rotors,no_nontilting_rotors,no_blades,max_thrust_requirement,rotor_radius,max_thrust_power,hover_power,f_flight_power,motor_rpm_hover,motor_power,motor_power_spec,motor_torque,max_forward_velocity,total_energy,flight_energy,ground_energy,sampling_energy,sleep_energy,design_mass,contingency_mass,motor_mass,battery_mass,solar_panel_mass,rotor_mass,structure_mass,ground_mobility_mass,flight_elec_mass,total_empty_mass,payload"""
        summary = [self.name, self._no_rotors, self._no_rotors, self._no_blades, self._max_thrust, self._rotor_radius, 
                   self._max_thrust_power, self.hover_power, self.f_flight_power, self.motor_rpm_hover, self._max_thrust_power, 
                   self._motor_power_spec, self._torque, self.max_forward_velocity, 
                   self._energy_per_sol, self._flight_energy, self._ground_mobility_energy, self._sampling_mechanism_energy, self._sleep_energy, 
                   self._design_mass, self._total_available_mass, self._motor_mass, self._battery_mass, self._solar_panel_mass, 
                   self._rotor_mass, self._ground_mobility_mass, self._flight_electronics_mass, self._empty_mass, self._payload]
        return summary

    ##########################################################
    ##### CALCULATIONS OF PARAMETERS
    ##########################################################

    def calc_max_thrust(self, design_mass):
        """Calculate thrust based on total design mass, not mass available for componentry (e.g. 50kg, with 40kg left for components)"""
        return self.dc.HOVER_THRUST_CONDITION * design_mass * mars_constants.GRAVITY
    
    def calc_rotor_area(self, thrust_per_rotor):
        """Rotor area of one rotor based on equation solidity = thrust / (density * blade_area * tip_speed^2)"""
        return thrust_per_rotor / (mars_constants.DENSITY * self.da.BLADE_LOADING * self.dc.TIP_SPEED_LIMIT**2)
    
    def calc_tip_speed(self, thrust_per_rotor, blade_area):
        return np.sqrt(thrust_per_rotor / (mars_constants.DENSITY * self.da.BLADE_LOADING * blade_area))

    def calc_rotor_radius(self, thrust_per_rotor):
        rotor_area = self.calc_rotor_area(thrust_per_rotor)
        self.logger.info(f"Rotor area requirement is: {rotor_area}")
        blade_area = rotor_area / self._no_blades
        self.logger.info(f"Blade area requirement is: {blade_area}")
        rotor_radius = 2.8978 * blade_area ** 0.5
        self.logger.info(f"Therefore rotor radius is: {rotor_radius}")
        return rotor_radius

    def calc_thrust_power(self, thrust_per_rotor, blade_area, induced_power_factor, tip_speed, no_rotors=None):
        """Considers propulsive efficiency (i.e. motor efficiency and power lost to other things such as servos)"""
        if no_rotors is None:
            no_rotors = self._no_rotors
        induced_power = induced_power_factor * thrust_per_rotor * \
            np.sqrt(thrust_per_rotor / (2 * mars_constants.DENSITY * self.disk_area))
        profile_power = mars_constants.DENSITY * blade_area * tip_speed**3 * self.da.DRAG_COEF_MEAN / 8
        self.logger.info(f"Induced power per rotor is {induced_power:.2f}W, profile power per rotor is {profile_power:.2f}W")
        thrust_power_per_rotor =  induced_power + profile_power
        return thrust_power_per_rotor * no_rotors / self.propulsive_efficiency
    
    def calc_max_thrust_power(self, max_thrust_per_rotor, blade_area):
        """Assuming ok to use hover induced power factor since thrust has increased for induced power.
        Using max tip speed limit governed by MACH limit of 0.8"""
        self.logger.info("Max thrust power calculations:")
        return self.calc_thrust_power(max_thrust_per_rotor, blade_area, self.induced_power_factor_hover, self.dc.TIP_SPEED_LIMIT)        
    
    def calc_torque(self, thrust_power, rotor_radius):
        """From Ronan's aerodynamics notes""" 
        rotational_speed = self.dc.TIP_SPEED_LIMIT / rotor_radius
        self.logger.info(f"Motor rotational speed at hover: {self.motor_rpm_hover:.2f}RPM")
        self.logger.info(f"Power required from the motors at max thrust is: {thrust_power:.2f}W")
        self._motor_power_spec = self.hover_power * 1.5
        self.logger.info(f"Motor is specced to: {self._motor_power_spec:.2f}W (150% hover power)")
        return thrust_power / rotational_speed
    
    def calc_energy_per_sol(self):
        self.logger.info("----\nEnergy calculations\n----")
        self.logger.info(f"Forward velocity used for mission: {self._mission_scenario.FORWARD_FLIGHT_SPEED:.2f}m/s")
        self._flight_energy = self._mission_scenario.get_single_flight_energy(self.hover_power, self.f_flight_power, self.da.AVIONICS_POWER)
        self.logger.info(f"Single flight: {self._flight_energy/1e6:.3f}MJ")
        # TODO - should this be different for the tilt rotor?
        self._ground_mobility_energy = self.da.GROUND_MOBILITY_POWER * self.da.GROUND_MOBILITY_TIME
        self.logger.info(f"Ground mobility: {self._ground_mobility_energy/1e6:.3f}MJ")
        self._sampling_mechanism_energy = self.da.SAMPLING_MECHANISM_POWER * self.da.SAMPLING_TIME
        self.logger.info(f"Sampling mechanism: {self._sampling_mechanism_energy/1e6:.3f}MJ")
        self._sleep_energy = 0.518 * self._design_mass**(1/3) * mars_constants.SOL_SECONDS
        self.logger.info(f"Sleeping: {self._sleep_energy/1e6:.3f}MJ")

        mission_energy = self._flight_energy + self._ground_mobility_energy + self._sampling_mechanism_energy + self._sleep_energy 
        return self.da.BATTERY_CONTINGENCY * mission_energy
    
    def calc_empty_mass(self, torque, energy):
        self.logger.info("----\nMass calculations\n----")
        self._motor_mass = self.da.MOTOR_MASS_FACTOR * 0.076 * torque**0.86 # kg - NASA MSH (based on MH)
        self.logger.info(f"Motor: {self._motor_mass:.2f}kg")
        
        energy_required = energy / self.da.USABLE_BATTERY_PERC
        energy_required_Wh = energy_required / (60*60)
        self._battery_mass = energy_required_Wh / self.da.BATTERY_DENSITY # NASA MSH paper
        self.logger.info(f"Battery: {self._battery_mass:.2f}kg")
        solar_panel_area = energy_required / self.da.SOLAR_PANEL_ENERGY_PER_SOL # m^2
        self._solar_panel_mass = solar_panel_area * self.da.SOLAR_PANEL_MASS_DENSITY # kg
        self.logger.info(f"Solar panel: {self._solar_panel_mass:.2f}kg")
        # TODO make this more representative using density?
        self._rotor_mass = (0.168/0.72) * self._rotor_radius * self._no_blades * self._no_rotors # ROAMX blade correlation between mass and radius
        self.logger.info(f"Rotors: {self._rotor_mass:.2f}kg")
        self._structure_mass = 1/3 * self._design_mass - (1 / self.da.ROTOR_MASS_FACTOR) * self._rotor_mass # based on MSH paper designs
        self.logger.info(f"Structure: {self._structure_mass:.2f}kg")
        self._ground_mobility_mass = self.da.GROUND_MOBILITY_MASS_PROPORTION * self._design_mass
        self.logger.info(f"Wheel + motor: {self._ground_mobility_mass:.2f}kg")
        self._flight_electronics_mass = self.da.ELECTRONICS_MASS
        self.logger.info(f"Flight electronics: {self._flight_electronics_mass:.2f}kg")
        return self._motor_mass + self._solar_panel_mass + self._battery_mass + self._rotor_mass + self._structure_mass + self._ground_mobility_mass + self._flight_electronics_mass
    
    def calc_energy_from_battery_mass(self, battery_mass):
        """battery_mass in kg, energy returned in joules"""
        return battery_mass * self.da.BATTERY_DENSITY * 60 * 60 * self.da.USABLE_BATTERY_PERC / self.da.BATTERY_CONTINGENCY
    
    def calc_payload(self, available_mass, empty_mass):
        return available_mass - empty_mass
    
    def hover_time_from_energy(self, energy):
        """Energy in J. self.hover_power in W. Hover time in seconds"""
        return energy / self.hover_power
    
    def f_flight_distance_from_energy(self, energy):
        """Energy in J. self.f_flight_power in W. self._mission_scenario.FORWARD_FLIGHT_SPEED in m/s. Range in metres"""
        return energy / self.f_flight_power * self._mission_scenario.FORWARD_FLIGHT_SPEED
    
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
    def total_diameter(self):
        if self._no_nonoverlapping_rotors == 1:
            return 2 * self._rotor_radius
        elif self._no_nonoverlapping_rotors == 4:
            return (1 + np.sqrt(2)) * 2 * self._rotor_radius
        elif self._no_nonoverlapping_rotors == 6:
            return 3 * self._rotor_radius * 2
        elif self._no_nonoverlapping_rotors == 8:
            return 17/5 * self._rotor_radius * 2

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
    def f_flight_thrust_per_rotor(self):
        return self.hover_thrust_per_rotor / np.cos(self.da.FORWARD_FLIGHT_TILT_ANGLE * np.pi / 180)
    
    @property
    def rotor_area(self):
        """Area for a single rotor (i.e. can be multiple blades attached to the same motor)"""
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
        self.logger.info("Hover power calculations:")
        return self.calc_thrust_power(self.hover_thrust_per_rotor, self.rotor_area, self.induced_power_factor_hover, self.hover_tip_speed)
    
    @property
    def f_flight_power(self):
        """Using hover thrust power because induced power factor increases induced power.
        Using increased advancing speed tip limit to increase profile power -> probably conservative because of retreating tip speed
        Using advancing tip speed limit instead of forward flight tip speed to be conservative since profile power increased by tip speed which
        is highest in the advancing portion of the blade in forward flight."""
        self.logger.info("Forward flight power calculations")
        return self.calc_thrust_power(self.f_flight_thrust_per_rotor, self.rotor_area, self.induced_power_factor_forward, self.dc.ADVANCING_TIP_SPEED_LIMIT)
    
    @property
    def hover_tip_speed(self):
        return self.calc_tip_speed(self.hover_thrust_per_rotor, self.rotor_area)

    @property
    def f_flight_tip_speed(self):
        """Forward tip speed assumes increased thrust in forward flight due to tilt required to produce forward velocity"""
        return self.calc_tip_speed(self.f_flight_thrust_per_rotor, self.rotor_area)
    
    @property
    def motor_rpm_hover(self):
        return self.dc.TIP_SPEED_LIMIT / (self._rotor_radius * 2 * np.pi) * 60

    @property
    def max_forward_velocity(self):    
        "Forward velocity limited by the advancing blade tip mach number"
        return self.dc.ADVANCING_TIP_SPEED_LIMIT - self.f_flight_tip_speed

    @property
    def induced_power_factor_hover(self):
        raise TypeError("In abstract base class. Instantiate type of aircraft.")
    
    @property
    def induced_power_factor_forward(self):
        raise TypeError("In abstract base class. Instantiate type of aircraft.")
    
    @property 
    def rotor_servo_power_proportion(self):
        raise TypeError("In abstract base class. Instantiate type of aircraft.")
    
    @property
    def propulsive_efficiency(self):
        return self._design_assumptions.MOTOR_EFFICIENCY * (1 - self.rotor_servo_power_proportion)


class ConventionalRotorcraft(Rotorcraft):

    def __init__(self, name, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions,
                 log_level=logging.WARNING):
        
        super().__init__(name, no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions, log_level)

    # from NASA MSH paper and other papers listed in Notion
    @property
    def induced_power_factor_hover(self):
        return 1.2
    
    @property
    def induced_power_factor_forward(self):
        return 2.0
    
    @property
    def rotor_servo_power_proportion(self):
        return 0
    

class CoaxialRotorcraft(Rotorcraft):

    def __init__(self, name, no_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions,
                 log_level=logging.WARNING):
        
        super().__init__(name, no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions, log_level)

        self._no_nonoverlapping_rotors = self._no_rotors / 2

    # from NASA MSH paper and other papers listed in Notion
    @property
    def induced_power_factor_hover(self):
        return 1.1
    
    @property
    def induced_power_factor_forward(self):
        return 1.6
    
    @property
    def rotor_servo_power_proportion(self):
        return 0.15
    

class TiltRotorcraft(ConventionalRotorcraft):

    def __init__(self, name, no_rotors, no_nontilt_rotors, no_blades, mission_scenario: FlightMissionScenario, 
                 design_constraints: DesignConstraints, design_assumptions: DesignAssumptions,
                 log_level=logging.WARNING):
        
        super().__init__(name, no_rotors, no_blades, mission_scenario, design_constraints, design_assumptions, log_level)

        self._no_nontilt_rotors = no_nontilt_rotors

    def calc_max_thrust(self, design_mass):
        """Calculate thrust based on total design mass, not mass available for componentry (e.g. 50kg, with 40kg left for components)"""
        return self.dc.HOVER_THRUST_CONDITION * design_mass * mars_constants.GRAVITY * self.tiltrotor_multiplier
    
    @property
    def tiltrotor_multiplier(self):
        return self._no_rotors / self._no_nontilt_rotors
    
    @property
    def f_flight_thrust_per_rotor(self):
        return self.hover_thrust_per_rotor * self.tiltrotor_multiplier
    
    @property
    def f_flight_power(self):
        """Using hover thrust power because induced power factor increases induced power.
        Using increased advancing speed tip limit to increase profile power -> probably conservative because of retreating tip speed
        Using advancing tip speed limit instead of forward flight tip speed to be conservative since profile power increased by tip speed which
        is highest in the advancing portion of the blade in forward flight."""
        self.logger.info("Forward flight power calculations: TILTROTOR")
        nontilted_power = self.calc_thrust_power(
            self.f_flight_thrust_per_rotor, self.rotor_area, self.induced_power_factor_forward, 
            self.dc.ADVANCING_TIP_SPEED_LIMIT, self._no_nontilt_rotors
        )
        tilted_power = self.calc_thrust_power(
            self.hover_thrust_per_rotor, self.rotor_area, self.induced_power_factor_hover,
            self.hover_tip_speed, self._no_rotors - self._no_nontilt_rotors
        )
        return nontilted_power + tilted_power
