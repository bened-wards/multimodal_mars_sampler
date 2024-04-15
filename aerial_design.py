import numpy as np

import mars_constants

class FlightMissionScenario:

    def __init__(self, hover_time=5*60, forward_flight_distance=600, climb_height=50, climb_rate=2, descent_rate=2):
        """Parameters for planned mission scenario (flight per sol)

        Args:
            hover_time (float): hover time in seconds
            forward_flight_distance (float): forward flight distance in metres
            climb_height (float): height to climb to from initial takeoff
            climb_rate (float): climbing rate in m/s
            descent_rate(float): absolute value of descent rate in m/s
        """
        self.HOVER_TIME = hover_time # seconds
        self.FORWARD_FLIGHT_DISTANCE = forward_flight_distance
        self.CLIMB_HEIGHT = climb_height
        self.CLIMB_RATE = climb_rate
        self.DESCENT_RATE = descent_rate

    def get_single_flight_energy(self, forward_velocity, hover_power, climb_power_factor=1.5, forward_power_factor=1.2, descent_power_factor=0.9):
        """Calculate the energy required for a single flight according to mission scenario

        Args:
            hover_power (float): hover power of the rotorcraft in Watts
            forward_velocity (float): velocity of the rotorcraft in m/s
        """
        climb_time = self.CLIMB_HEIGHT / self.CLIMB_RATE
        climb_power = hover_power * climb_power_factor
        climb_energy = climb_power * climb_time # Joules

        f_flight_time = self.FORWARD_FLIGHT_DISTANCE / forward_velocity
        f_flight_power = forward_power_factor * hover_power # TODO: Not sure if this is actually the case just put it in for now
        f_flight_energy = f_flight_power * f_flight_time

        hover_energy = hover_power * self.HOVER_TIME

        descent_time = self.CLIMB_HEIGHT / self.DESCENT_RATE
        descent_power = descent_power_factor * hover_power # TODO: Do some research into this, not really sure how to do it
        descent_energy = descent_power * descent_time

        return climb_energy + hover_energy + f_flight_energy + descent_energy
    
class DesignConstraints:
    CONTINGENCY_WEIGHT_FACTOR = 0.2 # 20%

    HOVER_THRUST_CONDITION = 1.5 # 150%

    MACH_HOVER_TIP_LIMIT = 0.8 
    HOVER_TIP_SPEED_LIMIT = mars_constants.SPEED_OF_SOUND * MACH_HOVER_TIP_LIMIT
    MACH_FORWARD_FLIGHT_TIP_LIMIT = 0.95
    FORWARD_FLIGHT_TIP_SPEED_LIMIT = MACH_FORWARD_FLIGHT_TIP_LIMIT * MACH_FORWARD_FLIGHT_TIP_LIMIT  

    def __init__(self, mass_limit=50, max_diameter=4.35):
        self.MASS_LIMIT = mass_limit # kg
        self.MAX_DIAMETER = max_diameter # metres

class DesignAssumptions:

    BLADE_LOADING = 0.175 # coefficient of thrust / solidity. From ROAMX rotor
    DRAG_COEF_MEAN = 1.02 / 28.25 # from Table 4 ROAMX motor paper: https://ntrs.nasa.gov/api/citations/20240000766/downloads/1689_Koning_Final_011924.pdf
    FIGURE_OF_MERIT = 0.65 # from Table 3 ROAMX motor paper: https://ntrs.nasa.gov/api/citations/20240000766/downloads/1689_Koning_Final_011924.pdf

    INDUCED_POWER_FACTOR = 1.2 # from NASA MSH paper

    # assumptions from NASA MSH paper
    MOTOR_EFFICIENCY = 0.8 # 80%
    ROTOR_SERVO_POWER = 0.15 # 15%
    PROPULSIVE_EFFICIENCY = MOTOR_EFFICIENCY * (1 - ROTOR_SERVO_POWER)

    AVIONICS_POWER = 35 # W - NASA MSH paper
    SAMPLING_MECHANISM_POWER = 15 # W - TODO make this more accurate but based on HoneyBee corer for now 
    # -> might want to make larger since optimising for largest payload which would imply larger power
    SAMPLING_TIME = 15 * 60 # seconds
    
    SOLAR_PANEL_POWER_DENSITY = 21.9 # W/m^2 - NASA MSH paper based on MH
    SOLAR_PANEL_MASS_DENSITY = 2.0 # kg/m^2
    USABLE_BATTERY_PERC = 0.7 # 10-80% depth-of-discharge
    BATTERY_DENSITY = 218.5 # Wh/kg - NASA MSH paper - JPL technology forecast 
    ELECTRONICS_MASS = 3 # TODO: make this something reasonable - should represent flight control/avionics weight

class AircraftType:
    CLASSIC_ROTORCRAFT = 1
    TILTROTORCRAFT = 2

class Rotorcraft:

    def __init__(self, no_rotors, no_blades, aircraft_type, mission_scenario, design_constraints, design_assumptions):
        self._mission_scenario = mission_scenario
        self._design_constraints = design_constraints
        self._design_assumptions = design_assumptions

        self._aircraft_type = aircraft_type
        self._no_rotors = no_rotors
        self._no_blades = no_blades

    ##########################################################
    ##### ANALYSES
    ##########################################################

    def calc_and_verify_initial_design(self, design_mass=None):
        self._design_mass = self.dc.MASS_LIMIT if not design_mass else design_mass

        self._thrust = self.calc_max_thrust(self._design_mass)

        self._rotor_radius = self.calc_rotor_radius(self.thrust_per_rotor)
        if self._rotor_radius * 2 * self._no_rotors > self.dc.MAX_DIAMETER:
            raise ValueError("Cannot create required thrust to fit in aeroshell.")
        
        self._thrust_power = self.calc_thrust_power(self.thrust_per_rotor, self.blade_area)

        self._torque = self.calc_torque(self._thrust_power, self._rotor_radius)

        self._energy_per_sol = self.calc_energy_per_sol()
        self._empty_mass = self.calc_empty_mass(self._torque, self._energy_per_sol, self.blade_area)
        
        self._payload = self.calc_payload(self.total_available_mass, self._empty_mass)
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
    
    def calc_rotor_radius(self, thrust_per_rotor):
        blade_area = thrust_per_rotor / (mars_constants.DENSITY * self.da.BLADE_LOADING * self.dc.TIP_SPEED**2)
        rotor_radius = 2.8978 * blade_area ** 0.5
        return rotor_radius
    
    def calc_thrust_power(self, thrust_per_rotor, blade_area):
        thrust_power_per_rotor = self.da.INDUCED_POWER_FACTOR * thrust_per_rotor * \
            np.sqrt(thrust_per_rotor / (2 * mars_constants.DENSITY * self.disk_area)) + \
                mars_constants.DENSITY * blade_area * self.dc.TIP_SPEED**3 * self.da.drag_coef_mean / 8
        return thrust_power_per_rotor * self._no_rotors
    
    def calc_torque(self, thrust_power, rotor_radius):
        rotational_speed = self.da.TIP_SPEED_LIMIT / rotor_radius
        return thrust_power / self.da.PROPULSIVE_EFFICIENCY / rotational_speed        
    
    def calc_energy_per_sol(self):
        flight_energy = self._mission_scenario.get_single_flight_energy(self.forward_velocity, self.hover_power)
        sampling_mechanism_energy = self.da.SAMPLING_MECHANISM_POWER * self.da.SAMPLING_TIME
        avionics_energy = self.da.AVIONICS_POWER * mars_constants.SOL_SECONDS
        sleep_energy = 0.518 * self._design_mass**(1/3) * mars_constants.SOL_SECONDS
        return flight_energy + sampling_mechanism_energy + sleep_energy + avionics_energy 
    
    def calc_empty_mass(self, torque, energy, blade_area):
        motor_mass = 0.076 * torque**0.86 # kg - NASA MSH (based on MH)

        total_power = energy / mars_constants.SOL_SECONDS
        solar_panel_area = total_power / self.da.SOLAR_PANEL_POWER_DENSITY # m^2
        solar_panel_mass = solar_panel_area * self.da.SOLAR_PANEL_MASS_DENSITY # kg

        energy_Wh = energy / (60*60)
        energy_required_Wh = energy_Wh / self.da.USABLE_BATTERY_PERC # TODO: did NASA MSH paper get this wrong and not actually consider USABLE_BATTERY_PERC?
        battery_mass = energy_required_Wh / self.da.BATTERY_DENSITY # NASA MSH paper

        # TODO decide between these methods
        # rotor_mass = 1.1 * blade_area # NASA MSH paper
        rotor_mass = (0.168/0.72) * self._rotor_radius # ROAMX blade correlation between mass and radius
        
        # TODO could consider things like hub, shaft, support arms, fuselage -> however I think this is overspecifying and hence scaling with design mass
        structure_mass = 1/3 * self._design_mass # based on MSH paper designs

        flight_electronics_mass = self.da.ELECTRONICS_MASS

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
    def thrust(self):
        return self._thrust

    @property
    def thrust_per_rotor(self):
        self._thrust_per_rotor = self._thrust / self._no_rotors
        return self._thrust_per_rotor
    
    @property
    def blade_area(self):
        return 0.119087 * self._rotor_radius ** 2
    
    @property
    def disk_area(self):
        return self._rotor_radius**2 * np.pi
    
    @property
    def thrust_power_per_rotor(self):
        self._thrust_power_per_rotor = self._thrust_power / self._no_rotors
        return self._thrust_power_per_rotor
    
    @property
    def hover_power(self):
        return self._thrust_power / self.dc.HOVER_THRUST_CONDITION
    
    @property
    def forward_velocity(self):
        if self._aircraft_type == AircraftType.CLASSIC_ROTORCRAFT:
            return self.dc.FORWARD_FLIGHT_TIP_SPEED_LIMIT - self.dc.HOVER_TIP_SPEED_LIMIT
        # TODO do this for AircraftType.TILTROTORCRAFT

    @property
    def total_power(self):
        return self._energy_per_sol / mars_constants.SOL_SECONDS
    

# mission setup
MASS_LIMIT = 50
mission_scenario = FlightMissionScenario(
    hover_time=5*60, forward_flight_distance=600, climb_height=50, 
    climb_rate=2, descent_rate=2)
design_constraints = DesignConstraints(mass_limit=MASS_LIMIT, max_diameter=4.35)
design_assumptions = DesignAssumptions()

# analysis (for each design)
quad = Rotorcraft(4, 2, AircraftType.CLASSIC_ROTORCRAFT, mission_scenario, design_constraints, design_assumptions)

# initial verification using design_mass = MASS_LIMIT
payload = quad.calc_and_verify_initial_design(MASS_LIMIT)

# payload efficiency tradeoff
# - vary design_mass and calculate payload/design_mass
design_masses, payload_efficiences = quad.payload_efficiency_analysis()

# payload vs hover/range tradeoff
# - fix design_mass (maybe at MASS_LIMIT, maybe at maximum payload efficiency) 
# - decrease the proportion of payload for sampling and move into battery to provide more energy/power
# - back calculate HOVER_TIME or FORWARD_FLIGHT_RANGE from total_power
MIN_PAYLOAD = 2
valid_payloads, extra_hover_times, extra_f_flight_distances = quad.trade_payload_for_battery(MASS_LIMIT, MIN_PAYLOAD)

mass_max_payload_efficiency = design_masses[np.argmax(payload_efficiences)]
valid_payloads, extra_hover_times, extra_f_flight_distances = quad.trade_payload_for_battery(mass_max_payload_efficiency, MIN_PAYLOAD)
