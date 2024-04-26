import mars_constants

class FlightMissionScenario:

    def __init__(self, hover_time=5*60, forward_flight_speed=30, forward_flight_distance=600, climb_height=50, climb_rate=2, descent_rate=2):
        """Parameters for planned mission scenario (flight per sol)

        Args:
            hover_time (float): hover time in seconds
            forward_flight_speed(float): forward flight speed in m/s
            forward_flight_distance (float): forward flight distance in metres
            climb_height (float): height to climb to from initial takeoff
            climb_rate (float): climbing rate in m/s
            descent_rate(float): absolute value of descent rate in m/s
        """
        self.HOVER_TIME = hover_time
        self.FORWARD_FLIGHT_SPEED = forward_flight_speed
        self.FORWARD_FLIGHT_DISTANCE = forward_flight_distance
        self.CLIMB_HEIGHT = climb_height
        self.CLIMB_RATE = climb_rate
        self.DESCENT_RATE = descent_rate

    def get_single_flight_energy(self, hover_power, f_flight_power, avionics_power, climb_power_factor=1.5, descent_power_factor=1.5):
        """Calculate the energy required for a single flight according to mission scenario

        Args:
            hover_power (float): hover power of the rotorcraft in Watts
            forward_velocity (float): velocity of the rotorcraft in m/s
        """
        climb_time = self.CLIMB_HEIGHT / self.CLIMB_RATE
        climb_power = hover_power * climb_power_factor
        climb_energy = climb_power * climb_time # Joules

        f_flight_time = self.FORWARD_FLIGHT_DISTANCE / self.FORWARD_FLIGHT_SPEED
        f_flight_energy = f_flight_power * f_flight_time

        hover_energy = hover_power * self.HOVER_TIME

        descent_time = self.CLIMB_HEIGHT / self.DESCENT_RATE
        descent_power = descent_power_factor * hover_power
        descent_energy = descent_power * descent_time

        total_time = climb_time + f_flight_time + self.HOVER_TIME + descent_time
        avionics_energy = avionics_power * total_time

        return climb_energy + hover_energy + f_flight_energy + descent_energy + avionics_energy
    
class DesignConstraints:
    CONTINGENCY_WEIGHT_FACTOR = 0.2 # 20%

    HOVER_THRUST_CONDITION = 1.5 # 150%

    MACH_TIP_LIMIT = 0.8 
    MACH_ADVANCING_TIP_LIMIT = 0.95

    def __init__(self, mass_limit=50, max_diameter=4.35):
        self.MASS_LIMIT = mass_limit # kg
        self.MAX_DIAMETER = max_diameter # metres

    @property
    def TIP_SPEED_LIMIT(self):
        return mars_constants.SPEED_OF_SOUND * self.MACH_TIP_LIMIT

    @property
    def ADVANCING_TIP_SPEED_LIMIT(self):
        return mars_constants.SPEED_OF_SOUND * self.MACH_ADVANCING_TIP_LIMIT  
    

class DesignAssumptions:

    BLADE_LOADING = 0.175 # coefficient of thrust / solidity. From ROAMX rotor
    DRAG_COEF_MEAN = 1.02 / 28.25 # from Table 4 ROAMX motor paper: https://ntrs.nasa.gov/api/citations/20240000766/downloads/1689_Koning_Final_011924.pdf

    # TODO - what angle did Ingenuity fly at?
    FORWARD_FLIGHT_TILT_ANGLE = 15 # degrees

    # assumptions from NASA MSH paper
    MOTOR_EFFICIENCY = 0.8 # 80%
    # power reserved for servo pitch control moved into aircraft classes
    # due to differences between coaxial and conventional

    AVIONICS_POWER = 35 # W - NASA MSH paper
    SAMPLING_MECHANISM_POWER = 15 # W - TODO make this more accurate but based on HoneyBee corer for now 
    # -> might want to make larger since optimising for largest payload which would imply larger power
    SAMPLING_TIME = 15 * 60 # seconds

    GROUND_MOBILITY_POWER = 100 # W - TODO make this more accurate - somewhat random for now
    GROUND_MOBILITY_TIME = 5 * 60 # seconds
    GROUND_MOBILITY_MASS_PROPORTION = 0.05 # 5%
    
    # assumption that NASA paper considered daylight hours in their calculation
    SOLAR_PANEL_ENERGY_PER_SOL = 540 * 60 * 60 # J/m^2 available in one sol (daylight charging) - NASA MSH paper
    # SOLAR_PANEL_POWER_DENSITY = 21.9 # W/m^2 - NASA MSH paper based on MH
    SOLAR_PANEL_MASS_DENSITY = 2.0 # kg/m^2
    BATTERY_CONTINGENCY = 1.2 # 120% of mission energy
    USABLE_BATTERY_PERC = 0.7 # 10-80% depth-of-discharge
    BATTERY_DENSITY = 218.5 # Wh/kg - NASA MSH paper - JPL technology forecast 
    ELECTRONICS_MASS = 2.65 # TODO: make this something reasonable - should represent flight control/avionics weight

    # extra mass factors
    ROTOR_MASS_FACTOR = 1
    MOTOR_MASS_FACTOR = 1
    