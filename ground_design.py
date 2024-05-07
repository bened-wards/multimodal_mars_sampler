### Resources
# gives Bekker equations and soil characteristic examples
# https://www.diva-portal.org/smash/get/diva2:1424622/FULLTEXT01.pdf

import math
import matplotlib.pyplot as plt

import mars_constants
from rotorcraft import TiltRotorcraft
from aerial_design import mission_setup

class SoilCharacteristics:

    def __init__(self, n, k_c, k_phi, description=""):
        """Represents the characteristics of a section of soil.

        Args:
            n (float): sinkage index
            k_c (float): coefficient of cohesion (kPa/m^(n-1))
            k_phi (float): coefficient of friction (kPa/m^n == kN/m^(n+2)))
            description (str): description of the soil type
        """
        self.n = n
        self.k_c = k_c
        self.k_phi = k_phi
        self.description = description

    def __str__(self):
        return f"SoilCharacteristics({self.description}, n={self.n}, k_c={self.k_c}, k_phi={self.k_phi})"


class Wheel:

    def __init__(self, width, diameter):
        """Wheel

        Args:
            width (float): width of wheel in metres
            diameter (float): diameter of wheel in metres
        """
        self.width = width
        self.diameter = diameter

class Vehicle:

    def __init__(self, mass, no_wheels, wheel: Wheel):
        """Vehicle

        Args:
            mass (float): mass in kg
            no_wheels (int): number of wheels
            wheel (Wheel): type of wheel attached to vehicle
        """
        self.mass = mass
        self.no_wheels = no_wheels
        self.wheel = wheel

    @property
    def weight(self):
        """Weight in N"""
        return self.mass * mars_constants.GRAVITY

class VehicleOnGivenSurface:

    def __init__(self, vehicle: Vehicle, soil_char: SoilCharacteristics, accel_required):
        self.vehicle = vehicle
        self.soil_char = soil_char

        self.accel_required = accel_required # m/s^2 -> based on 30deg angle incline requiring 2.14m/s^2 -> 40% margin

    @property
    def k(self):
        """Multiply by 1000 to convert from kPa/m^n to Pa/m^n to match weight which will be in N"""
        return (self.soil_char.k_c / self.vehicle.wheel.width + self.soil_char.k_phi) * 1000

    def sinkage(self):
        """Sinkage in metres"""
        W = self.vehicle.weight # newtons
        b = self.vehicle.wheel.width # metres
        D = self.vehicle.wheel.diameter # metres
        n = self.soil_char.n # unitless
        return math.pow((3*W) / (b * self.k * math.sqrt(D) * (3-n)), 1/(n+0.5))
    
    def rolling_resistance(self):
        """Rolling resistance in Newtons"""
        b = self.vehicle.wheel.width # metres
        z0 = self.sinkage() # metres
        n = self.soil_char.n
        return self.k * b * math.pow(z0, n+1) / (n+1)
    
    def force_required_per_wheel(self):
        """Force in Newtons"""
        return self.vehicle.mass / self.vehicle.no_wheels * self.accel_required + self.rolling_resistance()
    

class ActiveWheeledVehicleOnGivenSurface(VehicleOnGivenSurface):

    def __init__(self, vehicle: Vehicle, soil_char: SoilCharacteristics, accel_required=3):
        super().__init__(vehicle, soil_char, accel_required)

    def torque_required_per_wheel(self):
        return self.force_required_per_wheel() * self.vehicle.wheel.diameter / 2
                
class TiltrotorVehicleOnGivenSurface(VehicleOnGivenSurface):

    def __init__(self, vehicle: Vehicle, soil_char: SoilCharacteristics, accel_required=3):
        super().__init__(vehicle, soil_char, accel_required)

        self.margin = 1.5 # 150% margin for any losses in force lost due to moment pushing into ground
    
    def thrust_required(self):
        return self.force_required_per_wheel() * self.vehicle.no_wheels * self.margin

def tiltrotor_plot():
    ## Tiltrotor calculations
    mission_scenario, design_constraints, design_assumptions = mission_setup(max_diameter=1000)
    tiltrotor = TiltRotorcraft("Hex-tiltrotor", 6, 4, 4, mission_scenario, design_constraints, design_assumptions)
    tiltrotor.calc_and_verify_initial_design(MASS)
    no_forward_rotors = tiltrotor._no_rotors - tiltrotor._no_nontilt_rotors
    
    plt.figure()
    for effective_ground_mass in [10, 15, 20, 30, 50]:
        thrusts_required = []
        for soil_char in SOIL_CHARS:    
            vehicle = Vehicle(effective_ground_mass, NO_WHEELS, WHEEL)
            tiltrotor_ground = TiltrotorVehicleOnGivenSurface(vehicle, soil_char)
            thrust_required = tiltrotor_ground.thrust_required()
            thrusts_required.append(thrust_required)
            if thrust_required > tiltrotor.hover_thrust_per_rotor * no_forward_rotors:
                print(f"Tilt rotor cannot produce required thrust ({thrust_required:.2f}N > {tiltrotor.hover_thrust_per_rotor * no_forward_rotors:.2f}N) for {soil_char}")
            else:
                print(f"Tilt rotor can produce required thrust ({thrust_required:.2f}N < {tiltrotor.hover_thrust_per_rotor * no_forward_rotors:.2f}N) for {soil_char}")

        plt.plot(range(1,len(thrusts_required)+1), thrusts_required, marker='o', label=f'EGM={effective_ground_mass:.2f}kg')
    
    plt.plot(range(1,len(thrusts_required)+1), [tiltrotor.hover_thrust_per_rotor * no_forward_rotors]*len(thrusts_required), label='Maximum thrust producible', color="black")
    
    plt.title("Horizontal thrust required for varying effective ground masses (EGM)")
    plt.xlabel("Soil type")
    plt.ylabel("Thrust (N)")
    plt.xlim([1, len(thrusts_required)])
    plt.ylim([0, 500])
    plt.legend(loc='upper right', fontsize=9)

    plt.figure()
    for a in [1,2,3]:
        thrusts_required = []
        for soil_char in SOIL_CHARS:    
            tiltrotor_ground = TiltrotorVehicleOnGivenSurface(VEHICLE, soil_char, a)
            thrust_required = tiltrotor_ground.thrust_required()
            thrusts_required.append(thrust_required)
            if thrust_required > tiltrotor.hover_thrust_per_rotor * no_forward_rotors:
                print(f"Tilt rotor cannot produce required thrust ({thrust_required:.2f}N > {tiltrotor.hover_thrust_per_rotor * no_forward_rotors:.2f}N) for {soil_char}")
            else:
                print(f"Tilt rotor can produce required thrust ({thrust_required:.2f}N < {tiltrotor.hover_thrust_per_rotor * no_forward_rotors:.2f}N) for {soil_char}")

        plt.plot(range(1,len(thrusts_required)+1), thrusts_required, marker='o', label=f'Horizontal thrust required, a={a}')
    
    plt.plot(range(1,len(thrusts_required)+1), [tiltrotor.hover_thrust_per_rotor * no_forward_rotors]*len(thrusts_required), label='Maximum thrust producible', color="black")
    
    plt.title("Horizontal thrust required for varying accelerations required (50kg)")
    plt.xlabel("Soil type")
    plt.ylabel("Thrust (N)")
    plt.xlim([1, len(thrusts_required)])
    plt.ylim([0, 500])
    plt.legend(loc='upper right', fontsize=9)
    plt.show()

def active_wheel_calcs():
    ## Active wheel calculations
    desired_velocity = 0.1 # m/s
    for soil_char in SOIL_CHARS:
        print(f"\nActive wheeled with {soil_char}")
        active_ground = ActiveWheeledVehicleOnGivenSurface(VEHICLE, soil_char)
        torque_per_wheel = active_ground.torque_required_per_wheel()
        print(f"Torque required per motor is {torque_per_wheel:.2f}N.m")
        print(f"Power required by force*velocity is {active_ground.force_required_per_wheel() * desired_velocity:.2f}W")
        gear_ratio = torque_per_wheel / NOMINAL_TORQUE
        rpm = NOMINAL_RPM / gear_ratio
        velocity = rpm * 2 * math.pi / 60 * active_ground.vehicle.wheel.diameter
        print(f"Velocity achieved at {NOMINAL_POWER}W with gear ratio {gear_ratio:.2f} is {velocity:.2f}m/s={velocity*3.6:.2f}km/h")


## different wheels effect on rolling resistance, torque and velocity
def plot_wheel_characteristics():
    widths = [0.1, 0.15, 0.2, 0.25, 0.3] # metres
    diameters = [0.2, 0.25, 0.3, 0.35, 0.4] # metres
    # diameters = [0.3] * len(widths) # metres
    widths = [0.2] * len(diameters) # metres
    # diameters = [width*2 for width in widths] # metres
    plt.figure(figsize=(12,5))
    for width, diameter in zip(widths, diameters):
        wheel = Wheel(width, diameter)
        vehicle = Vehicle(MASS, NO_WHEELS, wheel)
        sinkages = []
        rolling_resistances = []
        torques_per_wheel = []
        velocities = []
        for soil_char in SOIL_CHARS:
            active_ground = ActiveWheeledVehicleOnGivenSurface(vehicle, soil_char)
            sinkages.append(active_ground.sinkage()*100)
            rolling_resistances.append(active_ground.rolling_resistance())
            torque_per_wheel = active_ground.torque_required_per_wheel()
            torques_per_wheel.append(torque_per_wheel)
            gear_ratio = torque_per_wheel / NOMINAL_TORQUE
            rpm = NOMINAL_RPM / gear_ratio
            velocity = rpm * 2 * math.pi / 60 * diameter
            velocities.append(velocity)
            # plt.scatter(active_ground.sinkage()/100, active_ground.rolling_resistance(), label=f"{soil_char.description}")
        
        # sort by sinkage but maintain order
        comb = list(zip(sinkages, rolling_resistances, torques_per_wheel, velocities))
        comb.sort(key=lambda x: x[0])
        sinkages, rolling_resistances, torques_per_wheel, velocities = zip(*comb)

        plt.subplot(1,3,1)
        plt.plot(sinkages, rolling_resistances, marker='o', label=f"d={int(wheel.diameter*100)}cm, w={int(wheel.width*100)}cm")

        plt.subplot(1,3,2)
        plt.plot(sinkages, torques_per_wheel, marker='o', label=f"d={int(wheel.diameter*100)}cm, w={int(wheel.width*100)}cm")

        plt.subplot(1,3,3)
        plt.plot(sinkages, velocities, marker='o', label=f"d={int(wheel.diameter*100)}cm, w={int(wheel.width*100)}cm")
        
    plt.subplot(1,3,1)
    plt.xlabel("Sinkage (cm)")
    plt.ylabel("Rolling resistance (N)")
    plt.xlim([-0.1, 2.2])
    plt.ylim([0, 47])

    plt.subplot(1,3,2)
    plt.xlabel("Sinkage (cm)")
    plt.ylabel("Torque per wheel (N.m)")
    plt.xlim([-0.1, 2.2])
    plt.ylim([0, 13.5])
    plt.title("Each point is a different soil type", fontsize=9)
    # plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2)
    # # to create legend for report
    # plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)

    plt.subplot(1,3,3)
    plt.xlabel("Sinkage (cm)")
    plt.ylabel("Velocity (m/s)")
    plt.xlim([-0.1, 2.2])
    plt.ylim([0, 0.35])

    plt.suptitle("Rolling resistance, torque and velocity vs sinkage for varying wheel diameters and widths")
    plt.tight_layout()
    plt.show()


def print_wheel_grid():
    """Choose soil characteristic that produces most work required = fine dust medium density"""
    widths = [0.1, 0.2, 0.3] # metres
    diameters = [0.2, 0.3, 0.4] # metres
    combinations = [(width, diameter) for width in widths for diameter in diameters]
    soil_char = SOIL_CHARS[3]
    print(f"\n\nDiffering wheel characteristics for soil: {soil_char.description}")
    for width, diameter in combinations:
        wheel = Wheel(width, diameter)
        vehicle = Vehicle(MASS, NO_WHEELS, wheel)
        
        active_ground = ActiveWheeledVehicleOnGivenSurface(vehicle, soil_char)
        sinkage = active_ground.sinkage()*100 # cm
        rolling_resistance = active_ground.rolling_resistance() # N
        torque_per_wheel = active_ground.torque_required_per_wheel() # N.m
        gear_ratio = torque_per_wheel / NOMINAL_TORQUE
        rpm = NOMINAL_RPM / gear_ratio
        velocity = rpm * 2 * math.pi / 60 * diameter # m/s

        print(f"Wheel(w={width},d={diameter}): R={rolling_resistance:.3f}N, sinkage={sinkage:.3f}cm, T={torque_per_wheel:.3f}N.m per wheel, v={velocity:.3f}m/s")
        
# GLOBALS
MASS = 50
NO_WHEELS = 4
WHEEL_WIDTH = 0.2 # metres
WHEEL_DIAMETER = 0.3 # metres
WHEEL = Wheel(WHEEL_WIDTH, WHEEL_DIAMETER)
VEHICLE = Vehicle(MASS, NO_WHEELS, WHEEL)

SOIL_CHARS = [
    SoilCharacteristics(0.67, 67.28, 0.68, "FineDust,LowDensity"),
    SoilCharacteristics(0.71, 61.96, 1.30, "FineDust,MedDensity"),
    SoilCharacteristics(0.75, 142.36, 1.66, "FineDust,HighDensity"),
    SoilCharacteristics(0.92, 1727.51, -14.12, "CoarseSand,LowDensity"),
    SoilCharacteristics(0.87, 1931.13, -16.41, "CoarseSand,MedDensity"),
    SoilCharacteristics(0.76, 2312.59, -30.10, "CoarseSand,HighDensity")
]
# motor characteristics
NOMINAL_VOLTAGE = 24 #V
NOMINAL_CURRENT = 0.5 #A
NOMINAL_RPM = 2760 
NOMINAL_TORQUE = 0.0255 # N.m
NOMINAL_POWER = NOMINAL_VOLTAGE * NOMINAL_CURRENT

if __name__ == "__main__":
    tiltrotor_plot()
    # active_wheel_calcs()
    # plot_wheel_characteristics()
    # print_wheel_grid()
