import math
import numpy as np
import random

NUM_PARTICLES = 1000

MEASUREMENTS = np.load("./precise_measurement_model_filled_shifted.npy")
MEASUREMENTS = np.clip(MEASUREMENTS, 0, 96.0)

class Rover:
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.theta = position[2]
        self.vel_forward = 0
        self.vel_angular = 0

class Particle:

    BIAS_FORWARD = 0.0  # forward bias when driving forwards (in percent of forward distance)
    STD_DEV_FORWARD = 0.05  # forward noise when driving forwards (in percent of forward distance)
    BIAS_LATERAL = 0.0  # lateral bias when driving forwards (in degrees per inch of forward distance)
    STD_DEV_LATERAL = 1.0  # lateral noise when driving forwards (in degrees per inch of forward distance)
    BIAS_ANGULAR = 0.0  # bias when rotating (in degrees)
    STD_DEV_ANGULAR = 0.02 # angular noise when rotating (in percent of angular rotation)

    BIAS_SENSOR = 0.0 # bias for sensor_reading vs particle expected measurement (in inches)
    STD_DEV_SENSOR = 2.0 # standard deviation for sensor_reading vs particle expected measurement (in inches)

    def __init__(self, position, weight=None):
        self.x = position[0]
        self.y = position[1]
        self.theta = position[2]
        if weight is not None:
            self.weight = weight
        else:
            self.weight = 1.0 / NUM_PARTICLES

    def calculate_expected_measurements(self) -> tuple:
        """
            Calculate expected sensor readings of a single particle using bilinear interpolation.
        """
        y_int = int((self.y - 3.75) / 0.75)
        x_int = int((self.x - 3.75) / 0.75)
        r_int = int(((-self.theta) % 360) / 15)

        x_frac = ((self.x - 3.75) / 0.75) - x_int
        y_frac = ((self.y - 3.75) / 0.75) - y_int
        r_frac = ((-self.theta % 360) / 15) - r_int

        interpolated_measurements = np.zeros((4, 6))

        for i in range(6):
            interpolated_measurements[0][i] = (MEASUREMENTS[y_int][x_int][(r_int + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[y_int][x_int][(r_int + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[1][i] = (MEASUREMENTS[y_int][x_int + 1][(r_int + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[y_int][x_int + 1][(r_int + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[2][i] = (MEASUREMENTS[y_int + 1][x_int][(r_int + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[y_int + 1][x_int][(r_int + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[3][i] = (MEASUREMENTS[y_int + 1][x_int + 1][(r_int + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[y_int + 1][x_int + 1][(r_int + 1 + (4 * i)) % 24] * (r_frac))
        expected_measurements = Particle.bilinear_interpolation(interpolated_measurements, x_frac, y_frac)
        return expected_measurements
    
    def move(self, forward: float, angular: float):
        """Move the particle with added Gaussian noise."""
        noisy_forward = forward + np.random.normal(0, Particle.STD_DEV_FORWARD)
        noisy_angular = angular + np.random.normal(0, Particle.STD_DEV_ANGULAR)

        self.x += noisy_forward * math.cos(math.radians(self.theta))
        self.y += noisy_forward * math.sin(math.radians(self.theta))
        self.theta = (self.theta + noisy_angular) % 360

    def update_weights(self, sensor_readings: np.ndarray):
        expected_measurements = self.calculate_expected_measurements()
        prob = 1.0
        for i in range(6):
            prob *= Particle.normal_pdf(expected_measurements[i], sensor_readings[i], Particle.STD_DEV_SENSOR)
        self.weight = prob

    @staticmethod
    def move_fast_euler_step_approximation(particles: np.ndarray, forward: float, angular: float) -> np.ndarray:
        """Vectorized move function for multiple particles using Euler step approximation."""
        if forward != 0:
            step_size = 1.0  # Define a suitable step size
            num_steps = abs(int(forward / step_size))
            step_forward = forward / num_steps
            for _ in range(num_steps):
                particles = Particle.move_fast(particles, step_forward, 0)
            remainder = forward - step_forward * num_steps
            particles = Particle.move_fast(particles, remainder, angular)
        elif angular != 0:
            particles = Particle.move_fast(particles, 0, angular)
        return particles

    @staticmethod
    def move_fast(particles: np.ndarray, forward: float, angular: float) -> np.ndarray:
        """Vectorized move function for multiple particles."""
        noisy_forward = forward * (1 + Particle.BIAS_FORWARD + np.random.normal(0, Particle.STD_DEV_FORWARD, size=particles.shape[0]))
        noisy_lateral = abs(forward) * (Particle.BIAS_LATERAL + np.random.normal(0, Particle.STD_DEV_LATERAL, size=particles.shape[0]))
        noisy_angular = angular * (1 + np.random.normal(0, Particle.STD_DEV_ANGULAR, size=particles.shape[0]))

        particles[:, 0] += noisy_forward * np.cos(np.radians(-(particles[:, 2] - 90)))
        particles[:, 1] += noisy_forward * np.sin(np.radians(-(particles[:, 2] - 90)))
        particles[:, 2] = (particles[:, 2] + noisy_angular + noisy_lateral) % 360

        return particles
    
    @staticmethod
    def update_weights_fast(sensor_readings: np.ndarray, particle_weights: np.ndarray, expected_measurements: np.ndarray) -> float:
        """
            Update weights for multiple particles based on sensor readings.
            sensor_readings: np.ndarray of shape (6,) - actual sensor readings
            expected_measurements: np.ndarray of shape (N, 6) - expected measurements for N particles
            Returns updated expected_measurements with weights in the last column.
        """
        # for i in range(6):
        for i in range(4):
            particle_weights *= np.reshape(Particle.normal_pdf(expected_measurements[:, i], sensor_readings[i], Particle.STD_DEV_SENSOR), (-1, 1))
        return particle_weights

    @staticmethod
    # Similar to update_weights_fast, but the standard deviation for a given sensor is higher if the reading is higher
    def update_weights_fast_variable_std(sensor_readings: np.ndarray, particle_weights: np.ndarray, expected_measurements: np.ndarray) -> float:
        """
            Update weights for multiple particles based on sensor readings with variable standard deviation.
            sensor_readings: np.ndarray of shape (6,) - actual sensor readings
            expected_measurements: np.ndarray of shape (N, 6) - expected measurements for N particles
            Returns updated expected_measurements with weights in the last column.
        """
        for i in range(6):
            std_devs = Particle.STD_DEV_SENSOR * (1 + sensor_readings[i] / (72.0 / 2))  # Increase std dev for higher readings
            particle_weights *= np.reshape(Particle.normal_pdf(expected_measurements[:, i], sensor_readings[i], std_devs), (-1, 1))
        return particle_weights

    @staticmethod
    def normal_pdf(x: np.ndarray, mean: float, std_dev: float) -> float | np.ndarray:
        """Calculate the normal probability density function."""
        exponent = -0.5 * ((x - mean) / std_dev) ** 2
        res = (1 / (std_dev * np.sqrt(2 * math.pi))) * np.exp(exponent)
        print(res.shape)
        return res

    @staticmethod
    def bilinear_interpolation(readings: np.ndarray, x_frac: float, y_frac: float) -> np.ndarray:
        """
            Perform bilinear interpolation to get expected sensor readings of a single particle.
        """
        top = readings[0, :] * (1 - x_frac) + readings[1, :] * x_frac
        bottom = readings[2, :] * (1 - x_frac) + readings[3, :] * x_frac
        return top * (1 - y_frac) + bottom * y_frac

    @staticmethod
    def bilinear_interpolation_fast(readings: np.ndarray, x_frac: np.ndarray, y_frac: np.ndarray) -> np.ndarray:
        """
            Perform vectorized bilinear interpolation to get expected sensor readings of multiple particles.
        """
        top = readings[:, 0, :] * (1 - x_frac[:, np.newaxis]) + readings[:, 1, :] * x_frac[:, np.newaxis]
        bottom = readings[:, 2, :] * (1 - x_frac[:, np.newaxis]) + readings[:, 3, :] * x_frac[:, np.newaxis]
        return top * (1 - y_frac[:, np.newaxis]) + bottom * y_frac[:, np.newaxis]

    @staticmethod
    def calculate_expected_measurements_fast(states: np.ndarray, debug=False) -> np.ndarray:
        """
            states: np.ndarray of shape (N, 3) where each row is (x, y, theta)
        """

        # Vectorized version to calculate expected sensor readings for multiple states
        states_copy = states.copy()
        # print rows and indices of arr1 where any value is less than 3.75 or greater than 96 - 3.75 for x, or less than 3.75 or greater than 48 - 3.75 for y
        # invalid_mask = (arr1[:, 0] < 3.75) | (arr1[:, 0] > 96 - 3.75) | (arr1[:, 1] < 3.75) | (arr1[:, 1] > 48 - 3.75)
        # print("Invalid states:", np.where(invalid_mask)[0])
        # print("Invalid states values:", arr1[invalid_mask])
        # If any invalid states, raise an error
        # if np.any(invalid_mask):
        #     input("Some states are out of bounds. Press Enter to continue...")
        #     raise ValueError("Some states are out of bounds for measurement model.")
        states_copy[:, :2] = ((states_copy[:, :2] - 3.75) / 0.75).astype(np.int32)
        states_copy[:, 2] = ((states_copy[:, 2] % 360) / 15).astype(np.int32)
        states_copy = states_copy.astype(np.int32)

        x_frac = ((states[:, 0] - 3.75) / 0.75) - states_copy[:, 0]
        y_frac = ((states[:, 1] - 3.75) / 0.75) - states_copy[:, 1]
        r_frac = ((states[:, 2] % 360) / 15) - states_copy[:, 2]

        interpolated_measurements = np.zeros((states_copy.shape[0], 4, 6))

        if debug:
            print((states[:, 2] % 360) / 15, (states_copy[:, 2]))
            print("States copy (indices):", states_copy)
            print("x_frac:", x_frac, "y_frac:", y_frac, "r_frac:", r_frac)
            print(MEASUREMENTS[states_copy[:, 1], states_copy[:, 0]])
            print(MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0]])
            print(MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1])
            print(MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1])

            # Roll each MEASUREMENTS[states_copy[:, 1], states_copy[:, 0]] by states_copy[:, 2] along last axis and print
            for i in range(6):
                rolled = np.roll(MEASUREMENTS[states_copy[:, 1], states_copy[:, 0]], -4 * i, axis=1)
                print(f"Rolled measurements for sensor {i} at states:")
                print(rolled)

        # print(states, states.dtype)
        for i in range(6):
            interpolated_measurements[:, 0, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 1, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 2, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + 1 + (4 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 3, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + 4 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + (4 * i)) % 24] * (r_frac))

        expected_measurements = Particle.bilinear_interpolation_fast(interpolated_measurements, x_frac, y_frac)
        return expected_measurements
    
    @staticmethod
    def calculate_expected_measurements_fast_90_deg(states: np.ndarray) -> np.ndarray:
        """
            Used for M2 with 4 sensors at 90 degree intervals.
        """
        # Accounting for new sensor arrangement
        states_copy = states.copy()
        states_copy[:, :2] = ((states_copy[:, :2] - 3.75) / 0.75).astype(np.int32)
        states_copy[:, 2] = ((states_copy[:, 2] % 360) / 15).astype(np.int32)
        states_copy = states_copy.astype(np.int32)

        x_frac = ((states[:, 0] - 3.75) / 0.75) - states_copy[:, 0]
        y_frac = ((states[:, 1] - 3.75) / 0.75) - states_copy[:, 1]
        r_frac = ((states[:, 2] % 360) / 15) - states_copy[:, 2]
        interpolated_measurements = np.zeros((states_copy.shape[0], 4, 4))

        offsets = [0, 6, 18, 12, None, None]
        print(MEASUREMENTS[states_copy[:, 1], states_copy[:, 0]])
        for i in range(4):
            interpolated_measurements[:, 0, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + offsets[i]) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + 1 + offsets[i]) % 24] * (r_frac))
            interpolated_measurements[:, 1, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + offsets[i]) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + offsets[i]) % 24] * (r_frac))
            interpolated_measurements[:, 2, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + offsets[i]) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + 1 + offsets[i]) % 24] * (r_frac))
            interpolated_measurements[:, 3, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + offsets[i]) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + offsets[i]) % 24] * (r_frac))
        expected_measurements = Particle.bilinear_interpolation_fast(interpolated_measurements, x_frac, y_frac)
        return expected_measurements


    @staticmethod
    def calculate_expected_measurements_fast_12(states: np.ndarray, debug=False) -> np.ndarray:
        """
            states: np.ndarray of shape (N, 3) where each row is (x, y, theta)
        """

        # Vectorized version to calculate expected sensor readings for multiple states
        states_copy = states.copy()
        # print rows and indices of arr1 where any value is less than 3.75 or greater than 96 - 3.75 for x, or less than 3.75 or greater than 48 - 3.75 for y
        states_copy[:, :2] = ((states_copy[:, :2] - 3.75) / 0.75).astype(np.int32)
        states_copy[:, 2] = ((states_copy[:, 2] % 360) / 15).astype(np.int32)
        states_copy = states_copy.astype(np.int32)

        x_frac = ((states[:, 0] - 3.75) / 0.75) - states_copy[:, 0]
        y_frac = ((states[:, 1] - 3.75) / 0.75) - states_copy[:, 1]
        r_frac = ((states[:, 2] % 360) / 15) - states_copy[:, 2]

        interpolated_measurements = np.zeros((states_copy.shape[0], 4, 12))

        # print(states, states.dtype)
        for i in range(12):
            interpolated_measurements[:, 0, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + 2 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0], (states_copy[:, 2] + 1 + (2 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 1, i] = (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + 2 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1], states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + (2 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 2, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + 2 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0], (states_copy[:, 2] + 1 + (2 * i)) % 24] * (r_frac))
            interpolated_measurements[:, 3, i] = (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + 2 * i) % 24] * (1 - r_frac)) + (MEASUREMENTS[states_copy[:, 1] + 1, states_copy[:, 0] + 1, (states_copy[:, 2] + 1 + (2 * i)) % 24] * (r_frac))

        expected_measurements = Particle.bilinear_interpolation_fast(interpolated_measurements, x_frac, y_frac)
        return expected_measurements
    
    @staticmethod
    def perturb_particles(particle_array, certainty):
        # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
        noise_scale = max(1 - certainty, 0)
        x_noise = np.random.normal(0, 0.5 + 2 * noise_scale, size=particle_array.shape[0])
        y_noise = np.random.normal(0, 0.5 + 2 * noise_scale, size=particle_array.shape[0])
        theta_noise = np.random.normal(0, 10, size=particle_array.shape[0])

        particle_array[:, 0] += x_noise
        particle_array[:, 1] += y_noise
        particle_array[:, 2] = (particle_array[:, 2] + theta_noise) % 360

        return particle_array
    
if __name__ == "__main__":
    print(MEASUREMENTS.shape)
    # Simple test
    p = Particle((56, 42.0, -90.0))
    p = Particle((6.0, 6.0, 0))
    p = Particle((30.0, 30.0, 45))
    print("Initial position:", p.x, p.y, p.theta)
    # Print expected measurements using Particles.calculate_expected_measurements_fast
    particle_array = np.array([[p.x, p.y, p.theta]])
    expected_measurements = Particle.calculate_expected_measurements_fast(particle_array, debug=True)
    print("Expected measurements:", expected_measurements[0])

    # Visualize particle
    from sample_particles import visualize_particles
    particle_array = np.array([[p.x, p.y, p.theta]])
    visualize_particles(particle_array)
    