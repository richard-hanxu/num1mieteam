import shapely.geometry as geom
import shapely.ops as ops
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from filterpy.monte_carlo import stratified_resample
import sys

from mcl_helpers import Rover, Particle, MEASUREMENTS
from planner import plan_path
from sample_particles import CLIP_SENSOR_VALUES, MAX_SENSOR_RANGE, WALLS, WALLS_WITH_BUFFER, OUTER, SAMPLING_REGION, sample_points, sample_starting_points, check_if_contains_points_vectorized, visualize_particles
from tcp_helpers import *

def get_sensor_readings() -> np.ndarray:
    """
    Simulate ultrasonic sensor readings based on the robot's position in simmer.
    
    IMPORTANT: YOU MAY NEED TO ADJUST THIS FUNCTION BASED ON THE NUMBER OF SENSORS YOU WANT TO USE
    ON YOUR ROBOT AND HOW THEY ARE POSITIONED
    """
    packet_tx = packetize("s0,s1,s2,s3,s4,s5")
    transmit(packet_tx)
    [responses, time_rx] = receive()
    while True:
        if responses[0] and (len(responses[0]) > 1 and responses[0][1] != "False"):
            break
    # Parse sensor readings into a numpy array
    sensor_values = []
    for resp in responses:
        if resp[0].startswith("s"):
            sensor_values.append(float(resp[1]))
    sensor_array = np.asarray(sensor_values)
    if CLIP_SENSOR_VALUES:
        sensor_array = np.clip(sensor_array, 0, MAX_SENSOR_RANGE)
    print(f"Sensor readings: {sensor_array}")
    return sensor_array

def predict_position_and_variance(particles_array: np.ndarray, 
                                  sensor_array: np.ndarray,
                                  position: np.ndarray | None,
                                  variance: np.ndarray | None) -> tuple:
    """
    Performs particle localization to:
    - Update particle weights based on sensor readings
    - Estimate the robot's current position and variance based on weighted particles
    - Resample particles based on weights

    Parameters:
    - particles_array: np.ndarray of shape (N, 3) where each row is (x, y, theta)
    - sensor_array: np.ndarray of shape (M,) where each element is a sensor reading
    - position: np.ndarray of shape (3,) representing previous estimated position (x, y, theta) or None
    - variance: np.ndarray of shape (2,) representing previous variance in (x, y
    Returns:
    - tuple: (predicted_position: np.ndarray of shape (3,), predicted_variance: np.ndarray of shape (2,))

    IMPORTANT: YOU MUST MAKE SURE THAT YOUR SENSOR_ARRAY MEASUREMENTS MATCH THE EXPECTED ORDER IN THE
    CALCULATION OF EXPECTED MEASUREMENTS IN MCL_HELPERS.PY

    """
    # Calculate expected measurements for each particle
    expected_measurements = Particle.calculate_expected_measurements_fast(particles_array)
    weights = np.ones((particles_array.shape[0], 1))
    weights = Particle.update_weights_fast(sensor_array, weights, expected_measurements)

    # For visualization purposes, we plot a heatmap where each cell contains the largest weight of any particle in that cell
    plt.figure(figsize=(12, 6))
    heatmap = np.zeros((16, 32))
    for i in range(particles_array.shape[0]):
        x_idx = int(particles_array[i, 0] // 3)
        y_idx = int(particles_array[i, 1] // 3)
        heatmap[y_idx, x_idx] = max(heatmap[y_idx, x_idx], weights[i, 0])
    plt.imshow(heatmap, extent=[0, 96, 48, 0], cmap="hot", interpolation="nearest")
    plt.colorbar(label="Max Particle Weight")
    plt.title("Particle Weight Heatmap")
    plt.show(block=False)

    position = np.average(particles_array, weights=weights.flatten(), axis=0)
    variance_x = np.average((particles_array[:, 0] - position[0])**2, weights=weights.flatten())
    variance_y = np.average((particles_array[:, 1] - position[1])**2, weights=weights.flatten())
    variance = np.array([variance_x, variance_y])
    return position, variance, weights

def main_loop():
    default_particles = sample_starting_points()
    random_particles = sample_points(OUTER, SAMPLING_REGION, num_samples=5000)
    all_particles = default_particles + random_particles

    # --- Initialize particles ---
    particle_array = np.asarray([[p[0], p[1], p[2]] for p in all_particles])

    # --- Initialize sensor readings (reading 4 measurements) ---
    sensor_array = np.zeros((4,))

    # --- Initialize predicted position ---
    predicted_position = None # will be a ndarray of shape (3,)
    predicted_position_variance = None # will be a ndarray of shape (2,) for x and y variance

    # --- Initialize destination ---
    destination = np.array([6.0, 6.0])  # x, y in inches

    # --- MAIN LOOP ---
    while True:
        user_input = input(
            """
            Enter w0:xx or r0:: to send a manual command to the robot in simmer
            Enter 'plan' to have the robot follow the output of your path planner
            Enter 'restart' to reinitialize particles (effectively restarting localization)
            Enter 'dest' to set a new destination for the robot (enter as x, y in inches)
            Enter 'info' to print all current localization info
            Enter 'exit' to quit
            """
        )
        # --- Quit the program ---
        if user_input.lower() == "exit":
            print("Exiting localization loop.")
            break
    
        # --- Restart localization ---
        elif user_input.lower() == "restart":
            print("Reinitializing particles...")
            default_particles = sample_starting_points()
            random_particles = sample_points(OUTER, SAMPLING_REGION, num_samples=5000)
            all_particles = default_particles + random_particles
            particle_array = np.asarray([[p[0], p[1], p[2]] for p in all_particles])
            predicted_position = None
            predicted_position_variance = None
            print(f"Reinitialized to {len(all_particles)} particles.")
            continue

        # --- Print current localization info ---
        elif user_input.lower() == "info":
            print(f"Current Sensor readings: {sensor_array}")
            print(f"Number of particles: {len(particle_array)}")
            if predicted_position is not None:
                print(f"Predicted position: x={predicted_position[0]:.2f}, y={predicted_position[1]:.2f}, theta={predicted_position[2]:.2f}")
            else:
                print("Predicted position: None")
            if predicted_position_variance is not None:
                print(f"Predicted position variance: x_var={predicted_position_variance[0]:.2f}, y_var={predicted_position_variance[1]:.2f}")
            else:
                print("Predicted position variance: None")
            continue

        # --- Set destination ---
        elif user_input.lower() == "dest":
            dest_input = input("Enter destination as x,y in inches (e.g., 84,36): ")
            try:
                x_str, y_str = dest_input.split(",")
                destination = np.array([float(x_str.strip()), float(y_str.strip())])
                print(f"Destination set to x={destination[0]:.2f}, y={destination[1]:.2f}")
            except:
                print("Invalid input format. Please enter as x,y (e.g., 84,36).")
            continue

        # --- Plan and execute movement ---
        elif user_input.lower() == "plan":
            forward_dist, angular_dist = plan_path(predicted_position, predicted_position_variance, sensor_array, np.array([84.0, 36.0]))
            print(f"r0:{round(angular_dist,2)}, w0:{round(forward_dist,2)}")

            # Send the commands to simmer
            # In simmer a positive rotation is clockwise but we want counterclockwise to be positive
            packet_tx = packetize(f"r0:-{round(angular_dist,2)}")
            transmit(packet_tx)
            while True:
                [responses, time_rx] = receive()
                print(responses)
                if responses[0] != False and (len(responses[0]) > 1 and responses[0][1] != "False"):
                    break
            time.sleep(abs(angular_dist) / 30.0 + 0.5)  # wait for rotation to finish
            # Update particle_array based on rotation
            particle_array = Particle.move_fast_euler_step_approximation(particle_array, 0.0, angular_dist)

            packet_tx = packetize(f"w0:{round(forward_dist,2)}")
            transmit(packet_tx)
            while True:
                [responses, time_rx] = receive()
                print(responses)
                if responses[0] != False and (len(responses[0]) > 1 and responses[0][1] != "False"):
                    break
            time.sleep(abs(forward_dist) / 6.0 + 0.5)  # wait for movement to finish

            particle_array = Particle.move_fast_euler_step_approximation(particle_array, forward_dist, 0.0)
            # Remove particles that are out of bounds
            invalid_mask = check_if_contains_points_vectorized(particle_array[:, :2], WALLS_WITH_BUFFER)
            particle_array = particle_array[~invalid_mask]
            # Remove particles that are outside the outer boundary
            valid_mask = check_if_contains_points_vectorized(particle_array[:, :2], [OUTER])
            particle_array = particle_array[valid_mask]
            print("Movement command sent.")
        
        elif user_input.lower()[:2] in {"w0", "r0"}:
            # --- Send manual command to robot ---
            # If rotation command, send negative for counterclockwise positive
            if user_input.lower()[:2]  == "r0":
                # Update particles_array based on rotation
                angular_dist = float(user_input.split(":")[1])
                packet_tx = packetize(f"r0:{-round(angular_dist, 2)}")
                transmit(packet_tx)
                while True:
                    [responses, time_rx] = receive()
                    print(responses)
                    if responses[0]:
                        break
                time.sleep(abs(angular_dist) / 45.0 + 0.5)  # wait for rotation to finish
                particle_array = Particle.move_fast_euler_step_approximation(particle_array, 0.0, angular_dist)
                # Remove particles that are out of bounds
                invalid_mask = check_if_contains_points_vectorized(particle_array[:, :2], WALLS_WITH_BUFFER)
                particle_array = particle_array[~invalid_mask]
                # Remove particles that are outside the outer boundary
                valid_mask = check_if_contains_points_vectorized(particle_array[:, :2], [OUTER])
                particle_array = particle_array[valid_mask]

            elif user_input.lower()[:2]  == "w0":
                # Update particles_array based on forward movement
                try:
                    forward_dist = float(user_input.split(":")[1])
                    packet_tx = packetize(f"w0:{round(forward_dist,2)}")
                    transmit(packet_tx)
                    while True:
                        [responses, time_rx] = receive()
                        print(responses)
                        if responses[0]:
                            break
                    time.sleep(abs(forward_dist) / 6.0 + 0.5)  # wait for movement to finish
                    particle_array = Particle.move_fast_euler_step_approximation(particle_array, forward_dist, 0.0)
                    # Remove particles that are out of bounds
                    invalid_mask = check_if_contains_points_vectorized(particle_array[:, :2], WALLS_WITH_BUFFER)
                    particle_array = particle_array[~invalid_mask]
                    # Remove particles that are outside the outer boundary
                    valid_mask = check_if_contains_points_vectorized(particle_array[:, :2], [OUTER])
                    particle_array = particle_array[valid_mask]
                except:
                    print("Invalid forward command format. Use w0:distance_in_inches")
            print("Manual command sent.")

            visualize_particles(particle_array)
            # --- After movement, get new sensor readings and update localization ---
            sensor_array = get_sensor_readings()
            predicted_position, predicted_variance, weights = predict_position_and_variance(particle_array, sensor_array, position=predicted_position, variance=predicted_position_variance)
            
            print(f"Updated predicted position: x={predicted_position[0]:.2f}, y={predicted_position[1]:.2f}, theta={predicted_position[2]:.2f}")
            print(f"Updated predicted variance: x_var={predicted_variance[0]:.2f}, y_var={predicted_variance[1]:.2f}")

            # Resample particles based on weights and variance
            # IMPORTANT: perturb_particles can be customized in mcl_helpers.py to adjust how particles are resampled
            num_particles = min(max(100, int(np.mean(predicted_variance) * 100)), 2000)
            print(f"Resampling to {num_particles} particles based on predicted variance.")
            # Normalize weights
            weights /= np.sum(weights)
            indices = stratified_resample(weights.flatten())
            particle_array = particle_array[indices[:num_particles]]
            particle_array = Particle.perturb_particles(particle_array, 1 / np.mean(predicted_variance))
            # Remove particles that are out of bounds
            invalid_mask = check_if_contains_points_vectorized(particle_array[:, :2], WALLS_WITH_BUFFER)
            particle_array = particle_array[~invalid_mask]
            # Remove particles that are outside the outer boundary
            valid_mask = check_if_contains_points_vectorized(particle_array[:, :2], [OUTER])
            particle_array = particle_array[valid_mask]
            print(f"Post-resampling particle count (due to boundaries): {len(particle_array)}")
            visualize_particles(particle_array)

if __name__ == "__main__":
    main_loop()