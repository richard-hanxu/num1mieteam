import asyncio
import logging
from ble_serial.bluetooth.ble_client import BLE_client

import shapely.geometry as geom
import shapely.ops as ops
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from filterpy.monte_carlo import stratified_resample
import sys
import time

from mcl_helpers import Rover, Particle

# === Your HM-10 parameters ===
ADAPTER = "hci0"                           # Linux; ignore on Windows
DEVICE = "D4:36:39:61:CF:5B"               # MAC of your module
SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB"
WRITE_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"
READ_UUID    = "0000FFE1-0000-1000-8000-00805F9B34FB"
WRITE_WITH_RESPONSE = True

# === Global line buffer for reassembly ===
ble_buffer = b""
response_event = asyncio.Event()
latest_response = None

# === Simulation parameters === 
CLIP_SENSOR_VALUES = True
MAX_SENSOR_RANGE = 96.0  # inches

# Define outer boundary (e.g., 0 ≤ x ≤ 10, 0 ≤ y ≤ 10)
outer = geom.Polygon(
    [(3.75, 3.75), (96 - 3.75, 3.75), (96 - 3.75, 48 - 3.75), (3.75, 48 - 3.75)]
)

# Define "walls" as polygons
wall1 = geom.Polygon([(12, 24), (24, 24), (24, 36), (12, 36)])
wall2 = geom.Polygon([(24, 12), (36, 12), (36, 24), (24, 24)])
wall3 = geom.Polygon([(36, 24), (60, 24), (60, 36), (36, 36)])
wall4 = geom.Polygon([(48, 0), (60, 0), (60, 12), (48, 12)])
wall5 = geom.Polygon([(72, 0), (84, 0), (84, 12), (72, 12)])
wall6 = geom.Polygon([(72, 24), (84, 24), (84, 48), (72, 48)])
walls = [wall1, wall2, wall3, wall4, wall5, wall6]

walls_with_buffer = [wall.buffer(3.75) for wall in walls]
sampling_region = outer.difference(ops.unary_union(walls_with_buffer))

def check_if_contains_points_vectorized(points: np.ndarray, polygons: list[geom.Polygon]):
    """
    Check if each point in points is contained within the given polygon.
    points: np.ndarray of shape (N, 2) where each row is (x, y)
    polygons: list of shapely Polygons
    Returns: np.ndarray of shape (N,) with boolean values
    """

    contained = np.array([False] * len(points))
    for polygon in polygons:
        path = Path(polygon.exterior.coords)
        contained = np.logical_or(path.contains_points(points), contained)
    return contained

def sample_points(
    boundaries: geom.Polygon, sampling_region: geom.Polygon, num_samples=1000
):
    min_x, min_y, max_x, max_y = boundaries.bounds
    points = []
    while len(points) < num_samples:
        random_point = geom.Point(
            np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)
        )
        # Check if sampling_region contains the point
        if sampling_region.contains(random_point):
            random_orientation = np.random.uniform(0, 360)
            points.append((random_point.x, random_point.y, random_orientation))
    return points

def sample_starting_points():
    """
    Sample from predefined starting locations, these are locations where the robot is known to start in SimMeR
    """
    points = []
    for x in np.arange(6, 90 + 0.01, 3):
        for y in np.arange(6, 42 + 0.01, 3):
            for k in np.arange(0, 360, 30):
                point = geom.Point(x, y)
                if sampling_region.contains(point):
                    points.append((x, y, k))
    # Print points as a ndarray
    return points

def visualize_particle_array(robot, particles: np.ndarray):
    """
    Visualize the robot, particles (as an Nx3 ndarray), and environment walls.
    - robot: an object with .x, .y, .theta attributes (or None)
    - particles: np.ndarray of shape (N, 3) -> [x, y, theta]
    - walls: list of shapely Polygons (optional)
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(0, 96)
    ax.set_ylim(0, 48)
    ax.set_aspect("equal")

    # Plot all particle positions
    xs = particles[:, 0]
    ys = particles[:, 1]
    thetas = np.radians(-(particles[:, 2] - 90))

    # Scatter plot of particle positions
    ax.scatter(xs, ys, s=2, color="blue", alpha=0.6)

    # Draw short orientation lines for each particle
    dx = np.cos(thetas)
    dy = np.sin(thetas)
    for i in range(len(xs)):
        ax.plot([xs[i], xs[i] + dx[i]],
                [ys[i], ys[i] + dy[i]],
                color="blue", linewidth=0.5)
        
    # Plot a point at 35,35 pointing at 30 degrees for reference
    ax.scatter(35, 35, s=20, color="red")
    ax.plot([35, 35 + 2 * np.cos(np.radians(30))],
            [35, 35 + 2 * np.sin(np.radians(30))],
            color="green", linewidth=2)
    ax.plot([35, 35 + 2 * np.cos(np.radians(60))],
            [35, 35 + 2 * np.sin(np.radians(60))],
            color="grey", linewidth=2)

    # Plot robot pose if available
    if robot is not None:
        ax.scatter(robot.x, robot.y, s=20, color="green")
        ax.plot([robot.x, robot.x + 2 * np.cos(np.radians(-(robot.theta - 90)))],
                [robot.y, robot.y + 2 * np.sin(np.radians(-(robot.theta - 90)))],
                color="green", linewidth=2)

    for wall in walls_with_buffer:
        x_wall, y_wall = wall.exterior.xy
        ax.fill(x_wall, y_wall, color="red", alpha=0.7)

    for wall in walls:
        x_wall, y_wall = wall.exterior.xy
        ax.plot(x_wall, y_wall, color="black", linewidth=2)
    
    # Draw outline of outer boundary
    x_outer, y_outer = outer.exterior.xy
    ax.plot(x_outer, y_outer, color="black", linewidth=2)

    # Draw grid lines every 3 inches and thick lines every 12 inches
    for x in range(0, 97, 3):
        if x % 12 == 0:
            ax.axvline(x, color="gray", linewidth=1)
        else:
            ax.axvline(x, color="lightgray", linewidth=0.5)
    for y in range(0, 49, 3):
        if y % 12 == 0:
            ax.axhline(y, color="gray", linewidth=1)
        else:
            ax.axhline(y, color="lightgray", linewidth=0.5)

    ax.invert_yaxis()  # Invert y-axis to match screen coordinates

    plt.xlabel("X (inches)")
    plt.ylabel("Y (inches)")
    plt.title("Particle Filter Visualization")
    plt.tight_layout()
    plt.show(block=False)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

def visualize_particles_array_predictions(robot, particles: np.ndarray, weights: np.ndarray):
    """
    Visualize the robot, particles (as an Nx3 ndarray), and environment walls.
    - robot: an object with .x, .y, .theta attributes (or None)
    - particles: np.ndarray of shape (N, 3) -> [x, y, theta]
    - weights: np.ndarray of shape (N, 1) -> particle weights
    """

    # Normalize weights for color mapping
    weights = np.clip(weights, 0, None)  # ensure no negatives
    if np.sum(weights) > 0:
        weights_norm = weights / np.max(weights)
    else:
        weights_norm = np.zeros_like(weights)

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(0, 96)
    ax.set_ylim(0, 48)
    ax.set_aspect("equal")

    xs = particles[:, 0]
    ys = particles[:, 1]
    thetas = np.radians(-(particles[:, 2] - 90))

    # Map weights to color scale (blue → red)
    cmap = cm.get_cmap('coolwarm')
    colors = cmap(weights_norm.flatten())

    # Scatter plot of particle positions, weighted by color
    sc = ax.scatter(xs, ys, s=8, c=colors, alpha=0.7, edgecolors='none')

    # Draw short orientation lines for each particle
    dx = np.cos(thetas)
    dy = np.sin(thetas)
    for i in range(len(xs)):
        ax.plot([xs[i], xs[i] + dx[i]],
                [ys[i], ys[i] + dy[i]],
                color=colors[i],
                linewidth=0.5)

    # Plot reference point at (35, 35)
    ax.scatter(35, 35, s=20, color="red")
    ax.plot([35, 35 + 2 * np.cos(np.radians(30))],
            [35, 35 + 2 * np.sin(np.radians(30))],
            color="green", linewidth=2)
    ax.plot([35, 35 + 2 * np.cos(np.radians(60))],
            [35, 35 + 2 * np.sin(np.radians(60))],
            color="grey", linewidth=2)

    # Plot robot pose if available
    if robot is not None:
        ax.scatter(robot.x, robot.y, s=40, color="green", edgecolor='black', zorder=3)
        ax.plot([robot.x, robot.x + 2 * np.cos(np.radians(-(robot.theta - 90)))],
                [robot.y, robot.y + 2 * np.sin(np.radians(-(robot.theta - 90)))],
                color="green", linewidth=2)

    # Plot environment walls if defined globally
    try:
        for wall in walls_with_buffer:
            x_wall, y_wall = wall.exterior.xy
            ax.fill(x_wall, y_wall, color="red", alpha=0.7)

        for wall in walls:
            x_wall, y_wall = wall.exterior.xy
            ax.plot(x_wall, y_wall, color="black", linewidth=2)

        x_outer, y_outer = outer.exterior.xy
        ax.plot(x_outer, y_outer, color="black", linewidth=2)
    except NameError:
        pass  # allow function to run even if no walls are defined

    # Draw grid lines
    for x in range(0, 97, 3):
        ax.axvline(x, color="lightgray" if x % 12 else "gray", linewidth=1 if x % 12 == 0 else 0.5)
    for y in range(0, 49, 3):
        ax.axhline(y, color="lightgray" if y % 12 else "gray", linewidth=1 if y % 12 == 0 else 0.5)

    ax.invert_yaxis()
    plt.xlabel("X (inches)")
    plt.ylabel("Y (inches)")
    plt.title("Particle Predictions (Weighted Visualization)")

    # Add colorbar for weights
    cbar = plt.colorbar(sc, ax=ax, fraction=0.03, pad=0.04)
    cbar.set_label("Particle Weight (low → high)")

    plt.tight_layout()
    plt.show(block=False)


def estimate(particles_array, weights):
    """
    Estimate the robot's position based on particles and their weights.
    """
    mean = np.average(particles_array, axis=0, weights=weights)
    return mean

def particle_variance(particles_array, weights, mean_x, mean_y):
    """
    Calculate the variance of the particle positions.
    """
    var_x = np.average((particles_array[:, 0] - mean_x) ** 2, weights=weights)
    var_y = np.average((particles_array[:, 1] - mean_y) ** 2, weights=weights)
    return [var_x, var_y]

# Jitter the particles' positions and orientations to keep them close to their original pose
def perturb_particles(particle_array, certainty):
    # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
    noise_scale = max(1 - certainty, 0)
    x_noise = np.random.normal(0, 3 + 0.5 * noise_scale, size=particle_array.shape[0])
    y_noise = np.random.normal(0, 3 + 0.5 * noise_scale, size=particle_array.shape[0])
    theta_noise = np.random.normal(0, 6, size=particle_array.shape[0])

    particle_array[:, 0] += x_noise
    particle_array[:, 1] += y_noise
    particle_array[:, 2] = (particle_array[:, 2] + theta_noise) % 360

    return particle_array

def clip_readings(sensor_array):
    if CLIP_SENSOR_VALUES:
        sensor_array = np.clip(sensor_array, 0.0, MAX_SENSOR_RANGE)
    return sensor_array

valid_positions = sample_points(outer, sampling_region, num_samples=5000)
# valid_positions_2 = sample_starting_points()
# valid_positions.extend(valid_positions_2)

particles_array = np.asarray([[p[0], p[1], p[2]] for p in valid_positions])
# sensor_array = np.zeros((12,))
sensor_array = np.zeros((6,))
expected_measurements = None

# === BLE receive callback ===
def receive_callback(value: bytes):
    global ble_buffer, latest_response
    ble_buffer += value
    if b"\n" in ble_buffer:
        line, ble_buffer = ble_buffer.split(b"\n", 1)
        ble_buffer = b""
        msg = line.decode("utf-8", errors="ignore").strip()
        if msg:
            latest_response = msg
            response_event.set()


# === BLE command sender ===
async def send_and_wait(ble, message: str, delay=1.0):
    global latest_response
    latest_response = None
    response_event.clear()
    ble.queue_send((message + "\n").encode("utf-8"))
    print(f"📤 Sent: {message}")
    await asyncio.sleep(delay)
    try:
        await asyncio.wait_for(response_event.wait(), timeout=3.0)
        print(f"📥 Received: {latest_response}")
        return latest_response
    except asyncio.TimeoutError:
        print("⚠️ Timeout waiting for BLE response.")
        return None


# === Main interactive loop ===
async def interactive_particle_loop(ble):
    global particles_array, sensor_array, expected_measurements

    while True:
        user_input = input("\nEnter command (w<num>, r<num>, s, u, sample, exp, v, regenerate_all, exit): ").strip()
        if user_input.lower() == "exit":
            print("Exiting loop.")
            sys.exit(0)
            break

        # === 1️⃣ Move commands: wN or rN ===
        if user_input[0].lower() in {"w", "r"}:
            try:
                # Convert command to movement
                bt_value = user_input[1:]
                if user_input[0].lower() == "w":
                    bt_value = float(user_input[1:]) * 5/6 * 100
                    bt_value = "w" + str(round(bt_value, 2))
                    # Round bt_value to 2 decimals and convert to string
                elif user_input[0].lower() == "r":
                    bt_value = float(user_input[1:]) * 1 / 12 * 100
                    bt_value = "r" + str(round(bt_value, 2))
                    print(bt_value)
                if bt_value == 0:
                    print("Move value is 0, skipping BLE command and calculating expected measurements.")
                    # expected_measurements = Particle.calculate_expected_measurements_fast(particles_array)
                    expected_measurements = Particle.calculate_expected_measurements_fast_90_deg(particles_array)
                else:
                    response = await send_and_wait(ble, bt_value)
                    print(f"Responsetest: {response}")
                    if response:
                        # parse actual encoder feedback like "w3:2978"
                        # _, count_str = response.split(":")
                        # actual_count = float(count_str)
                        forward_vel = 0.0
                        angular_vel = 0.0

                        if user_input[0].lower() == "w":
                            forward_vel = float(user_input[1:])  # your scaling
                        elif user_input[0].lower() == "r":
                            angular_vel = float(user_input[1:]) % 360

                        particles_array = Particle.move_fast_euler_step_approximation(
                            particles_array, forward_vel, angular_vel
                        )
                        print(particles_array)
                        invalid_mask = check_if_contains_points_vectorized(particles_array[:, :2], walls_with_buffer)
                        particles_array = particles_array[~invalid_mask]
                        valid_mask = check_if_contains_points_vectorized(particles_array[:, :2], [outer])
                        particles_array = particles_array[valid_mask]
                        # visualize_particle_array(None, particles_array)
                        # print(particles_array)
                        # Update expected measurements after movement
                        # expected_measurements = Particle.calculate_expected_measurements_fast(particles_array)
                        expected_measurements = Particle.calculate_expected_measurements_fast_90_deg(particles_array)
            except Exception as e:
                print(f"⚠️  move command: {e}")

        elif user_input.lower() == "v":
            # Visualize current particles
            visualize_particle_array(None, particles_array)

        elif user_input.lower()[:3] == "exp":
            user_input = user_input.split(":")[1]
            x_pos, y_pos, r_pos = map(float, user_input.split(","))
            state = np.asarray([[x_pos, y_pos, r_pos]])
            # Particle.calculate_expected_measurements_fast(state, debug=True)
            single_expected_measurement = Particle.calculate_expected_measurements_fast_90_deg(state)
            print(f"Expected measurements at this position: {single_expected_measurement[0]}")
        # === 2️⃣ Sensor command: s ===
        elif user_input.lower() == "s":
            response = await send_and_wait(ble, user_input)
            if response and response.startswith("s:"):
                parts = response.split(":")[1].split(",")
                print(sensor_array)
                print(parts)
                for i in range(6):
                    sensor_array[i] = float(parts[i])
                if CLIP_SENSOR_VALUES:
                    sensor_array = clip_readings(sensor_array)
                print(f"Sensor readings: {sensor_array}")
            # which_indices = int(input("Enter 0 or 1"))
            # # Assign readings to every other index of sensor_array, depending on which_indices
            # response = await send_and_wait(ble, user_input)
            # if response and response.startswith("s:"):
            #     parts = response.split(":")[1].split(",")
            #     print(sensor_array)
            #     print(parts)
            #     for i in range(6):
            #         sensor_array[2 * i + which_indices] = float(parts[i])
            #     if CLIP_SENSOR_VALUES:
            #         sensor_array = clip_readings(sensor_array)
            #     print(f"Sensor readings: {sensor_array}")
                # sensor_array = sensor_values

        # === 3️⃣ Update weights: u ===
        elif user_input.lower() == "u":
            if sensor_array is not None and particles_array.shape[0] > 0:
                weights = np.ones((particles_array.shape[0], 1))
                # expected_measurements = Particle.calculate_expected_measurements_fast_12(particles_array)
                expected_measurements = Particle.calculate_expected_measurements_fast_90_deg(particles_array)
                weights = Particle.update_weights_fast(sensor_array, weights, expected_measurements)

                top_indices = np.argsort(weights[:, 0])[-100:]
                print("Top particles:")
                for idx in top_indices:
                    print(f"Particle: {particles_array[idx]}, Weight: {weights[idx,0]}")
                visualize_particle_array(None, particles_array[top_indices])
            else:
                print("No sensor data or particles available.")

        # === 4️⃣ Resample: rs ===
        elif user_input.lower() == "sample":
            if sensor_array is not None and expected_measurements is not None:
                weights = np.ones((particles_array.shape[0], 1))
                weights = Particle.update_weights_fast(sensor_array, weights, expected_measurements)
                # weights = Particle.update_weights_fast_variable_std(sensor_array, weights, expected_measurements)
                weights /= np.sum(weights)
                pred_x, pred_y, pred_theta = estimate(particles_array, weights[:, 0])
                # Print estimated position
                print(f"Estimated Position: x={pred_x:.2f}, y={pred_y:.2f}, theta={pred_theta:.2f}")
                var_x, var_y = particle_variance(particles_array, weights[:, 0], pred_x, pred_y)
                print(f"Particle Variance: var_x={var_x:.4f}, var_y={var_y:.4f}")
                visualize_particles_array_predictions(None, particles_array, weights)
                avg_var = (var_x + var_y) / 2
                num_particles = max(min(int(50 * avg_var), 3000), 500)
                print(f"Resampling {num_particles} particles.")
                indices = stratified_resample(weights[:, 0])
                print(f"Indices: {indices}")
                particles_array = particles_array[indices[:num_particles]]
                particles_array = perturb_particles(particles_array, 1 / avg_var)
                visualize_particle_array(None, particles_array)
            else:
                print("No weights or measurements available.")
        
        elif user_input.lower() == "regenerate_all":
            valid_positions = sample_points(outer, sampling_region, num_samples=3000)
            valid_positions_2 = sample_starting_points()
            valid_positions.extend(valid_positions_2)
            particles_array = np.asarray([[p[0], p[1], p[2]] for p in valid_positions])
            print("Regenerated all particles")

        else:
            print("Unknown command.")

# === Main BLE entrypoint ===
async def run_ble():
    ble = BLE_client(ADAPTER, "HM10")
    ble.set_receiver(receive_callback)
    while True:
        try:
            print(f"Connecting to {DEVICE}...")
            await ble.connect(DEVICE, "public", SERVICE_UUID, 10.0)
            await ble.setup_chars(WRITE_UUID, READ_UUID, "rw", WRITE_WITH_RESPONSE)
            print("✅ Connected to HM-10!")

            await asyncio.gather(
                ble.send_loop(),
                interactive_particle_loop(ble)
            )

        except Exception as e:
            print(f"⚠️ Connection error: {e}")
            print("Reconnecting in 2 seconds...")
            time.sleep(2)

        finally:
            await ble.disconnect()
            print("Disconnected. Retrying...")
            time.sleep(2)

# === Run script ===
if __name__ == "__main__":
    # Initialize particles

    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(run_ble())
    except KeyboardInterrupt:
        print("\nProgram stopped.")
