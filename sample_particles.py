import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import shapely.geometry as geom
import shapely.ops as ops
import time

from mcl_helpers import Particle
from tcp_helpers import packetize, transmit, receive, response_string

# === Simulation parameters === 
CLIP_SENSOR_VALUES = True
MAX_SENSOR_RANGE = 96.0  # inches

# Define outer boundary (e.g., 0 ≤ x ≤ 10, 0 ≤ y ≤ 10)
OUTER = geom.Polygon(
    [(3.75, 3.75), (96 - 3.75, 3.75), (96 - 3.75, 48 - 3.75), (3.75, 48 - 3.75)]
)

# Define "walls" as polygons
wall1 = geom.Polygon([(12, 24), (24, 24), (24, 36), (12, 36)])
wall2 = geom.Polygon([(24, 12), (36, 12), (36, 24), (24, 24)])
wall3 = geom.Polygon([(36, 24), (60, 24), (60, 36), (36, 36)])
wall4 = geom.Polygon([(48, 0), (60, 0), (60, 12), (48, 12)])
wall5 = geom.Polygon([(72, 0), (84, 0), (84, 12), (72, 12)])
wall6 = geom.Polygon([(72, 24), (84, 24), (84, 48), (72, 48)])
WALLS = [wall1, wall2, wall3, wall4, wall5, wall6]
WALLS_WITH_BUFFER = [wall.buffer(3.75) for wall in WALLS]
SAMPLING_REGION = OUTER.difference(ops.unary_union(WALLS_WITH_BUFFER))


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
    Samples from predefined starting locations, which are the centres of the 12x12 in. squares in the maze, with 
    30 degree increments in heading.
    Returns a list of (x, y, heading) tuples representing valid starting positions.
    """
    points = []
    for x in np.arange(6, 90 + 0.01, 3):
        for y in np.arange(6, 42 + 0.01, 3):
            for k in np.arange(0, 360, 30):
                point = geom.Point(x, y)
                if SAMPLING_REGION.contains(point):
                    points.append((x, y, k))
    return points

def visualize_particles(particles_array: np.ndarray):
    """
    visualize_particles(particles_array)
    Visualizes the given particles in a Matplotlib figure, overlaying them on the maze
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    # Fill the entire screen with blue
    ax.fill([0, 96, 96, 0], [0, 0, 48, 48], color="blue", alpha=0.3)
    # Then draw the valid sampling region in white
    x_outer, y_outer = OUTER.exterior.xy
    ax.fill(x_outer, y_outer, color="white")
    # Plot points on a grid
    ax.set_xlim(0, 96)
    ax.set_ylim(0, 48)
    ax.set_aspect("equal")
    for i in range(particles_array.shape[0]):
        ax.scatter(particles_array[i, 0], particles_array[i, 1], s=2)
        ax.plot(
            [
                particles_array[i, 0],
                particles_array[i, 0] + 1 * np.cos(np.radians(-(particles_array[i, 2] - 90))),
            ],
            [
                particles_array[i, 1],
                particles_array[i, 1] + 1 * np.sin(np.radians(-(particles_array[i, 2] - 90))),
            ],
            color="blue",
            linewidth=0.5,
        )
    # Draw regions where particles cannot exist in blue
    for wall in WALLS_WITH_BUFFER:
        x_wall, y_wall = wall.exterior.xy
        ax.fill(x_wall, y_wall, color="blue", alpha=0.3)
    # Plot walls
    for wall in WALLS:
        x_wall, y_wall = wall.exterior.xy
        ax.fill(x_wall, y_wall, color="red")
    # Draw grid lines spaced every 3 inches, and thicker lines every 12 inches
    for x in range(0, 97, 3):
        linewidth = 1.5 if x % 12 == 0 else 0.5
        ax.axvline(x, color="gray", linestyle="--", linewidth=linewidth, alpha=0.5)
    for y in range(0, 49, 3):
        linewidth = 1.5 if y % 12 == 0 else 0.5
        ax.axhline(y, color="gray", linestyle="--", linewidth=linewidth, alpha=0.5)

    ax.invert_yaxis()  # Invert y-axis to match screen coordinates
    plt.show(block=True)

def test_sampling(num_points=1000):
    """
    test_sampling(num_points)
    Test helper that samples valid particle positions, measures sampling performance,
    and visualizes a forward motion test for the sampled particles.
    This function performs the following steps:
    - Calls sample_points(...) to generate a collection of valid (x, y, heading)
        sample positions within the configured sampling_region (and constrained by
        the provided outer geometry / obstacles).
    - Measures and prints the elapsed time required to generate the samples.
    - Produces a Matplotlib visualization that overlays the original and moved
        particle positions and orientations, and fills wall geometries for context.
        The plot is shown in blocking mode.
    Side effects:
    - Prints a timing summary to stdout.
    - Displays a Matplotlib figure (blocking).
    - Depends on several globals and modules being available in the calling scope:
        outer, sampling_region, walls, sample_points, Particle, np, plt.
    Returns:
    - None
    Notes:
    - The number of samples is controlled via the num_samples argument passed to
        sample_points; adjust there to change test density.
    - The plot uses small line segments to indicate particle headings and limits
        the axes to the expected domain (0..96, 0..48) in the original code.
    - If required globals or geometry objects are missing or malformed, the
        function will raise exceptions.
    """
    start_time = time.time()
    valid_positions = sample_points(OUTER, SAMPLING_REGION, num_samples=num_points)
    end_time = time.time()
    print(
        f"Sampled {len(valid_positions)} valid positions in {end_time - start_time:.2f} seconds."
    )
    particles_array = np.asarray([[p[0], p[1], p[2]] for p in valid_positions])
    visualize_particles(particles_array)

if __name__ == "__main__":
    test_sampling()