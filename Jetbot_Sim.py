# Inputs: List of ordered pairs (x,y) of length n , where the last item is start and first is goal.
# create a list of angles and distances from point a to b, where a is current and b is next. Iterate through and get the
# angles,lengths until a "path" from start to goal is provided. Start from the starting point, The last pair. Just inverse the list first.
import math

def read_points_from_file(filename):
    points = []
    with open(filename, 'r') as file:
        for line in file:
            # Remove whitespace and parentheses, then split by comma
            stripped_line = line.strip().replace('(', '').replace(')', '')
            point = tuple(map(float, stripped_line.split(', ')))
            points.append(point)

    points.reverse()    #Points will be in wrong order by default
    return points

# Gives the angle relative to the point with Atan2
def calculate_angle_and_distance(points): 
    angles_and_distances = []

    for i in range(len(points) - 1):
        point_a = points[i]
        point_b = points[i + 1]

        # Calculate the distance between point A and B
        distance = math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1] - point_a[1])**2)

        # Calculate the angle
        angle = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])

        # Convert angle to degrees
        angle_degrees = math.degrees(angle)

        angles_and_distances.append((angle_degrees, distance))

    return angles_and_distances

def calculate_vectors(points):
    vectors = []

    for i in range(len(points) - 1):
        point_a = points[i]
        point_b = points[i + 1]

        # Create a vector from point A to B
        vector = (point_b[0] - point_a[0], point_b[1] - point_a[1])

        vectors.append(vector)

    return vectors

# This one resets robot orientation to 0,0 upon reaching each point. Resets coordinate fram, gives
# angles jetbot needs to acutally turn.
def calculate_robot_path(points):
    angles_and_distances = []
    current_angle = 0

    for i in range(len(points) - 1):
        point_a = points[i]
        point_b = points[i + 1]

        # Check and convert point_a and point_b to tuples if they are NumPy arrays
        if isinstance(point_a, np.ndarray):
            point_a = tuple(point_a)
        if isinstance(point_b, np.ndarray):
            point_b = tuple(point_b)

        # Calculate the angle to the next point
        next_angle = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        relative_angle = next_angle - current_angle

        # Convert relative angle to degrees and ensure it is between -180 and 180 degrees
        relative_angle_degrees = math.degrees(relative_angle) % 360
        if relative_angle_degrees > 180:
            relative_angle_degrees -= 360

        # Calculate the distance to the next point
        distance = math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1] - point_a[1])**2)

        angles_and_distances.append((relative_angle_degrees, distance))

        current_angle = next_angle

    return angles_and_distances
if __name__ == "__main__":
    points = read_points_from_file("sample_output.txt")
    print("x,y")
    print(points)
    print("----------------------------------------------------")

    path = calculate_angle_and_distance(points)
    print("(Degrees, distance):")
    print(path)
    print("----------------------------------------------------")

    vectors = calculate_vectors(points)
    print("vectors:")
    print(vectors)
    print("----------------------------------------------------")

    robot_angles = calculate_robot_path(points)
    print("robot angles:")
    print(robot_angles)
    print("----------------------------------------------------")

    # print(len(points))
    # print(len(path))

