'''Methods for landmark extraction from range scans.'''
from math import radians, sin, cos, fsum, inf, sqrt, atan, pi, tau, isclose
from random import choice, sample, seed

def normalize_angle(angle):
    '''Normalize the provided angle to [0, tau).'''
    ret_angle = angle
    while ret_angle < 0:
        ret_angle += tau
    while ret_angle >= tau:
        ret_angle -= tau
    return ret_angle

def normalized_angle_difference(first, second):
    '''Computes the normalized difference between two angles, result in [-pi, pi].'''
    first_norm = normalize_angle(first)
    second_norm = normalize_angle(second)
    diff = second - first
    return min(diff, diff + tau, diff - tau, key = abs)

def linear_regression(points):
    '''Calculates ax + by + c = 0 from the given points.'''

    n = len(points)
    sx = fsum([p[0] for p in points])
    sy = fsum([p[1] for p in points])
    sxx = fsum([p[0] ** 2 for p in points])
    sxy = fsum([p[0] * p[1] for p in points])
    syy = fsum([p[1] ** 2 for p in points])

    try:
        a = ((n * sxy) - (sx * sy)) / ((n * sxx) - (sx ** 2))
    except ZeroDivisionError:
        # If the line is vertical, y is not considered.
        a = -1
        b = 0
        c = sx / n
    else:
        b = -1
        c = (sy / n) - ((a * sx) / n)

    return (a, b, c)

def min_distance(line, point):
    '''Calculates the minimum distance between a line and a point.'''

    a, b, c = line

    # Extracting coordinates this way allows extra elements to be ignored.
    x = point[0]
    y = point[1]

    return abs((a * x) + (b * y) + c) / sqrt((a ** 2) + (b ** 2))

def closest_point(line, point):
    '''Calculates the point on a line closest to another point.'''

    a, b, c = line

    # Extracting coordinates this way allows extra elements to be ignored.
    x = point[0]
    y = point[1]

    line_x = ((b * ((b * x) - (a * y))) - (a * c)) / ((a ** 2) + (b ** 2))
    line_y = ((a * ((a * y) - (b * x))) - (b * c)) / ((a ** 2) + (b ** 2))

    return (line_x, line_y)

def extract_spike(pos, raw_data, spike_threshold=0.5, **kwargs):
    '''Observe environment and extract landmarks using the spike technique.
       raw_data is a list of consecutive, evenly spaced, (d_theta, distance) pairs.'''

    wrap_around = len(raw_data) > 1 and isclose(
        normalized_angle_difference(raw_data[0][0], raw_data[1][0]),
        normalized_angle_difference(raw_data[-1][0], raw_data[0][0]))

    landmarks = []
    x, y, theta = pos

    for i, (d_theta, B) in enumerate(raw_data):
        if not wrap_around and (i == 0 or i == (len(raw_data) - 1)):
            continue

        A = raw_data[(i - 1) % len(raw_data)][1]
        C = raw_data[(i + 1) % len(raw_data)][1]

        if A < 0 or B < 0 or C < 0:
            # Just skip points with negative ranges.
            continue

        if ((A - B) + (C - B)) >= spike_threshold:
            mark_x = x + (B * cos(theta + d_theta))
            mark_y = y + (B * sin(theta + d_theta))

            landmarks.append((mark_x, mark_y))

    return landmarks

def extract_ransac(pos, raw_data, seed_override=None,
                   ransac_max_tries=100, ransac_samples=5, ransac_range=radians(20),
                   ransac_error=0.1, ransac_consensus=30, **kwargs):
    '''Observe environment and extract landmarks using the RANSAC technique.
       raw_data is a list of (d_theta, distance) pairs.'''
    assert(ransac_range > 0 and ransac_range <= pi)

    # Allow setting the random seed for predictable testing.
    if seed_override is not None:
        seed(seed_override)

    landmarks = []
    x, y, theta = pos

    # Process the data into points and angles.
    data = [(x + (distance * cos(theta + angle)),
             y + (distance * sin(theta + angle)),
             normalize_angle(angle)) for angle, distance in raw_data]

    associated = set()
    for i in range(ransac_max_tries):
        if len(data) - len(associated) < ransac_consensus:
            # If there are not enough points to reach consensus, then quit.
            break

        # Select a base point for the fit.
        point = choice([p for p in data if not p in associated])

        # Define the start and end for the considered range.
        start = normalize_angle(point[2] - ransac_range)
        end = normalize_angle(point[2] + ransac_range)

        # Generate a list of unclaimed points within the range.
        if start < end:
            possible_points = [p for p in data if not p in associated
                               and p != point and (p[2] >= start and p[2] <= end)]
        else:
            possible_points = [p for p in data if not p in associated
                               and p != point and (p[2] >= end or p[2] <= start)]

        if len(possible_points) < (ransac_samples - 1):
            # If there are not enough points for the fit, then restart.
            continue

        # Select configured number of points, always including the base point.
        points = [point] + sample(possible_points, ransac_samples - 1)

        # Generate best fit line.
        line = linear_regression(points)
        # TODO(Daniel): transition to using 'total least squares'

        # Restart if line doesn't meet consensus.
        supporters = []
        for point in data:
            if not point in associated and min_distance(line, point) < ransac_error:
                supporters.append(point)
        if len(supporters) < ransac_consensus:
            continue

        # Associate all points that are close enough.
        for point in supporters:
            associated.add(point)

        # Regenerate best fit using all points that are close enough.
        line = linear_regression(supporters)

        # Calculate landmark representation and add to landmarks list.
        landmarks.append(closest_point(line, (0,0)))

    return landmarks
