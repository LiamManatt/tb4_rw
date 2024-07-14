import numpy as np

def calculate_slope(data):
    slopes = []
    for i in range(1, len(data)):
        x_diff = data[i][0] - data[i - 1][0]
        y_diff = data[i][1] - data[i - 1][1]
        slope = y_diff / x_diff
        slopes.append(slope)
    return slopes

def create_BezierCurve(path):
    # sort x
    bezier_points = np.array(path)
    sorted_indices = np.argsort(bezier_points[:, 0])
    points_sorted = bezier_points[sorted_indices]

    # Bezier interpolation
    degree = len(bezier_points)/5.  # set the order of Bazier
    coefficients = np.polyfit(points_sorted[:, 0], points_sorted[:, 1], degree)

    # smooth the path
    x_smooth = np.linspace(min(points_sorted[:, 0]), max(points_sorted[:, 0]), 500)
    y_smooth = np.polyval(coefficients, x_smooth)
    # plt.plot(points_sorted[:, 0], points_sorted[:, 1], 'b', label='Original Data')
    # plt.plot(x_smooth, y_smooth, 'g--', label='Bezier Curve')
    # plt.plot(start[0], start[1], 'ro')
    # plt.plot(goal[0], goal[1], 'bo')
    # plt.grid(True)

    # reverse x_smooth and y_smooth
    x_smooth_reversed = x_smooth[::-1]
    y_smooth_reversed = y_smooth[::-1]
    # making new list that contain both data
    smooth_data = list(zip(x_smooth_reversed, y_smooth_reversed))
    smooth_data = [[x, y] for x, y in smooth_data]
    smooth_slope = calculate_slope(smooth_data)   # the slope is calculated from right to left
    # smooth_theta = np.arctan(smooth_slope)+np.pi
    return smooth_data, np.arctan(smooth_slope)+np.pi, x_smooth_reversed, y_smooth_reversed

