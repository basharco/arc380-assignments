import numpy as np
import compas.geometry as cg
import compas_rrc as rrc

# Define any additional imports here if needed

from copy import deepcopy
import math

### CONSTANTS ###

TWO_PI = 2 * math.pi

# COORDINATES
POINT = [132.41, 451.07, 26.12]
POINT_XAXIS = [-89.87, 451.06, 27.10]
POINT_XYPLANE = [29.89, 295.55, 25]


def create_frame_from_points(point1: cg.Point, point2: cg.Point, point3: cg.Point) -> cg.Frame:
    """Create a frame from three points.

    Args:
        point1 (cg.Point): The first point (origin).
        point2 (cg.Point): The second point (along x axis).
        point3 (cg.Point): The third point (within xy-plane).

    Returns:
        cg.Frame: The frame that fits the given 3 points.
    """
    frame = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.c.

    frame = cg.Frame.from_points(point1, point2, point3)

    # ====================================================================================
    return frame


def transform_task_to_world_frame(ee_frame_t: cg.Frame, task_frame: cg.Frame) -> cg.Frame:
    """Transform a task frame to the world frame.

    Args:
        ee_frame_t (cg.Frame): The end-effector frame defined in task space.
        task_frame (cg.Frame): The task frame.

    Returns:
        cg.Frame: The task frame in the world frame.
    """
    ee_frame_w = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.d.

    T = cg.Transformation.from_frame_to_frame(cg.Frame.worldXY(), task_frame)
    ee_frame_w = deepcopy(ee_frame_t)

    ee_frame_w.transform(T)

    # ====================================================================================
    return ee_frame_w


# ====================== Drawing effects and helper functions ============================

# Part 2

def lift(abb: rrc.AbbClient, dist: float = 10, speed: float = 30) -> None:
    """Lift the arm above its current position (i.e., move it in the negative z direction) by 'dist' mm.

    Args:
        abb (rrc.AbbClient): The ABB client.
        dist (float): The distance to lift the arm by in mm. Defaults to 10.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    curr_frame = abb.send_and_wait(rrc.GetFrame())

    curr_frame.point[2] += dist

    _ = abb.send_and_wait(rrc.MoveToFrame(curr_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))


def calculate_rectangle_corners(pos: list[float], length: float, height: float, degrees: float = 0) -> list[list[float]]:
    """Calculate the coordinates of a rectangle's corners.

    Args:
        pos (list[float]): The 2-dimensional coordinates of the rectangle's center.
        length (float): The rectangle's length in mm.
        height (float): The rectangle's width in mm.
        degrees (float): The rectangle's counterclockwise rotation about its center in degrees.

    Returns:
        list[list[float]]: A list of absolute 2-dimensional coordinates representing each corner of the rectangle in the
            task space in counterclosckwise order.
    """
    half_length = length / 2
    half_height = height / 2

    min_bb_x = pos[0] - half_length
    min_bb_y = pos[1] - half_height
    max_bb_x = pos[0] + half_length
    max_bb_y = pos[1] + half_height

    point1 = [min_bb_x, min_bb_y, 0]
    point2 = [max_bb_x, min_bb_y, 0]
    point3 = [max_bb_x, max_bb_y, 0]
    point4 = [min_bb_x, max_bb_y, 0]

    rads = math.radians(degrees)
    
    points = cg.rotate_points_xy([point1, point2, point3, point4], rads, [pos[0], pos[1], 0])

    return [point[:2] for point in points]


def calculate_polygon_vertices(pos: list[float], num_sides: int, radius: float) -> list[list[float]]:
    """Calculate the coordinates of a regular polygon's vertices.

    Args:
        pos (list[float]): The 2-dimensional coordinates of the polygon's center.
        num_sides (float): The number of sides of the polygon.
        radius (float) The distance in mm from the polygon's center to any one of its vertices.

    Returns:
        list[list[float]]: A list of absolute 2D coordinates representing each vertex of the polygon in the task
            space in counterclosckwise order.
    """
    vertices = []

    for i in range(num_sides):
        x = pos[0] + radius * math.cos(TWO_PI * i / num_sides)
        y = pos[1] + radius * math.sin(TWO_PI * i / num_sides)

        vertices.append([x, y])

    return vertices

def go_to_point(abb: rrc.AbbClient, origin: cg.Frame, vertex: list[float], speed: float = 30, zone = rrc.Zone.FINE) -> None:
    """Go to a point.

    Args:
        abb (rrc.AbbClient): The ABB client.
        origin (cg.Frame): A Frame defining the task space's origin.
        vertex (list[float]): The target (2D or 3D) coordinate.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    frame = deepcopy(origin)

    for i in range(len(vertex)):
        if i == 2:
            frame.point[i] += vertex[i]
        else:
            frame.point[i] -= vertex[i]

    _ = abb.send_and_wait(rrc.MoveToFrame(frame, speed, zone, rrc.Motion.LINEAR))


def draw_shape_from_vertices(abb: rrc.AbbClient, origin: cg.Frame, vertices: list[list[float]], speed: float = 30) -> None:
    """Draw a closed shape specified by the coordinates of its vertices.

    Args:
        abb (rrc.AbbClient): The ABB client. 
        origin (cg.Frame): A Frame defining the task space's origin.
        coords (list[list[float]]): A List of 2D coordinates defining the desired shape's vertices in counterclockwise order.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    for vertex in vertices:
        go_to_point(abb, origin, vertex, speed)

    go_to_point(abb, origin, vertices[0], speed)
    lift(abb=abb, speed=speed)


def draw_lines_from_vertices(abb: rrc.AbbClient, origin: cg.Frame, strokes: list[list[list[float]]], speed: float = 30) -> None:
    """Draw strokes, each connected by a series of vertices (2-dimensional points). Lift the pen between each separate
        series of vertices (stroke)

    Args:
        abb (rrc.AbbClient): The ABB client.
        origin (cg.Frame): A frame defining the task space's origin.
        strokes (list[list[list[float]]]): a list of coordinate groupings outlining the strokes to draw.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """    
    for stroke in strokes:
        for vertex in stroke:
            go_to_point(abb, origin, vertex, speed)

        lift(abb, speed)

    lift(abb=abb, speed=speed)

def draw_line_with_changing_stroke(abb: rrc.AbbClient, origin: cg.Frame, endpoints: list[list[float]], speed: float = 30) -> None:
    """Draw a line with changing stroke thickness
    
    Args:
        abb (rrc.AbbClient): The ABB client.
        origin (cg.Frame): A frame defining the task space's origin.
        endpoints (list[list[points]]): A list of the line's two 2-dimensional endpoints.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    endpoint1 = endpoints[0]
    endpoint2 = endpoints[1]

    endpoint1.append(-3)
    endpoint2.append(0) # Should this be 2?

    go_to_point(abb, origin, endpoint1, speed)
    go_to_point(abb, origin, endpoint2, speed)

    lift(abb=abb, speed=speed)


def draw_dashed_line(abb: rrc.AbbClient, origin: cg.Frame, endpoints: list[list[float]], ratio: float, num_segments: int = 10, speed: float = 30) -> None:
    """Draw a dashed line.
    
    Args:
        abb (rrc.AbbClient): The ABB Client.
        origin (cg.Frame): A frame defining the task space's origin.
        endpoints (list[list[float]]): A list of the line's 2-dimensional endpoints.
        ratio (float): The dash/(dash + gap) ratio of the dashed line.
        num_segments (int): The number of dashed segments to include in the line. Defaults to 10.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    endpoint1 = endpoints[0]
    endpoint2 = endpoints[1]

    endpoint2[0] *= -1
    endpoint2[1] *= -1

    go_to_point(abb, origin, endpoint1, speed)

    dist_x = endpoint1[0] - endpoint2[0]
    dist_y = endpoint1[1] - endpoint2[1]

    dash_x = ratio * dist_x / num_segments
    dash_y = ratio * dist_y / num_segments

    gap_x = (1 - ratio) * dist_x / num_segments
    gap_y = (1 - ratio) * dist_y / num_segments

    for i in range(num_segments):
        point1 = [endpoint1[0] + (i + 1) * dash_x + i * gap_x, endpoint1[1] + (i + 1) * dash_y + i * gap_y]
        point2 = [endpoint1[0] + (i + 1) * (dash_x + gap_x), endpoint1[1] + (i + 1) * (dash_y + gap_y)]

        go_to_point(abb, origin, point1, speed)
        lift(abb, speed=speed)
        go_to_point(abb, origin, point2, speed)

    lift(abb=abb, speed=speed)

def draw_bezier_curve(abb: rrc.AbbClient, origin: cg.Frame, control_points: list[list[float]], resolution: float = 100, speed: float = 30) -> None:
    """Draw a Bezier curved line defined by a series of control points.
    
    Args:
        abb (rrc.AbbClient): The ABB client.
        origin (cg.Frame): A frame defining the task space's origin.
        control_points (list[list[float]]): A list of 2-dimensional coordinates to represents the curve's control
            points.
        resolution (float): The number of points along the curve to use in dictating the robot's path. Defaults to 100.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    points = []

    for control_point in control_points:
        points.append(cg.Point(control_point[0], control_point[1]))

    curve = cg.Bezier(points)
    locus = curve.locus(resolution)

    for point in locus:
        go_to_point(abb, origin, [point.x, point.y], speed)

    lift(abb=abb, speed=speed)

def draw_circle(abb: rrc.AbbClient, origin: cg.Frame, center: list[float], radius: float, speed: float = 30) -> None:
    curr_angle = 0
    while curr_angle <= TWO_PI:
        curr_point = [center[0] + (radius*math.cos(curr_angle)), center[1] + (radius*math.sin(curr_angle))]
        go_to_point(abb, origin, curr_point, zone = rrc.Zone.Z10)
        curr_angle += math.pi/36 

    lift(abb=abb, speed=speed)

def go_home(abb: rrc.AbbClient, speed: float = 30) -> None:
    """Send the robot home.
    
    Args
        abb (rrc.AbbClient): The ABB client.
        speed (float): The speed at which the arm should move in mm/s. Defaults to 30.
    """
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    _ = abb.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

# ========================================================================================


if __name__ == '__main__':
    
    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # ================================== YOUR CODE HERE ==================================

    # Parts 1.e. and 2

    ### SETUP ###
    abb_rrc.send(rrc.SetTool('group2_pen'))

    abb_speed = 30 # [mm/s]

    task_space_point1 = cg.Point(POINT[0], POINT[1], POINT[2])
    task_space_point2 = cg.Point(POINT_XAXIS[0], POINT_XAXIS[1], POINT_XAXIS[2])
    task_space_point3 = cg.Point(POINT_XYPLANE[0], POINT_XYPLANE[1], POINT_XYPLANE[2])

    ee_point1 = cg.Point(0, 0, 0)
    ee_point2 = cg.Point(1, 0, 0)
    ee_point3 = cg.Point(0, -1, 0)

    task_frame = create_frame_from_points(task_space_point1, task_space_point2, task_space_point3)
    ee_frame = create_frame_from_points(ee_point1, ee_point2, ee_point3)

    paper = transform_task_to_world_frame(ee_frame, task_frame)

    ### RECTANGLE ###
    corners = calculate_rectangle_corners(pos=[150, 75], length=50, height=30, degrees=45)
    draw_shape_from_vertices(abb=abb_rrc, origin=paper, vertices=corners)

    ### POLYGON ###
    vertices = calculate_polygon_vertices(pos=[150, 75], num_sides=6, radius=40)
    draw_shape_from_vertices(abb=abb_rrc, origin=paper, vertices=vertices)

    ### CIRCLE ###
    draw_circle(abb=abb_rrc, origin=paper, center=[50, 50], radius=15)

    ### LINE WITH CHANGING THICKNESS ###
    draw_line_with_changing_stroke(abb=abb_rrc, origin=paper, endpoints=[[85, 145], [160, 145]])

    ### DASHED LINE ###
    draw_dashed_line(abb=abb_rrc, origin=paper, endpoints=[[10, 10], [200, 10]], ratio=0.5)

    ### INITIALS ###
    stroke_p = [(0,0), (0,50), (25,50), (25,25), (0,25)]
    stroke_k = [(30,0), (30,50), (30,25), (55,0), (30,25), (55,50)]
    stroke_n = [(65,0), (65,50), (90,0), (90,50)]
    stroke_e = [(120,50), (95,50), (95,25), (120, 25), (95,25), (95,0), (120,0)]
    stroke_b = [(130,0), (130,50), (155, 50), (155,0), (130,0), (130,25), (155,25)]
    stroke_e2 = [(185,50), (160,50), (160,25), (185,25), (160,25), (160,0), (185,0)]

    strokes = [stroke_p, stroke_k, stroke_n, stroke_e, stroke_b, stroke_e2]

    draw_lines_from_vertices(abb=abb_rrc, origin=paper, strokes=strokes)

    ### LOGO ###
    stroke_p = [[0, 0], [0, 50], [25, 50], [25, 25], [0, 25]]
    stroke_n = [[30, 0], [30, 50], [55, 0], [55, 50]]
    stroke_b = [[60, 0], [60, 50], [85, 50], [85, 0], [60, 0], [60, 25], [85, 25]]

    strokes = [stroke_p, stroke_n, stroke_b]

    draw_lines_from_vertices(abb=abb_rrc, origin=paper, strokes=strokes)

    corners = calculate_rectangle_corners(pos=[42.5, 25], length=107, height=80)
    lift(abb=abb_rrc, dist=100)
    draw_shape_from_vertices(abb=abb_rrc, origin=paper, vertices=corners)
    
    vertices = calculate_polygon_vertices(pos=[42.5, 25], num_sides=8, radius=70)
    lift(abb=abb_rrc, dist=125)
    draw_shape_from_vertices(abb=abb_rrc, origin=paper, vertices=vertices)

    draw_circle(abb=abb_rrc, origin=paper, center=[42.5, 25], radius=70)

    lift(abb=abb_rrc, dist=100)
    draw_dashed_line(abb=abb_rrc, origin=paper, endpoints=[[0, -7.5], [85, 9]], ratio=0.5)

    lift(abb=abb_rrc, dist=100)
    draw_line_with_changing_stroke(abb=abb_rrc, origin=paper, endpoints=[[0, 55], [42.5, 60]])

    lift(abb=abb_rrc, dist=100)
    draw_line_with_changing_stroke(abb=abb_rrc, origin=paper, endpoints=[[85, 55], [42.5, 60]])

    ### HOME ###
    go_home(ros, abb_rrc)

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
