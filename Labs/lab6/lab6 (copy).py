import time
import json
import rospy
import copy
import math
import random
import argparse
from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
# g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_X = 1.8 # 2m wide
# g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_SIZE_Y = 1.2 # 1.5m tall
# g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_X = 0.05625 # Each col represents 6.25cm
# g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_MAP_RESOLUTION_Y = 0.0375 # Each row represents 4.6875cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,1)
g_src_coordinates = (0,0)

# Lab 4 Globals
pose2d_sparki_odometry = None # Pose2D, contains x, y, theta
servo_angle = 0
IR_sensors = None

# Waypoints
g_waypoints = []

# My globals
g_dest_x = None
g_dest_y = None
g_FIXING_BEARING_ERROR = 0 # THESE BOIS ARE CONSTANT
g_FIXING_DISTANCE_ERROR = 1 # THESE BOIS ARE CONSTANT
g_error_state = g_FIXING_BEARING_ERROR


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array)
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells)
    new_map[random_cell] = 1

  return new_map


def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

  return grid


def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS


def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y


def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  global g_WORLD_MAP

  # Source is out of bounds
  if vertex_source < 0 or vertex_source >= g_NUM_Y_CELLS * g_NUM_X_CELLS:
    return 1000

  # Destination is out of bounds
  if vertex_dest < 0 or vertex_dest >= g_NUM_Y_CELLS * g_NUM_X_CELLS:
    return 1000

  # Source and destination are unoccupied
  if g_WORLD_MAP[vertex_source] != 1 and g_WORLD_MAP[vertex_dest] != 1:
    # Adjacent
    source_i, source_j = vertex_index_to_ij(vertex_source)
    dest_i, dest_j =  vertex_index_to_ij(vertex_dest)
    if (source_i == dest_i and abs(source_j - dest_j) == 1) or (source_j == dest_j and abs(source_i - dest_i) == 1):
      return 1
    else:
      return 1000
  else:
    return 1000


def run_dijkstra(source_vertex):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run

  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  dist = [1000] * g_NUM_X_CELLS * g_NUM_Y_CELLS
  dist[source_vertex] = 0

  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  Q_cost = [(source_vertex, 0)]

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS

  # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!
  while Q_cost:
    u, c = Q_cost.pop(0)
    neighbors = [u - 1, u + 1, u - g_NUM_X_CELLS, u + g_NUM_X_CELLS]
    for v in neighbors:
      if v < g_NUM_Y_CELLS*g_NUM_X_CELLS and v >= 0 and v < g_NUM_Y_CELLS*g_NUM_X_CELLS and v >= 0:
        alt = dist[u] + get_travel_cost(u, v)
        if alt < dist[v]:
          dist[v] = alt
          prev[v] = u
          Q_cost.append((v, alt))
          Q_cost = sorted(Q_cost, key=lambda u: u[1])

  # Return results of algorithm run
  return prev


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = []

  # TODO: Insert your code here
  final_path.append(dest_vertex)
  curr_vertex = dest_vertex

  while curr_vertex != source_vertex:
    if prev[curr_vertex] == -1:
      print('There is probably not a path')
      return []
    final_path.insert(0, prev[curr_vertex])
    curr_vertex = prev[curr_vertex]
  return final_path


def render_map(map_array):
  '''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  map_string = ""
  for j in range(g_NUM_Y_CELLS):
    for i in range(g_NUM_X_CELLS):
      if map_array[g_NUM_Y_CELLS * (g_NUM_Y_CELLS-1-j) + i] == 0:
        map_string += " . "
      else:
        map_string += "[ ]"
    map_string += "\n"
  print(map_string)
  pass


def part_1():
  global g_WORLD_MAP

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
  g_WORLD_MAP = create_test_map(g_WORLD_MAP)
  # g_WORLD_MAP = [0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0]

  # Use render_map to render your initialized obstacle map
  render_map(g_WORLD_MAP)

  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
  prev = run_dijkstra(ij_to_vertex_index(*g_src_coordinates))
  for i in range(g_NUM_Y_CELLS):
    print(prev[(g_NUM_Y_CELLS-1-i)*g_NUM_X_CELLS:(g_NUM_Y_CELLS-1-i)*g_NUM_X_CELLS+4])
  print("")

  final_path = reconstruct_path(prev, ij_to_vertex_index(*g_src_coordinates), ij_to_vertex_index(*g_dest_coordinates))

  '''
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
  '''
  print("Source: " + str(g_src_coordinates))
  print("Goal: " + str(g_dest_coordinates))
  print("Path: " + " -> ".join([str(p) for p in final_path]))


def _draw_path_on_image(path, image_filename):
  '''
  Path is a list of vertices
  image_filename is the image to be drawn on
  '''
  if image_filename is None:
    raise Exception("Image file not found.")

  img = Image.open(image_filename)

  xys = []
  for p in path:
    x, y = ij_coordinates_to_xy_coordinates(*vertex_index_to_ij(p))
    xys.append((x / 0.0015, (1.2 - y) / 0.0015))

  draw = ImageDraw.Draw(img)
  draw.line(xys, fill="red", width=10)

  img.save("image1.png", "PNG")
  img.show()

  return


def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP
  global g_waypoints

  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)

  g_src_coordinates = (float(args.src_coordinates[0]), float(args.src_coordinates[1]))
  g_dest_coordinates = (float(args.dest_coordinates[0]), float(args.dest_coordinates[1]))

  '''
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''
  pixel_height = len(pixel_grid)
  pixel_width = len(pixel_grid[0])
  print(pixel_height, 1.2 / pixel_height)
  print(pixel_width, 1.8 / pixel_width)
  for y in range(pixel_height):
    for x in range(pixel_width):
      if pixel_grid[pixel_height-1-y, x] == 255.0:
        i, j = xy_coordinates_to_ij_coordinates(0.0015 * x, 0.0015 * y)
        vertex_index = ij_to_vertex_index(i, j)
        g_WORLD_MAP[vertex_index] = 1

  render_map(g_WORLD_MAP)

  prev = run_dijkstra(ij_to_vertex_index(*xy_coordinates_to_ij_coordinates(*g_src_coordinates)))

  path = reconstruct_path(prev, ij_to_vertex_index(*xy_coordinates_to_ij_coordinates(*g_src_coordinates)), ij_to_vertex_index(*xy_coordinates_to_ij_coordinates(*g_dest_coordinates)))

  # _draw_path_on_image(path, args.obstacles)

  print("Source: " + str(g_src_coordinates))
  print("Goal: " + str(g_dest_coordinates))
  print("Path: " + " -> ".join([str(p) for p in path]))

  for p in path:
    g_waypoints.append(ij_coordinates_to_xy_coordinates(*vertex_index_to_ij(p)))

  print(g_waypoints)
  main()


def loop():
  global g_dest_x, g_dest_y
  global g_error_state

  # Need goal world coordinates and current world coordinates
  curr_x, curr_y, curr_theta = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta
  print(curr_x, curr_y, g_dest_x, g_dest_y)

  # Figure out dest_theta by pointing to dest_x, dest_y
  dest_theta = math.atan2(g_dest_y - curr_y, g_dest_x - curr_x)

  # Compute error functions
  d_err = math.sqrt(math.pow(curr_x - g_dest_x, 2) + math.pow(curr_y - g_dest_y, 2))
  b_err = dest_theta - curr_theta

  # Keep variables in appropriate range
  if b_err > math.pi: b_err -= 2 * math.pi
  if b_err <= -math.pi: b_err += 2 * math.pi

  if (abs(b_err) < 0.02):
    g_error_state = g_FIXING_DISTANCE_ERROR
  elif (abs(d_err) < 0.05):
    g_error_state = g_FIXING_BEARING_ERROR

  if (abs(b_err) < 0.02 and abs(d_err) < 0.05):
    # Get next waypoint, if there is one
    if g_waypoints:
      g_dest_x, g_dest_y = g_waypoints.pop(0)
      return 1 # We are still going
    else:
      print("No more waypoints to go, you have reached your destination.")
      return 0 # No more waypoints

  # Fix error
  motor_movement = Float32MultiArray()
  if (g_error_state == g_FIXING_BEARING_ERROR):
    # move towards fixing bearing
    if b_err >= 0:
      motor_movement.data = [1.0, -1.0] # turn left
    else:
      motor_movement.data = [-1.0, 1.0] # turn right
  else:
    # move towards fixing distance
    motor_movement.data = [1.0, 1.0]

  # Send movement to simulator
  publisher_motor.publish(motor_movement)
  publisher_render.publish(Empty())

  return 1


def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global IR_THRESHOLD, g_CYCLE_TIME
    global pose2d_sparki_odometry
    global g_dest_x, g_dest_y

    init()
    g_dest_x, g_dest_y = g_waypoints.pop(0)
    g_dest_x, g_dest_y = g_waypoints.pop(0)

    while not rospy.is_shutdown():
        # Implement cycle time
        starting_time = time.time()

        if loop():
          pass
        else:
          print("Stop moving")
          motor_movement = Float32MultiArray()
          motor_movement.data = [0.0, 0.0]
          publisher_motor.publish(motor_movement)
          publisher_render.publish(Empty())

        # Line following
        #   To create a message for changing motor speed, use Float32MultiArray()
        #   (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        # motor_movement = Float32MultiArray()
        # if IR_sensors[2] < IR_THRESHOLD:
        #     motor_movement.data = [1.0, 1.0]
        # elif IR_sensors[1] < IR_THRESHOLD: # move left
        #     motor_movement.data = [-1.0, 1.0]
        # elif IR_sensors[3] < IR_THRESHOLD: # move right
        #     motor_movement.data = [1.0, -1.0]
        # else:
        #     motor_movement.data = [0.0, 0.0]
        # publisher_motor.publish(motor_movement)
        # publisher_render.publish(Empty())

        # Implement cycle time
        difference_time = time.time() - starting_time
        rospy.sleep(g_CYCLE_TIME - difference_time)


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry

    # Set up publishers and subscribers
    rospy.init_node("lab4")

    publisher_motor = rospy.Publisher("/sparki/motor_command", Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher("/sparki/ping_command", Empty, queue_size=10)
    publisher_servo = rospy.Publisher("/sparki/set_servo", Int16, queue_size=10)
    publisher_odom = rospy.Publisher("/sparki/set_odometry", Pose2D, queue_size=10)
    publisher_render = rospy.Publisher("/sparki/render_sim", Empty, queue_size=10)

    subscriber_odometry = rospy.Subscriber("/sparki/odometry", Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber("/sparki/state", String, callback_update_state)

    rospy.sleep(1)

    # Set up initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    # pose2d_sparki_odometry = Pose2D()

    # Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    # publisher_servo.publish(45)


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    # Copy data into local odometry variable
    pose2d_sparki_odometry = data


def callback_update_state(data):
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    # Load data into program's state variables
    global IR_sensors, servo_angle

    IR_sensors = state_dict["light_sensors"]
    servo_angle = state_dict["servo"]
    if "ping" in state_dict and state_dict["ping"] != -1:
      print("You shouldn't be asking for ultrasonic data.")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  # part_1()
  part_2(args)
