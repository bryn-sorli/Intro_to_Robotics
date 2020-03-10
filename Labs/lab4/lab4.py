import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
servo_angle = 0
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
IR_sensors = None
#TODO: Create data structure to hold map representation
map_sub = None
height = 42
width = 60
max_distance = math.sqrt(height**2 + width**2)

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None
publisher_render = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.05 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        # Implement cycle time
        starting_time = time.time()

        # Line following
        #   To create a message for changing motor speed, use Float32MultiArray()
        #   (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        motor_movement = Float32MultiArray()
        if IR_sensors[2] < IR_THRESHOLD:
            motor_movement.data = [1.0, 1.0]
        elif IR_sensors[1] < IR_THRESHOLD: # move left
            motor_movement.data = [-1.0, 1.0]
        elif IR_sensors[3] < IR_THRESHOLD: # move right
            motor_movement.data = [1.0, -1.0]
        else:
            motor_movement.data = [0.0, 0.0]
        publisher_motor.publish(motor_movement)
        publisher_render.publish(Empty())

        # Loop closure
        if IR_sensors[1] < IR_THRESHOLD and IR_sensors[2] < IR_THRESHOLD and IR_sensors[3] < IR_THRESHOLD:
            rospy.loginfo("Loop Closure Triggered")
            new_pose = Pose2D()
            new_pose.x, new_pose.y, new_pose.theta = 0, 0, pose2d_sparki_odometry.theta
            publisher_odom.publish()

        # Publish ping to get the ultrasonic sensor data
        publisher_ping.publish(Empty())

        # Implement cycle time
        difference_time = time.time() - starting_time
        rospy.sleep(CYCLE_TIME - difference_time)

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
    # rospy.spin()

    # Set up initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()

    # Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    publisher_servo.publish(45)

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
    if "ping" in state_dict:
        # Populate map from ping by converting ultrasonic coords to robot coords to world coords lol
        populate_map_from_ping(convert_robot_coords_to_world(convert_ultrasonic_to_robot_coords(state_dict["ping"])))

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = x_us * math.cos(servo_angle), x_us * math.sin(servo_angle)
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = pose2d_sparki_odometry.x + x_r, pose2d_sparki_odometry.y + y_r
    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    global map_sub
    return

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    global width
    return (width*j + i)

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    global width
    return (cell_index//width, cell_index%width)

def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    global max_distance
    goal =  cell_index_to_ij(cell_index_to)
	position = cell_index_to_ij(cell_index_from) 
    t_cost = math.sqrt(goal**2 + position**2)

    t_cost = t_cost*100

    return t_cost

if __name__ == "__main__":
    main()
