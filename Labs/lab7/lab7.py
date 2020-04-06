import math
import argparse
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


def main(args):
    print(args)
    
    # Set up node and publisher
    rospy.init_node("lab7")
    publisher_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.sleep(1)

    # Create message
    header = Header(stamp=rospy.Time.now(), frame_id="map")
    position = Point(x=args.x, y=args.y)
    orientation = Quaternion(z=math.sin(0.5 * args.theta), w=math.cos(0.5 * args.theta))
    pose = Pose(position=position, orientation=orientation)
    goal = PoseStamped(header=header, pose=pose)
    print(goal)

    # Publish message
    publisher_goal.publish(goal)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Getting to a destination on a Gazebo and RViz map")
    parser.add_argument("-x", "--x", type=float, default=0.0, help="Goal x coordinate")
    parser.add_argument("-y", "--y", type=float, default=0.0, help="Goal y coordinate")
    parser.add_argument("-theta", "--theta", type=float, default=0.0, help="Goal theta orientation")
    args = parser.parse_args()

    main(args)
