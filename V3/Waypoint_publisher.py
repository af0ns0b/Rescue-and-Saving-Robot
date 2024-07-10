import xml.etree.ElementTree as ET
import rospy # type: ignore
from std_msgs.msg import String

def parse_kml_file(kml_file):
    tree = ET.parse(kml_file)
    root = tree.getroot()
    
    # Find the Document element in the KML file
    document = root.find('.//{{http://www.opengis.net/kml/2.2}Document}')
    
    # Find the Folder element containing the waypoints 
    folder = document.find('.//{{http://www.opengis.net/kml/2.2}Folder}')

    # Get the list of Placemark elements (waypoints)
    waypoints = folder.findall('.//{{http://www.opengis.net/kml/2.2}Placemark}')

    # Extract the waypoint coordinates and names
    waypoints_list = []
    for waypoint in waypoints:
        name = waypoint.find('.//{{http://www.opengis.net/kml/2.2}name}').text
        coords = waypoint.find('.//{{http://www.opengis.net/kml/2.2}Point}').find('.//{{http://www.opengis.net/kml/2.2}coordinates}').text
        lat, lon, alt = coords.split(',')
        waypoints_list.append((name, lat, lon, alt))

    return waypoints_list

def send_waypoints_to_robot(waypoints_list):
    # Initialize the ROS node
    rospy.init_node('waypoint_publisher')

    # Create a publisher for the waypoint topic
    waypoint_pub = rospy.Publisher('robot_waypoints', String, queue_size=10)

    # Loop through the waypoints and publish them to the topic
    for waypoint in waypoints_list:
        name, lat, lon, alt = waypoint
        waypoint_msg = f'WAYPOINT,{name},{lat},{lon},{alt}'
        waypoint_pub.publish(waypoint_msg)
        rospy.sleep(10.)  # Wait for 10 second before publishing the next waypoint

    # Shutdown the ROS node
    rospy.signal_shutdown('Waypoint publisher finished')

def main():
    # Parse the KML file and extract the waypoints
    kml_file = '/home/afonso/catkin_ws/src/waypoint/src/waypoint_mission.kml'
    waypoints_list = parse_kml_file(kml_file)

    # Create a ROS node to receive waypoints
    rospy.init_node('waypoint_publisher')

    # Send the waypoints to the robot using ROS
    send_waypoints_to_robot(waypoints_list)

    # Spin the ROS node
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

