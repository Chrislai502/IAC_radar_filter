import rclpy
from rclpy.node import Node

from delphi_esr_msgs.msg import EsrTrack
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque

# ---------------------------------------------------------------------------- #
#                                FILTERING NODE                                #
# ---------------------------------------------------------------------------- #
class FilterNode(Node):
  
  def __init__(self):
    super().__init__('radar_filter_node')

    # QOS profile for the subscription
    self.qos_profile =  QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
    )

    # Create the subscriptions
    '''
    std_msgs/Header header

    # ESR Track (Not the most accurate)
    string        canmsg

    uint8         id
    float32       lat_rate
    bool          grouping_changed
    bool          oncoming
    uint8         status
    float32       angle
    float32       range
    bool          bridge_object
    bool          rolling_count
    float32       width
    float32       range_accel
    uint8         med_range_mode
    float32       range_rate

    Example ESR message:
    header:
      stamp:
        sec: 1673037874
        nanosec: 743309147
      frame_id: radar_front
    canmsg: 7c6065b83ffb559
    track_id: 42
    track_lat_rate: 7.75
    track_group_changed: false
    track_status: 3
    track_angle: -1.2000000476837158
    track_range: 146.40000915527344
    track_bridge_object: false
    track_rolling_count: false
    track_width: 0.0
    track_range_accel: -0.05000000074505806
    track_med_range_mode: 2
    track_range_rate: -27.26999855041504

    
    Example Marker message:
    Marker basically only have x and Y coordinates.
    '''
    self.esr_track_subscription = self.create_subscription(
        EsrTrack, 
        '/radar_front/esr_track', 
        self.esr_track_callback, 
        10)
    
    self.marker_subscription = self.create_subscription(
        Marker, 
        '/radar_front/radar_visz_static', 
        self.marker_callback, 
        10)
    
    # Create Publisher
    self.marker_publisher = self.create_publisher(Marker, '/radar_filtered', 10)
    
    # Create the buffers
    self.esr_id_buffer = deque(maxlen=6)
    self.esr = deque(maxlen=6)
    self.most_recent_marker = None
    self.markers = deque(maxlen=10)     # Pick the minimum Marker over 6 markers to publish
    self.distances = deque(maxlen=10)
  
  def color_marker_red(self, marker):
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker


  # ---------------------------------------------------------------------------- #
  #                                 ESR CALLBACK                                 #
  # ---------------------------------------------------------------------------- #
  def esr_track_callback(self, msg):
    '''
    The ESR callback function is the function that will be called the most.
    Based on experieriment with ROSBAG data, There is around 7 ESR msgs for every
    marker msg. And Descovery is that ESR messages does come a little later than 
    its marker messages. Example is shown below. 
    '''

    # Append the ID and the message to the buffer
    self.esr_id_buffer.append(msg.track_id)
    self.esr.append(msg)

    # If the ESR message comes later within maxlen - 1, compare the two messages
    '''
    Example:
    esr ID: 61
    esr ID: 0
    esr ID: 3
    esr ID: 6
    marker ID: 10
    esr ID: 7
    esr ID: 9
    esr ID: 10
    esr ID: 12
    esr ID: 14
    '''
    if self.most_recent_marker != None and \
      msg.track_id == self.most_recent_marker.id:
      print (f"ID of both messages match!, marker ID: {self.most_recent_marker.id}, esr ID: {msg.track_id}")
      print (f"esr dist: {msg.track_range:.2f}", f"esr speed:{msg.track_range_rate:.2f}", \
             f"marker x: {self.most_recent_marker.pose.position.x:.2f}, marker y: {self.most_recent_marker.pose.position.y:.2f}")

      # append the distance to the list
      self.distances.append(msg.track_range)
      self.markers.append(self.most_recent_marker)

      # Publish the candidate marker
      idx = self.distances.index(min(self.distances))
      print(f"Candidate Marker Range: , {self.distances[idx]:.2f}")
      self.marker_publisher.publish(self.color_marker_red(self.markers[idx]))

      # Clear the buffers
      self.esr.clear()
      self.esr_id_buffer.clear()
      self.most_recent_marker = None
      self.filtered_marker = None


  # ---------------------------------------------------------------------------- #
  #                                MARKER CALLBACK                               #
  # ---------------------------------------------------------------------------- #
  def marker_callback(self, msg):

    # Update the most recent marker
    self.most_recent_marker = msg

    # If the marker message comes later
    '''
    Rare case, ESR msg normally comes later.
    But regardless, check if ESR has arrived.
    '''
    self.check()

  # Check the two messages and filter our if necessary
  def check(self):
    
    if self.esr is not None and self.most_recent_marker is not None:
      
      # Check if the ID of most recent marker is in the ESR buffer
      if self.most_recent_marker.id in self.esr_id_buffer:

        # get the index of the matched ESR message
        index = self.esr_id_buffer.index(self.most_recent_marker.id)
        print (f'ID of both messages match!, marker ID: {self.most_recent_marker.id}, esr ID: {self.esr[index].track_id}')
        
        # append the distance to the list
        self.distances.append(self.esr[index].track_range)
        self.markers.append(self.most_recent_marker)

        # print x, y, of the markers
        index = self.esr_id_buffer.index(self.most_recent_marker.id)
        print (f"esr dist: {self.esr[index].track_range:.2f}", f"esr speed:{self.esr[index].track_range_rate:.2f}", \
              f"marker x: {self.most_recent_marker.pose.position.x:.2f}, marker y: {self.most_recent_marker.pose.position.y:.2f}")

        # Publish the candidate marker
        idx = self.distances.index(min(self.distances))
        print(f"Candidate Marker Range: , {self.distances[idx]:.2f}")
        self.marker_publisher.publish(self.color_marker_red(self.markers[idx]))

        # Clear the buffers
        self.esr.clear()
        self.esr_id_buffer.clear()
        self.most_recent_marker = None

# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)

    node = FilterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()