import hashlib
import rclpy
import colorsys
import numpy as np
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')

        # Paraméterek
        self.declare_parameter('min_unknown_cells', 12)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # ROS interfészek
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/global_frontiers', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot nyilvántartás
        self.robot_frames = {}     
        self.action_clients = {}
        self.current_targets = {}  # Aktuális (y, x) célpont robotonként
        self.blacklists = {}       # Hibás (y, x) pontok készlete robotonként
        
        self.latest_map_msg = None
        self.current_frontiers = []

        # Időzítő az új robotok keresésére (10 másodpercenként)
        self.timer = self.create_timer(10.0, self.scan_for_active_robots)

    def scan_for_active_robots(self):
        # Megnézzük a topic listát robot_description után kutatva
        topic_list = self.get_topic_names_and_types()
        map_topics = [n for n, t in topic_list if n.endswith('/robot_description')]
        robot_names = [t.split('/')[1] for t in map_topics if len(t.split('/')) > 2]
        
        for name in robot_names:
            if name not in self.robot_frames:
                self.robot_frames[name] = f"{name}/base_link"
                self.action_clients[name] = ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
                self.current_targets[name] = None
                self.blacklists[name] = set()
                self.get_logger().info(f"+++ Robot '{name}' detektálva és inicializálva.")

    def map_callback(self, msg):
        self.latest_map_msg = msg
        # Frontier keresés és zóna-alapú szűrés
        self.current_frontiers = self.find_frontiers(msg)
        self.publish_frontier_markers(self.current_frontiers, msg)

        # Ha egy robot tétlen, adunk neki munkát
        for robot in self.robot_frames:
            if self.current_targets[robot] is None:
                self.assign_next_goal(robot)

    def find_frontiers(self, map_msg):
        height, width = map_msg.info.height, map_msg.info.width
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        res = map_msg.info.resolution

        origin = map_msg.info.origin.position
        
        raw_frontiers = []
        # BIZTONSÁGI MARZS: Legalább 6 pixel (kb. 30cm) távolság a falaktól
        margin = 6 
        
        for y in range(margin, height - margin, 3): # 3-as lépésköz a sebességért
            for x in range(margin, width - margin, 3):
                # Szabad terület és van ismeretlen szomszédja
                if data[y, x] == 0 and -1 in data[y-1:y+2, x-1:x+2]:
                    # Fal-ellenőrzés a környezetben
                    if np.sum(data[y-margin:y+margin+1, x-margin:x+margin+1] == 100) == 0:
                        raw_frontiers.append((y, x))

        if not raw_frontiers:
            return []

        clusters = []
        cell_dist_limit = 1.5 / res # 1.5 méteren belüli pontokat egynek veszünk
        
        while raw_frontiers:
            by, bx = raw_frontiers.pop(0)
            world_x = origin.x + bx * res
            world_y = origin.y + by * res
            
            # ZÓNA BLACKLIST ELLENŐRZÉS
            is_blocked = False
            for r_name in self.blacklists:
                for fy, fx in self.blacklists[r_name]:
                    dist = math.sqrt((world_x - (origin.x + fx*res))**2 + 
                                     (world_y - (origin.y + fy*res))**2)
                    if dist < 1.2: # 1.2 méteres körzetben tiltjuk a hibás pont környékét
                        is_blocked = True
                        break
                if is_blocked: break
            
            if not is_blocked:
                clusters.append((by, bx))
                # Clustering: kidobjuk a közeli többi nyers pontot
                raw_frontiers = [f for f in raw_frontiers if math.hypot(f[0]-by, f[1]-bx) > cell_dist_limit]
            
            if len(clusters) > 15: break
            
        return clusters

    def assign_next_goal(self, robot_name):
        if self.latest_map_msg is None or not self.current_frontiers:
            return
        if self.current_targets[robot_name] is not None:
            return

        res = self.latest_map_msg.info.resolution
        origin = self.latest_map_msg.info.origin.position
        
        try:
            t = self.tf_buffer.lookup_transform("world", self.robot_frames[robot_name], rclpy.time.Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
        except: return

        best_f = None
        min_dist = float('inf')
        # Ne menjünk oda, ahova a másik robot már tart
        taken = [v for k, v in self.current_targets.items() if k != robot_name and v is not None]

        for f in self.current_frontiers:
            if f in taken: continue
            dist = math.hypot(origin.x + f[1]*res - rx, origin.y + f[0]*res - ry)
            
            if dist < 0.8: continue # Túl közelre nem küldjük (rángatózás ellen)
            
            if dist < min_dist:
                min_dist = dist
                best_f = f

        if best_f:
            self.current_targets[robot_name] = best_f
            self.send_action_goal(robot_name, best_f)

    def send_action_goal(self, robot_name, cell):
        client = self.action_clients[robot_name]
        if not client.wait_for_server(timeout_sec=1.0):
            self.current_targets[robot_name] = None
            return

        # PoseStamped manuális felépítése a típusbiztonság miatt
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "world"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        res = self.latest_map_msg.info.resolution
        origin = self.latest_map_msg.info.origin.position
        goal_pose.pose.position.x = origin.x + (cell[1] + 0.5) * res
        goal_pose.pose.position.y = origin.y + (cell[0] + 0.5) * res
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"[{robot_name}] Cél küldése: {goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}")
        
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, robot_name))

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{robot_name}] A Nav2 elutasította a célt!")
            self.current_targets[robot_name] = None
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.get_result_callback(fut, robot_name))

    def get_result_callback(self, future, robot_name):
        status = future.result().status
        # 4 = SUCCEEDED
        if status != 4:
            failed_target = self.current_targets[robot_name]
            if failed_target:
                self.get_logger().error(f"[{robot_name}] HIBA (Status: {status}). Terület blacklistre téve.")
                self.blacklists[robot_name].add(failed_target)
        
        self.current_targets[robot_name] = None
        # Rövid várakozás után újratervezés
        self.assign_next_goal(robot_name)

    def publish_frontier_markers(self, frontiers, map_msg):
        m = Marker(type=Marker.SPHERE_LIST, action=Marker.ADD, ns="frontiers")
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.8
        
        res, origin = map_msg.info.resolution, map_msg.info.origin.position
        for y, x in frontiers:
            p = Point(x=origin.x + x*res, y=origin.y + y*res, z=0.1)
            m.points.append(p)
        
        self.marker_pub.publish(MarkerArray(markers=[m]))

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()S