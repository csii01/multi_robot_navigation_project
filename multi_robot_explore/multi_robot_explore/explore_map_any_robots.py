import hashlib
from shapely import node
import rclpy
import colorsys
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import ColorRGBA
import numpy as np

from nav2_msgs.srv import ClearEntireCostmap

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')

        self.declare_parameter('min_unknown_cells', 12)
        self.min_unknown_cells = self.get_parameter('min_unknown_cells').get_parameter_value().integer_value
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.add_on_set_parameters_callback(self.update_parameter_callback)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/global_frontiers', 10)
        self.robot_frames = {}     
        self.goal_pubs = {}

        self.global_map = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_targets = {}
        self.target_start_times = {}
        self.blacklists = {}  # EREDETI: Ez a dict tárolja a feketelistát
        self.retreat_until = {}
        self.robot_marker_colors = {}

        self.robot_scan_period_sec = 10.0
        self.timer = self.create_timer(self.robot_scan_period_sec, self.scan_for_active_robots)

    def scan_for_active_robots(self):
        node = rclpy.create_node('robot_scanner')
        topic_list = node.get_topic_names_and_types()
        map_topics = [name for name, types in topic_list if name.endswith('/robot_description')]
        robot_names = [t.split('/')[1] for t in map_topics if len(t.split('/')) > 2]
        node.destroy_node()
        self.update_robot_lists(robot_names)
    
    def update_robot_lists(self, robot_names):
        for name in robot_names:
            if name not in self.robot_frames:
                self.robot_frames[name] = f"{name}/base_link"
                self.goal_pubs[name] = self.create_publisher(PoseStamped, f'/{name}/goal_pose', 10)
                self.current_targets[name] = None
                self.target_start_times[name] = None
                
                # MÓDOSÍTÁS 1: Halmaz (set) helyett egy szótárt (dict) hozunk létre a robotnak.
                # Ebben fogjuk tárolni a (y, x) koordinátákat és a hozzájuk tartozó időbélyeget.
                self.blacklists[name] = {}  
                self.retreat_until[name] = 0.0

                color_rgba = ColorRGBA()
                hash_object = hashlib.sha256(name.encode('utf-8'))
                hash_hex = hash_object.hexdigest()
                hash_int = int(hash_hex[:8], 16)
                hue = (hash_int) % 360
                r, g, b = colorsys.hsv_to_rgb(hue/360.0, 0.8, 0.8)

                color_rgba.r = r
                color_rgba.g = g
                color_rgba.b = b
                color_rgba.a = 1.0
                self.robot_marker_colors[name] = color_rgba
                self.get_logger().info(f"New robot found '{name}' found, added to dictionaries")

    def update_parameter_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'min_unknown_cells' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.min_unknown_cells = param.value
                self.get_logger().info(f'Updating minimum unknown cells threshold to {self.min_unknown_cells}')
                return result
        return result

    # MÓDOSÍTÁS 2: Új függvény a feketelista tisztítására.
    def clean_expired_blacklists(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for robot in self.blacklists:
            # Összegyűjtjük azokat a cellákat, amik régebben kerültek be, mint 60 másodperc
            expired_cells = [cell for cell, timestamp in self.blacklists[robot].items() if now - timestamp > 60.0]
            # Töröljük őket a szótárból
            for cell in expired_cells:
                del self.blacklists[robot][cell]
                self.get_logger().info(f"[{robot}] Elfelejtettük a régi feketelistás pontot: {cell}")
    # --------------------------------------------------------

    def get_home_pose(self, map_msg, robot_name):
        try:
            map_frame = f"{robot_name}/map"
            pose_in_map = PoseStamped()
            pose_in_map.header.frame_id = map_frame
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.pose.position.x = 0.0
            pose_in_map.pose.position.y = 0.0
            pose_in_map.pose.position.z = 0.0
            pose_in_map.pose.orientation.w = 1.0

            transform = self.tf_buffer.lookup_transform(
                'world', map_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )
            pose_world = do_transform_pose(pose_in_map.pose, transform)

            resolution = map_msg.info.resolution
            origin = map_msg.info.origin.position
            x = int((pose_world.position.x - origin.x) / resolution)
            y = int((pose_world.position.y - origin.y) / resolution)
            return (y, x)

        except Exception as e:
            self.get_logger().warn(f"Failed to compute home cell for {robot_name}: {e}")
            return None

    def get_closest_frontier(self, frontiers, map_msg, robot_frame):
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        try:
            transform = self.tf_buffer.lookup_transform(
                map_msg.header.frame_id, robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            def frontier_distance(f):
                y, x = f
                fx = origin.x + (x + 0.5) * resolution
                fy = origin.y + (y + 0.5) * resolution
                return np.hypot(fx - rx, fy - ry)
            return min(frontiers, key=frontier_distance)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed for {robot_frame}: {e}")
            return None

    def transform_blacklists_to_world(self, map_msg):
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        world_points = []

        try:
            transforms = {}
            for name in self.robot_frames:
                transforms[name] = self.tf_buffer.lookup_transform(
                'world', f'{name}/map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for robot_x/map to world: {e}")

        for robot, blacklist in self.blacklists.items():
            frame = f"{robot}/map"
            try:
                transform = transforms[robot]
                # MÓDOSÍTÁS 3: Mivel a blacklist már szótár (dict), a keys() metódust kell hívni
                for y, x in blacklist.keys(): 
                    local_x = origin.x + (x + 0.5) * resolution
                    local_y = origin.y + (y + 0.5) * resolution
                    pose = PoseStamped()
                    pose.header.frame_id = frame
                    pose.pose.position.x = local_x
                    pose.pose.position.y = local_y
                    pose.pose.position.z = 0.1
                    pose.pose.orientation.w = 1.0
                    try:
                        transformed = do_transform_pose(pose.pose, transform)
                        world_points.append(Point(
                            x=transformed.position.x,
                            y=transformed.position.y,
                            z=transformed.position.z))
                    except Exception as e:
                        self.get_logger().warn(f"Transform failed for blacklist {robot} cell {(y,x)}: {e}")
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {frame} to world: {e}")

        return world_points

    def publish_blacklist_markers(self, world_points):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "blacklisted_frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2)

        marker.points.extend(world_points)
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def map_callback(self, msg):
        # MÓDOSÍTÁS 4: Minden térképfrissítésnél meghívjuk a feketelista takarítóját
        self.clean_expired_blacklists() 
        
        self.check_and_blacklist_stuck_targets(msg)
        self.global_map = msg
        frontiers = self.find_frontiers(msg)
        
        blacklist_points = self.transform_blacklists_to_world(msg)
        self.publish_blacklist_markers(blacklist_points)

        filtered_frontiers = []
        for cell in frontiers:
            py = msg.info.origin.position.y + (cell[0] + 0.5) * msg.info.resolution
            px = msg.info.origin.position.x + (cell[1] + 0.5) * msg.info.resolution
            if all(np.hypot(px - p.x, py - p.y) > 0.8 for p in blacklist_points):
                filtered_frontiers.append(cell)

        self.publish_frontier_markers(filtered_frontiers, msg)
        frontiers = filtered_frontiers

        if len(frontiers) == 0:
            self.get_logger().info("No more frontiers found")
            for robot, frame in self.robot_frames.items():
                home = self.get_home_pose(msg, robot)
                self.publish_goal_pose(home, msg, robot)
        else:
            self.get_logger().info(f"Found {len(frontiers)} frontiers")
            for robot, frame in self.robot_frames.items():
                closest = self.get_closest_frontier(frontiers, msg, frame)
                if closest:
                    self.publish_selected_frontier(closest, msg, robot)
                    self.publish_goal_pose(closest, msg, robot)

    def find_frontiers(self, map_msg):
        height = map_msg.info.height
        width = map_msg.info.width
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        frontiers = []

        for y in range(2, height - 2):
            for x in range(2, width - 2):
                if data[y, x] != 0:
                    continue
                neighborhood = data[y-1:y+2, x-1:x+2].flatten()
                if -1 not in neighborhood:
                    continue
                unknown_area = data[y-2:y+3, x-2:x+3]
                if np.sum(unknown_area == -1) < self.min_unknown_cells:
                    continue
                if np.sum(unknown_area == 100) > 2:
                    continue
                frontiers.append((y, x))

        return frontiers

    def publish_selected_frontier(self, cell, map_msg, robot_name):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{robot_name}_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color = self.robot_marker_colors[robot_name]
        marker.lifetime = Duration(sec=2)

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        y, x = cell
        px = origin.x + (x + 0.5) * resolution
        py = origin.y + (y + 0.5) * resolution
        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def check_and_blacklist_stuck_targets(self, map_msg):
        now = self.get_clock().now().nanoseconds / 1e9
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        try:
            transforms = {name: self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)) for name, frame in self.robot_frames.items()}
            map_transforms = {name: self.tf_buffer.lookup_transform(f'{name}/map', 'world', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)) for name in self.robot_frames}
        except Exception:
            return

        for robot, target in self.current_targets.items():
            if not target: continue
            start_time = self.target_start_times[robot]
            if not start_time or now - start_time < 10.0: continue

            try:
                transform = transforms[robot]
                rx, ry = transform.transform.translation.x, transform.transform.translation.y
                y, x = target
                tx, ty = origin.x + (x + 0.5) * resolution, origin.y + (y + 0.5) * resolution
                dist = np.hypot(tx - rx, ty - ry)
                
                self.get_logger().warn(f"[{robot}] DEADLOCK DETECTED! (Distance: {dist:.1f}m). Initiating escape maneuver.")
                
                # 1. LOKÁLIS ÉS GLOBÁLIS COSTMAP TÖRLÉSE (Hogy a planner hajlandó legyen vonalat rajzolni)
                client_local = self.create_client(ClearEntireCostmap, f'/{robot}/local_costmap/clear_entirely_local_costmap')
                if client_local.wait_for_service(timeout_sec=0.5):
                    client_local.call_async(ClearEntireCostmap.Request())
                    
                client_global = self.create_client(ClearEntireCostmap, f'/{robot}/global_costmap/clear_entirely_global_costmap')
                if client_global.wait_for_service(timeout_sec=0.5):
                    client_global.call_async(ClearEntireCostmap.Request())

                # 2. Feketelista (ha túl közel van)
                if dist < 5.0:
                    try:
                        t_pose = do_transform_pose(PoseStamped(pose=Point(x=tx, y=ty, z=0.0)), map_transforms[robot])
                        lx, ly = int((t_pose.position.x - origin.x) / resolution), int((t_pose.position.y - origin.y) / resolution)
                        self.blacklists[robot][(ly, lx)] = now
                    except Exception: pass

                # 3. MENEKÜLÉS HAZA (És a marker kirajzolása!)
                self.retreat_until[robot] = now + 15.0
                home_cell = self.get_home_pose(map_msg, robot)
                if home_cell:
                    self.get_logger().info(f"[{robot}] Retreating to Home position for 15 seconds.")
                    # Kirajzoljuk a nagy bogyót a Home pozícióra is, hogy LÁSSUK!
                    self.publish_selected_frontier(home_cell, map_msg, robot) 
                    self.publish_goal_pose(home_cell, map_msg, robot)
                    self.current_targets[robot] = home_cell
                    self.target_start_times[robot] = now

            except Exception as e:
                self.get_logger().warn(f"Stuck check failed: {e}")

    def publish_goal_pose(self, cell, map_msg, robot_name):
        pub = self.goal_pubs[robot_name]
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position
        y, x = cell
        pose.pose.position.x = origin.x + (x + 0.5) * resolution
        pose.pose.position.y = origin.y + (y + 0.5) * resolution
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        pub.publish(pose)
        if self.current_targets[robot_name] != cell:
            self.current_targets[robot_name] = cell
            self.target_start_times[robot_name] = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Sent goal for {robot_name} to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

    def publish_frontier_markers(self, frontiers, map_msg):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "global_frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2)

        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        for y, x in frontiers:
            px = origin.x + (x + 0.5) * resolution
            py = origin.y + (y + 0.5) * resolution
            marker.points.append(Point(x=px, y=py, z=0.1))

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()