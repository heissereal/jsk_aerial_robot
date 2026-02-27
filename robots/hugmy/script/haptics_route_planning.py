#!/usr/bin/env python3
import rospy, copy, math, threading
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker,MarkerArray
from geometry_msgs.msg import Pose, PoseArray,Point
from std_msgs.msg import Float32MultiArray

WAYPOINT_PREFIX = "wp_"
OBST_PREFIX = "obs_"

class PathUI:
    def __init__(self):
        self.server = InteractiveMarkerServer("path_ui")
        self.menu = MenuHandler()

        # publishers
        self.pub_wps = rospy.Publisher("/nav/waypoints", PoseArray, queue_size=1, latch=True)
        self.pub_obs = rospy.Publisher("/nav/obstacles/circles", Float32MultiArray, queue_size=1, latch=True)
        self.field_pub = rospy.Publisher("/nav/field", Marker, queue_size=1, latch=True)
        self.text_pub = rospy.Publisher("/nav/text", MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("/nav/path_marker", Marker, queue_size=1, latch=True)
        self.path_pts_pub = rospy.Publisher("/nav/path_points", PoseArray, queue_size=1, latch=True)

        self.waypoint_for_robot_pub = rospy.Publisher("/waypoints", PoseArray, queue_size=1)

        self.mode = rospy.get_param("~mode", "edit")  # "edit" or "plan"

        self.max_seg_len = rospy.get_param("~max_seg_len", 0.5)
        self.path_z = rospy.get_param("~path_z", 0.0)


        self.rebuild_delay = rospy.get_param("~rebuild_delay", 0.05)
        self.rebuild_rate  = rospy.get_param("~rebuild_rate", 20.0)

        self._lock = threading.RLock()
        self._dirty = False
        self._next_rebuild_time = rospy.Time(0)

        self._ui_timer = rospy.Timer(rospy.Duration(1.0/self.rebuild_rate), self._ui_tick)

        # state
        self.frame = rospy.get_param("~frame_id", "map") ###### should change
        self.wp_radius_vis = rospy.get_param("~wp_radius_vis", 0.08)
        self.obs_default_r = rospy.get_param("~obs_default_radius", 0.35)
        self.obstacles = []  # list of dict: {"name": str, "x": float, "y": float, "r": float}
        self.waypoints = []  # list of dict: {"name": str, "x": float, "y": float, "z": float}
        self.user_set_waypoints = []

        self.last_text_cnt = 0
        self.clear_all_text()

        # context menu entries
        self.entry_add_wp_here = self.menu.insert("Add waypoint", callback=self.menu_add_wp_here)
        self.entry_delete = self.menu.insert("Delete", callback=self.menu_delete)
        self.entry_set_order = self.menu.insert("Set order by index...", callback=self.menu_noop)  # placeholder

        self.server.applyChanges()
        self.publish_arrays()

        # initial seeds
        init_wps = rospy.get_param("~init_waypoints", [[0,0,0],[2,0,0],[2,2,0]])
        for i, p in enumerate(init_wps):
           self.add_waypoint(p[0], p[1], p[2])

        init_obs = rospy.get_param("~init_obstacles", [[1.0,0.7,0.3]])
        for x,y,r in init_obs:
            self.add_obstacle(x,y,r)

        self.add_field()

    def clear_all_text(self):
        ma = MarkerArray()
        # for i in range(self.last_text_cnt + 20):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        # m.id = i
        m.ns = "wp_text"
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.text_pub.publish(ma)
        self.last_text_cnt = 0

    def make_marker_sphere(self, scale, r,g,b,a):
        m = Marker()
        m.type = Marker.SPHERE
        m.scale.x = scale; m.scale.y = scale; m.scale.z = scale
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a
        return m

    def make_button_marker(self, label):
        button = Marker()
        button.ns = "btn"
        button.id = 0
        button.type = Marker.CUBE
        button.pose.orientation.w = 1.0
        button.scale.x = 0.35; button.scale.y = 0.18; button.scale.z = 0.06
        button.color.r = 0.2; button.color.g = 0.2; button.color.b = 0.2; button.color.a = 1.0

        text = Marker()
        text.ns = "btn"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = 0.06
        text.pose.orientation.w = 1.0
        text.scale.z = 0.10
        text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
        text.text = label
        return button, text

    def make_marker_cylinder(self, radius, height, r,g,b,a):
        m = Marker()
        m.type = Marker.CYLINDER
        m.scale.x = radius*2; m.scale.y = radius*2; m.scale.z = height
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a
        return m

    def make_path_marker(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "target_path"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.scale.x = 0.03# line width

        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 1.0

        for w in self.waypoints:
            p = Point()
            p.x = w["x"]
            p.y = w["y"]
            p.z = w["z"]
            m.points.append(p)
        self.path_pub.publish(m)

    def drag_control(self):
        ctrl = InteractiveMarkerControl()
        ctrl.always_visible = True
        if self.mode == "edit":
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            ctrl.orientation.w = 1.0
            ctrl.orientation.x = 0.0
            ctrl.orientation.y = 1.0  # XY平面でドラッグ
            ctrl.orientation.z = 0.0
        else:
            ctrl.interaction_mode = InteractiveMarkerControl.NONE
        return ctrl

    def insert_im(self, name, pose, marker, is_obstacle=False):
        im = InteractiveMarker()
        im.header.frame_id = self.frame
        im.name = name
        im.description = name
        im.pose = pose
        im.scale = 0.3

        drag_ctrl = self.drag_control()
        drag_ctrl.markers.append(marker)
        im.controls.append(drag_ctrl)
        menu_ctrl = InteractiveMarkerControl()
        menu_ctrl.interaction_mode = InteractiveMarkerControl.MENU
        menu_ctrl.name = "menu"
        im.controls.append(menu_ctrl)

        self.server.insert(im, self.fb_move)
        self.menu.apply(self.server, name)
        # self.server.applyChanges()

    def find_wp(self, name):
        for i, w in enumerate(self.waypoints):
            if w["name"] == name: return i
        return -1

    def find_obs(self, name):
        for i, o in enumerate(self.obstacles):
            if o["name"] == name: return i
        return -1

    # ---------- create ----------
    def add_waypoint(self, x,y,z):
        with self._lock:
            self.user_set_waypoints.append({"name":"","x":x,"y":y,"z":z, "auto": False})
        self.request_rebuild(delay=0.02)

    def add_mode_button(self):
        im = InteractiveMarker()
        im.header.frame_id = self.frame
        im.name = "btn_mode"
        im.description = ""
        im.scale = 1.0

        im.pose.position.x = 0.0
        im.pose.position.y = 0.0
        im.pose.position.z = 0.6
        im.pose.orientation.w = 1.0

        label = "MODE: EDIT" if self.mode == "edit" else "MODE: PLAN"
        cube, text = self.make_button_marker(label)

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctrl.always_visible = True
        ctrl.markers.append(cube)
        ctrl.markers.append(text)
        im.controls.append(ctrl)

        self.server.insert(im, self.fb_button)


    def add_field(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.id = 0
        m.ns = "FIELD"
        m.pose = Pose(); m.pose.position.x=0.0; m.pose.position.y=0.0; m.pose.position.z=0.0; m.pose.orientation.w=1.0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = 4.0; m.scale.y = 4.0; m.scale.z = 0.01
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.2
        self.field_pub.publish(m)

    def add_waypoint_name(self):
        text_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            name = wp.get("name", "")
            if not name:
                name = f"{WAYPOINT_PREFIX}{i}"
                wp["name"] = name
            text = Marker()
            text.header.frame_id = self.frame
            text.header.stamp = rospy.Time.now()
            text.ns = "wp_text"
            text.id = i
            text.pose.position.x = wp["x"]+0.02; text.pose.position.y=wp["y"]+0.02; text.pose.position.z=wp["z"]+0.2; text.pose.orientation.w=1.0
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.15
            text.text = wp["name"]
            text.color.r = 0.0; text.color.g = 0.5; text.color.b = 0.8; text.color.a = 1.0
            text_array.markers.append(text)

        self.text_pub.publish(text_array)
        self.last_text_cnt = len(self.waypoints)


    def add_obstacle(self, x,y,r=None):
        if r is None: r = self.obs_default_r
        name = f"{OBST_PREFIX}{len(self.obstacles)}"
        pose = Pose(); pose.position.x=x; pose.position.y=y; pose.position.z=0.0
        mk = self.make_marker_cylinder(r, 0.05, 1.0,0.2,0.2,0.4)
        self.insert_im(name, pose, mk, is_obstacle=True)
        self.obstacles.append({"name":name,"x":x,"y":y,"r":r})
        self.publish_arrays()

    # ---------- feedback ----------
    def fb_move(self, fb):
        if self.mode != "edit":
            return

        name = fb.marker_name
        if not name.startswith(WAYPOINT_PREFIX):
            return

        with self._lock:
            i = self.find_wp(name)
            x = fb.pose.position.x; y = fb.pose.position.y; z = fb.pose.position.z
            if i >= 0 and i < len(self.user_set_waypoints):
                self.user_set_waypoints[i]["x"]=x; self.user_set_waypoints[i]["y"]=y; self.user_set_waypoints[i]["z"]=z
            elif name.startswith(OBST_PREFIX):
                i = self.find_obs(name)
                if i >= 0:
                    self.obstacles[i]["x"]=x; self.obstacles[i]["y"]=y
        self.request_rebuild(delay=0.0)

    def fb_button(self, fb):
        if fb.marker_name != "btn_mode":
            return
        if fb.event_type != fb.BUTTON_CLICK:
            return

        self.mode = "plan" if self.mode == "edit" else "edit"
        self.request_rebuild(delay=0.08)

    def request_rebuild(self, delay=None):
        if delay is None:
            delay = self.rebuild_delay
        with self._lock:
            self._dirty = True
            t = rospy.Time.now() + rospy.Duration(delay)
            if t > self._next_rebuild_time:
                self._next_rebuild_time = t

    def _ui_tick(self, _event):
        with self._lock:
            if not self._dirty:
                return
            if rospy.Time.now() < self._next_rebuild_time:
                return
            self._dirty = False

        self._do_rebuild()

    def _do_rebuild(self):
        with self._lock:
            # modeに応じて self.waypoints を作り直す
            if self.mode == "edit":
                self.waypoints = copy.deepcopy(self.user_set_waypoints)
            else:
                wps = copy.deepcopy(self.user_set_waypoints)
                self.waypoints = self.add_compensate_waypoints(wps)

        self.clear_all_text()
        self.update_all_marker()
        self.publish_arrays()

    # ---------- menu callbacks ----------
    def menu_add_wp_here(self, fb):
        x = fb.pose.position.x; y = fb.pose.position.y
        self.add_waypoint(x,y,0.0)

    def menu_delete(self, fb):
        if self.mode != "edit":
            return

        name = fb.marker_name
        if not name.startswith(WAYPOINT_PREFIX):
            return

        i = self.find_wp(name)
        with self._lock:
            if i >= 0 and i < len(self.user_set_waypoints):
                self.user_set_waypoints.pop(i)

        self.request_rebuild(delay=0.02)

        # # self.server.erase(name)
        # if name.startswith(WAYPOINT_PREFIX):
        #     i = self.find_wp(name)
        #     if i >= 0 and i < len(self.user_set_waypoints):
        #         self.user_set_waypoints.pop(i)
        # # elif name.startswith(OBST_PREFIX):
        # #     i = self.find_obs(name)
        # #     if i >= 0: self.obstacles.pop(i)
        # # self.clear_all_text()
        # # self.update_all_marker()
        # # self.publish_arrays()

    def update_all_marker(self):
        with self._lock:
            self.server.clear()
            self.add_mode_button()

        for i, wp in enumerate(self.waypoints):
            wp["name"] = f"{WAYPOINT_PREFIX}{i}"
            pose = Pose()
            pose.position.x = wp["x"]
            pose.position.y = wp["y"]
            pose.position.z = wp["z"]
            pose.orientation.w = 1.0

            if self.mode == "plan" and wp.get("auto",False):
                mk = self.make_marker_sphere(self.wp_radius_vis*1.5, 1.0,0.0,0.0,1.0)
            else:
                mk = self.make_marker_sphere(self.wp_radius_vis*1.5, 0.5,1.0,1.0,1.0)
            self.insert_im(wp["name"], pose, mk, is_obstacle=False)

        self.server.applyChanges()

    # def menu_obs_r_plus(self, fb):
    #     name = fb.marker_name
    #     if not name.startswith(OBST_PREFIX): return
    #     i = self.find_obs(name); 
    #     if i<0: return
    #     self.obstacles[i]["r"] *= 1.1
    #     self.update_obstacle_marker(i)

    def add_compensate_waypoints(self,wps):
        if len(wps) < 2:
            return wps

        new_wps = []
        for i in range(len(wps) - 1):
            a = wps[i]
            b = wps[i + 1]
            new_wps.append(a)

            dx = b["x"] - a["x"]
            dy = b["y"] - a["y"]
            dist = math.hypot(dx, dy)
            if dist <= 1e-9:
                continue

            n_insert = int(dist // self.max_seg_len)
            for k in range(1, n_insert + 1):
                t = (k * self.max_seg_len) / dist
                if t >= 1.0:
                    break
                new_wps.append({"name": "", "x": a["x"] + t*dx, "y": a["y"] + t*dy, "z": a["z"] + t*(b["z"] - a["z"]),"auto": True})

        new_wps.append(wps[-1])
        return new_wps

    def menu_noop(self, fb):
        with self._lock:
            for i, wp in enumerate(self.waypoints):
                wp["name"] = f"{WAYPOINT_PREFIX}{i}"

        self.clear_all_text()
        self.add_waypoint_name()

    def update_obstacle_marker(self, idx):
        o = self.obstacles[idx]
        self.server.erase(o["name"])
        pose = Pose(); pose.position.x=o["x"]; pose.position.y=o["y"]; pose.position.z=0.0
        mk = self.make_marker_cylinder(o["r"], 0.05, 1.0,0.2,0.2,0.4)
        self.insert_im(o["name"], pose, mk, is_obstacle=True)
        self.server.applyChanges()
        self.publish_arrays()

    # ---------- publish arrays ----------
    def publish_arrays(self):
        pa = PoseArray()
        pa.header.frame_id = self.frame
        pa.header.stamp = rospy.Time.now()
        for w in self.waypoints:
            p = Pose(); p.position.x=w["x"]; p.position.y=w["y"]; p.position.z=w["z"]
            pa.poses.append(p)
        self.pub_wps.publish(pa)

        arr = Float32MultiArray()
        data = []
        for o in self.obstacles:
            data += [o["x"], o["y"], o["r"]]
        arr.data = data
        self.pub_obs.publish(arr)
        self.add_waypoint_name()
        self.make_path_marker()

        if self.mode == "plan":
            self.publish_path()

        else:
            pass

    def publish_path(self):
        pa = PoseArray()
        pa.header.frame_id = self.frame
        pa.header.stamp = rospy.Time.now()

        for w in self.waypoints:
            p = Pose()
            p.position.x = w["x"]
            p.position.y = w["y"]
            p.position.z = self.path_z
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.path_pts_pub.publish(pa)
        self.make_path_marker()

        self.waypoint_for_robot_pub.publish(pa)

        

if __name__ == "__main__":
    rospy.init_node("path_ui")
    PathUI()
    rospy.spin()
