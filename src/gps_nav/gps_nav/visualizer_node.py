"""Plot only accepted estimated poses; optional headless PNG output."""
import math
import os
from threading import RLock
import matplotlib
if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from gps_nav_interfaces.msg import LocalizationStatus, NavigationStatus, RobotControlStatus
from .estimator_node import yaw_from_quaternion
from .trajectory_history import TrajectoryHistory


class VisualizerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('visualizer', **kwargs)
        self.declare_parameter('headless',False)
        self.declare_parameter('output_file','trajectory.png')
        self.declare_parameter('save_on_draw',True)
        self.declare_parameter('mode','simulation')
        self.declare_parameter('draw_period_s',2.0)
        self._state_lock=RLock()
        self.goal=None; self.pose=None; self.history=TrajectoryHistory(); self.status=None; self.nav=None
        self.last_valid_nav=None; self.arrival_seen=False; self.robot_status=None
        self.closed=False; self.draw_history=[]
        self.headless=self.get_parameter('headless').value
        if self.headless: plt.switch_backend('Agg')
        self.fig,self.ax=plt.subplots(figsize=(10,6))
        self.info=self.fig.text(0.70,0.93,'',va='top',fontsize=10)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.create_subscription(PointStamped,'/navigation_goal',self.on_goal,10)
        self.create_subscription(PoseStamped,'/estimated_position',self.on_pose,100)
        self.create_subscription(Path,'/estimated_path',self.on_path,10)
        self.create_subscription(Empty,'/session_reset',self.on_reset,10)
        self.create_subscription(LocalizationStatus,'/localization_status',self.on_status,10)
        self.create_subscription(NavigationStatus,'/navigation_status',self.on_nav,10)
        self.create_subscription(RobotControlStatus,'/robot/control_status',self.on_robot_status,10)
        draw_period = self.get_parameter('draw_period_s').value
        if draw_period > 0:
            self.create_timer(draw_period,self.draw)

    def on_close(self,_):
        if not self.headless:
            self.fig.savefig(self.get_parameter('output_file').value,dpi=120)
        self.closed=True
        rclpy.try_shutdown()

    def on_goal(self,m):
        with self._state_lock:
            new_goal=(m.point.x,m.point.y)
            if self.goal is not None and math.dist(self.goal,new_goal) > 1e-8:
                self.arrival_seen=False
                self.last_valid_nav=None
                self.nav=None
            self.goal=new_goal
    def on_pose(self,m):
        with self._state_lock:
            if self.history.add_pose(m): self.pose=m
    def on_path(self,m):
        with self._state_lock: self.history.merge_path(m)
    def on_reset(self,_):
        now=self.get_clock().now().to_msg()
        with self._state_lock:
            self.history.clear((now.sec,now.nanosec))
            self.pose=None; self.nav=None; self.status=None
            self.last_valid_nav=None; self.arrival_seen=False
    def on_status(self,m):
        with self._state_lock: self.status=m
    def on_robot_status(self,m):
        with self._state_lock: self.robot_status=m
    def on_nav(self,m):
        with self._state_lock:
            self.nav=m
            if m.distance_valid: self.last_valid_nav=m
            self.arrival_seen |= m.destination_reached

    def draw(self, save=None):
        with self._state_lock:
            goal=self.goal; pose=self.pose; status=self.status; nav=self.nav
            last=self.last_valid_nav; arrival_seen=self.arrival_seen
            robot=self.robot_status
            samples=self.history.snapshot()
            xs,ys=self.history.plot_coordinates()
        self.draw_history.append(len(samples))
        ax=self.ax; ax.clear()
        ax.scatter([0],[0],marker='s',label='Session start',color='green')
        if goal: ax.scatter([goal[0]],[goal[1]],marker='*',s=160,label='Destination',color='red')
        if samples:
            ax.plot(xs,ys,label='Estimated trajectory',color='blue')
        if pose:
            p=pose.pose.position; yaw=yaw_from_quaternion(pose.pose.orientation) or 0
            ax.scatter([p.x],[p.y],label='Estimated position',color='navy')
            ax.arrow(p.x,p.y,0.35*math.cos(yaw),0.35*math.sin(yaw),color='navy',width=0.015)
            if goal and math.hypot(goal[0]-p.x,goal[1]-p.y) > 1e-6:
                angle=math.atan2(goal[1]-p.y,goal[0]-p.x)
                ax.arrow(p.x,p.y,0.35*math.cos(angle),0.35*math.sin(angle),color='red',width=0.008)
        lines=[f"Mode: {self.get_parameter('mode').value}",
               f"Last valid distance: {last.distance_m:.2f} m" if last else 'Distance: unavailable',
               f"Last heading error: {math.degrees(last.heading_error_rad):.1f} deg" if last and math.isfinite(last.heading_error_rad) else 'Heading error: unavailable',
               f"Guidance: {nav.recommendation}" if nav else 'Guidance: unavailable',
               f"Arrival event: {'yes' if arrival_seen else 'no'}",
               f"Flow quality: {status.flow_quality}" if status else 'Flow quality: unavailable',
               f"IMU: {'fresh' if status.imu_fresh else 'stale'}" if status else 'IMU: unavailable',
               f"Localization: {'valid' if status.position_valid else status.reason}" if status else 'Localization: unavailable',
               f"Robot control: {'ENABLED' if robot.enabled else 'FAULT' if robot.fault_latched else 'disabled'}" if robot else 'Robot control: unavailable',
               f"Microcontroller: {'connected' if robot.link_connected else 'disconnected'}" if robot else 'Microcontroller: unavailable',
               f"Last ACK: {robot.last_ack_command or 'none'} #{robot.last_ack_sequence}" if robot else 'Last ACK: unavailable',
               f"Control reason: {robot.reason}" if robot else 'Control reason: unavailable']
        self.info.set_text('\n'.join(lines))
        ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
        ax.set_title(f'GPS-denied navigation estimate — {self.get_parameter("mode").value} mode')
        bounds=[(0.0,0.0),*samples.values()]
        if goal: bounds.append(goal)
        if pose: bounds.append((pose.pose.position.x,pose.pose.position.y))
        left=min(p[0] for p in bounds); right=max(p[0] for p in bounds)
        bottom=min(p[1] for p in bounds); top=max(p[1] for p in bounds)
        span=max(right-left,top-bottom,1.0)
        half=span/2+max(0.4,0.08*span)
        ax.set_xlim((left+right)/2-half,(left+right)/2+half)
        ax.set_ylim((bottom+top)/2-half,(bottom+top)/2+half)
        ax.set_aspect('equal',adjustable='box'); ax.grid(True)
        ax.legend(loc='upper left',bbox_to_anchor=(1.02,0.52),fontsize=9)
        self.fig.subplots_adjust(left=0.12,right=0.68,bottom=0.12,top=0.90)
        if (save if save is not None else self.get_parameter('save_on_draw').value):
            self.fig.savefig(self.get_parameter('output_file').value,dpi=120)
        if not self.headless:
            self.fig.canvas.draw_idle(); plt.pause(0.001)

    def destroy_node(self):
        if self.headless:
            self.draw(save=True)
        plt.close(self.fig)
        super().destroy_node()


def main():
    rclpy.init(); node=VisualizerNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    except RuntimeError:
        if rclpy.ok(context=node.context): raise
    finally: node.destroy_node(); rclpy.try_shutdown()
