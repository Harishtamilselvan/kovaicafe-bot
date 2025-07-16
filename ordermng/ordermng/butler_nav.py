# ROS 2 Navigation Task (Milestone 12 - Handle Single/Last Cancel Correctly)
# by HARISH T - Ensures robot returns to kitchen if only or last table is cancelled

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time
import threading
import tkinter as tk

class ButlerNav(Node):
    def __init__(self):
        super().__init__('butler_navigation_task')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Navigation stack is active.")

        self.home_pose = self.create_pose(0.0, 0.0, 0.0)
        self.kitchen_pose = self.create_pose(0.7522, -4.3628, 0.0)
        self.table1_pose = self.create_pose(5.0911, 0.8884, 0.0)
        self.table2_pose = self.create_pose(4.9230, 5.1587, 1.57)
        self.table3_pose = self.create_pose(-3.7633, 2.6452, -1.57)

        self.food_ready = False
        self.selected_tables = []
        self.confirmation_flags = {"table1": False, "table2": False, "table3": False}
        self.food_retaken = False
        self.kitchen_wait_triggered = False
        self.from_start_delivery = False
        self.cancel_flags = {"kitchen": False, "table1": False, "table2": False, "table3": False}
        self.status_message = "Idle"

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def update_status(self, msg):
        self.status_message = msg
        print(f"[STATUS] {msg}")

    def safe_go_to(self, pose, label=""):
        if self.cancel_flags.get(label, False):
            self.get_logger().warn(f"[Cancelled] Navigation to {label} aborted before starting.")
            if label == "kitchen":
                self.update_status("Kitchen cancelled. Returning home.")
                self.navigator.cancelTask()
                self.navigator.goToPose(self.home_pose)
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)
                return "cancelled"
            return "cancelled"

        self.navigator.goToPose(pose)
        self.update_status(f"Navigating to {label}")
        while not self.navigator.isTaskComplete():
            if self.cancel_flags.get(label, False):
                self.get_logger().warn(f"[Cancelled] Aborting navigation to {label}.")
                self.navigator.cancelTask()
                if label == "kitchen":
                    self.update_status("Kitchen cancelled. Returning home.")
                    self.navigator.goToPose(self.home_pose)
                    while not self.navigator.isTaskComplete():
                        time.sleep(0.5)
                return "cancelled"
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"[Going to {label}] Distance: {feedback.distance_remaining:.2f} m")
            time.sleep(1.0)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"[Arrived at {label}] Navigation succeeded.")

            if label == "kitchen":
                if self.from_start_delivery:
                    if not self.kitchen_wait_triggered:
                        self.update_status("Waiting in kitchen (start delivery)")
                        while not self.kitchen_wait_triggered and not self.cancel_flags[label]:
                            time.sleep(1.0)
                        if self.cancel_flags[label]:
                            return "cancelled"
                    else:
                        self.get_logger().info("[Start Delivery] Wait already pressed. Proceeding.")
                else:
                    self.update_status("Waiting in kitchen (manual call)")
                    wait_start = time.time()
                    while time.time() - wait_start < 20:
                        if self.kitchen_wait_triggered or self.cancel_flags[label]:
                            break
                        time.sleep(1.0)
                    if self.cancel_flags[label]:
                        return "cancelled"
                    if not self.kitchen_wait_triggered:
                        self.get_logger().warn("[Kitchen Timeout] No Wait button. Returning Home.")
                        self.safe_go_to(self.home_pose, "home")
                        return False

            if label.startswith("table"):
                self.update_status(f"Waiting for {label} confirmation")
                confirmed = False
                wait_start = time.time()
                while time.time() - wait_start < 20:
                    if self.confirmation_flags[label]:
                        confirmed = True
                        break
                    if self.cancel_flags[label]:
                        return "cancelled"
                    time.sleep(1.0)
                if confirmed:
                    self.get_logger().info(f"[Delivery Confirmed] {label} confirmed receipt.")
                    return True
                else:
                    self.get_logger().warn(f"[Timeout] No confirmation from {label}. Logging as failed.")
                    return "failed"

            return True
        else:
            self.get_logger().warn(f"[Navigation Failed] {label} result: {result}")
            return "failed"

    def execute_delivery(self):
        if not self.food_ready or not self.selected_tables:
            self.get_logger().warn("[System] Cannot deliver. Food not placed or no tables selected.")
            return

        for table in self.selected_tables:
            self.confirmation_flags[table] = False

        self.from_start_delivery = True
        failed_tables = []
        cancelled_tables = []
        successful_deliveries = []

        result = self.safe_go_to(self.kitchen_pose, "kitchen")
        if result == "cancelled" or not result:
            return

        for table in self.selected_tables:
            if self.cancel_flags.get(table, False):
                self.get_logger().info(f"[Cancelled] Skipping delivery to {table}.")
                cancelled_tables.append(table)
                continue
            table_pose = getattr(self, f"{table}_pose")
            result = self.safe_go_to(table_pose, table)
            if result == "failed":
                failed_tables.append(table)
            elif result == "cancelled":
                cancelled_tables.append(table)
            else:
                successful_deliveries.append(table)

        if not successful_deliveries or cancelled_tables or failed_tables:
            self.update_status("Returning to kitchen with unserved items.")
            if not self.safe_go_to(self.kitchen_pose, "kitchen"):
                return
            self.update_status("Waiting for 'Food Retaken'")
            while not self.food_retaken:
                time.sleep(1.0)

        self.update_status("Returning to home.")
        self.safe_go_to(self.home_pose, "home")

        self.food_ready = False
        self.selected_tables.clear()
        self.kitchen_wait_triggered = False
        self.from_start_delivery = False
        self.food_retaken = False
        for key in self.cancel_flags:
            self.cancel_flags[key] = False
        self.update_status("Idle")


def main(args=None):
    rclpy.init(args=args)
    node = ButlerNav()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def launch_gui():
        root = tk.Tk()
        root.title("Kitchen Control Panel")

        status_label = tk.Label(root, text="Status: Idle", fg="blue")
        status_label.pack(pady=5)

        def update_gui_status():
            while rclpy.ok():
                status_label.config(text=f"Status: {node.status_message}")
                time.sleep(1.0)

        def call_robot():
            node.kitchen_wait_triggered = False
            node.from_start_delivery = False
            for key in node.cancel_flags:
                node.cancel_flags[key] = False
            threading.Thread(target=lambda: executor.create_task(node.safe_go_to(node.kitchen_pose, "kitchen"))).start()

        def mark_food_ready():
            node.food_ready = True

        def select_table(table):
            if table not in node.selected_tables:
                node.selected_tables.append(table)

        def confirm_delivery(table):
            node.confirmation_flags[table] = True

        def food_retaken():
            node.food_retaken = True

        def kitchen_wait():
            node.kitchen_wait_triggered = True

        def start_delivery():
            threading.Thread(target=lambda: executor.create_task(node.execute_delivery())).start()

        def cancel_kitchen():
            node.cancel_flags["kitchen"] = True

        def cancel_table(table):
            node.cancel_flags[table] = True

        tk.Button(root, text="Call Robot to Kitchen", command=call_robot).pack()
        tk.Button(root, text="Wait in Kitchen", command=kitchen_wait).pack()
        tk.Button(root, text="Place Food on Robot", command=mark_food_ready).pack()
        tk.Button(root, text="Start Delivery", command=start_delivery).pack()

        for table in ["table1", "table2", "table3"]:
            tk.Button(root, text=f"Deliver to {table.upper()}", command=lambda t=table: select_table(t)).pack()
            tk.Button(root, text=f"Cancel {table.upper()} Order", command=lambda t=table: cancel_table(t)).pack()
            tk.Button(root, text=f"Confirm {table.upper()} Delivery", command=lambda t=table: confirm_delivery(t)).pack()

        tk.Button(root, text="Cancel Kitchen", command=cancel_kitchen).pack()
        tk.Button(root, text="Food Retaken", command=food_retaken).pack()

        threading.Thread(target=update_gui_status, daemon=True).start()
        root.mainloop()

    threading.Thread(target=launch_gui).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
