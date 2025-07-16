# kovaicafe-bot
the kovaicafe_bot is butler bot for managing orders in the cafe autonomously which is safe and reliable

Butler Navigation System Documentation:-

Overview:-
 
The Butler Navigation System is a ROS 2-based autonomous delivery framework designed to simulate a smart restaurant butler robot. It supports multi-order food delivery operations, offering a dynamic, responsive interaction model for real-world hospitality environments.

Features:-

    • Dynamic multi-table delivery
    • Live order cancellation and confirmation workflows
    • Intelligent wait logic at the kitchen and tables
    • GUI-based task management and monitoring
    • Robust error handling and return logic

Functional Description:-

    The system leverages ROS 2 Humble and Nav2 Simple Commander API to perform the following key functions:
    1. Navigate to Kitchen — Initiated via GUI.
    2. Wait for Food Placement — Supports manual wait or a timeout mechanism.
    3. Table Selection — Allows users to assign one or more delivery destinations.
    4. Autonomous Delivery — Robot visits selected tables sequentially.
    5. Order Confirmation — Waits at each table for user confirmation.
    6. Order Cancellation — At any point, individual deliveries or kitchen phase can be canceled.
    7. Return Protocol — Unconfirmed orders trigger a kitchen return before going home.

Code Walkthrough (Main Logic):-

**butler_nav.py:-

This is the core script that defines and controls robot behavior during the delivery cycle. It includes:

   1. Initialization (``)
   
    • The constructor is the first method called when the node is created.
    • It calls super().__init__('butler_navigation_task') to initialize the ROS node.
    • Sets up the BasicNavigator from Nav2 Simple Commander, which abstracts away complex action server calls for goal navigation.
    • It initializes all important goal poses (home, kitchen, table1, table2, table3) using create_pose().
    • Defines all internal control flags like selected_tables, food_ready, confirmation_flags, cancel_flags, and status_message, which hold the robot’s dynamic state.
    • These flags are directly influenced by GUI button presses.
   
  3. Pose Creation (``)
   
    • The method takes in x, y coordinates and yaw (orientation in radians).
    • Converts yaw to a quaternion using quaternion_from_euler() for orientation representation in ROS.
    • Constructs and returns a PoseStamped message, which includes frame ID, timestamp, position, and orientation.
    • These poses are then used in navigator.goToPose() calls for actual motion.
   
   4. Navigation Method (``)
   
    This function encapsulates the process of safe goal execution with cancellation handling, condition-based waiting, and  result interpretation.
    Steps:
    1. Send goal to Nav2 using goToPose(pose).
    2. Continuously monitor the task using isTaskComplete().
    3. Inside the loop, extract real-time feedback with getFeedback().
    4. Check for cancel flags, which may be updated through GUI buttons.
    5. After reaching the destination:
        ◦ If it’s the kitchen:
            ▪ If in delivery mode, wait for the user to press “Wait in Kitchen”.
            ▪ If called manually, wait for up to 20 seconds.
        ◦ If it’s a table:
            ▪ Waits for confirmation flag for up to 20 seconds.
            ▪ If confirmed: success; if not: logs as timeout/failure.
    6. Returns status as a string: 'arrived', 'cancelled', or 'timeout'.
    This modular method ensures that every movement has clear outcome handling.

6. Main Delivery Loop (``)





   
       This is the delivery execution thread which is started once “Start Delivery” is pressed.
       Workflow:
   
       1. Pre-checks
        ◦ Verifies food_ready and selected_tables are set.
       2. Step 1: Go to Kitchen
        ◦ Calls safe_go_to(self.kitchen_pose, 'kitchen')
        ◦ If cancelled, aborts delivery, goes home.
       3. Step 2: Iterate over tables
        ◦ For every table in selected_tables:
            ▪ Skips if canceled.
            ▪ Calls safe_go_to() and waits for confirmation.
            ▪ Logs successful, failed, and cancelled tables.
       4. Step 3: Handle Errors
        ◦ If any orders failed or were canceled:
            ▪ Return to kitchen.
            ▪ Wait for food_retaken flag from GUI.
       5. Step 4: Return Home
        ◦ Navigates to home position.
        ◦ Updates status to “Returning Home…”
       6. Step 5: Reset Flags
        ◦ Clears all selected tables, flags, confirmations, and cancellations.
        ◦ Sets status_message = "Idle".
        
Multithreaded: This function runs in a background thread so that GUI stays responsive and does not block input or crash.

 
GUI Description:-

    Built with Python’s built-in tkinter, the graphical interface provides:
    • Call Robot to Kitchen — Triggers kitchen visit
    • Wait in Kitchen — Lets the robot know food is being placed
    • Place Food — Flags the system that food is ready
    • Select Tables (1, 2, 3) — Choose delivery destinations
    • Start Delivery — Initiates the execute_delivery() sequence
    • Cancel Table/Kitchen — Cancels a particular destination mid-process
    • Confirm Order (per table) — Confirms receipt at a specific table
    • Food Retaken — Signals food is placed again for failed orders
    • Live Status Label — Constantly updated with current robot state
Each button directly sets or clears internal flags in the ButlerNav object that influence behavior in real-time.

Delivery Flow Diagram:- 

     [Call Robot to Kitchen]
        ↓
    [Robot Navigates to Kitchen]
        ↓
     Wait in Kitchen (manual or 20s timeout)]
        ↓
    [Food Placed + Table Selection + Start Delivery]
        ↓
    [Robot Visits Each Table in Sequence]
        ↓
    ├── [Order Confirmed] → continue
    ├── [Cancelled]       → skip
    └── [No Confirmation] → mark as failed
        ↓
    [If Any Failed or Cancelled → Return to Kitchen → Wait for Food Retaken]
        ↓
    [Return to Home Base]

How to Run
# Step 1: Launch the covaicafe_bot simulation in gazebo fortress (warning dont use gazebo classic for this pkg)

    ros2 launch kovaicafe_bot launch_sim.launch.py 

<img width="1851" height="1023" alt="Screenshot from 2025-07-16 17-54-34" src="https://github.com/user-attachments/assets/af623ef2-d7e5-4252-8751-113650e7877d" />


# Step 2: Run the localization launch file:-

    ros2 launch kovaicafe_bot localization_launch.py 
 
# Step 3: Run the navigation launch file:-

    ros2 launch kovaicafe_bot navigation_launch.py 

# Step 3: launch rviz2:-
    rviz2
# Step 4: add map and robot model in rviz2 then set the 2d Pose Estimate of the robot 
# Step 5: run the ordermng node:-
    ros2 run  ordermng butler_nav

# Step 5: all set to go now the butler bot delivers order based on the gui instruction 
 


  



 

 

 

Author:-

Harish T
Robotics Engineer 

Last Updated:-

July 16, 2025
