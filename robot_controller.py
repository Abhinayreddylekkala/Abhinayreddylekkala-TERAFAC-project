import requests
import time
import numpy as np
import matplotlib.pyplot as plt
import random # Import the random library

# --- Configuration & TUNING PARAMETERS ---
SERVER_URL = "http://127.0.0.1:5000"

# These values have been tuned for the final strategy
FORWARD_SPEED = 25.0      # A good, steady forward speed
TURN_SPEED = 25.0         # How sharply the robot turns to find the goal
AVOIDANCE_BACKUP = -25.0  # Back up further to get clear

# --- API Helper Functions ---
def set_goal(corner_name):
    """Tells the simulator to place a goal in a corner."""
    print(f"Setting new goal in corner: {corner_name}")
    try:
        requests.post(f"{SERVER_URL}/goal", json={"corner": corner_name})
    except Exception as e:
        print(f"Error setting goal: {e}")

def reset_simulation():
    """Resets the robot and the collision count on the server."""
    print("Resetting simulation and collision count...")
    try:
        requests.post(f"{SERVER_URL}/reset")
    except Exception as e:
        print(f"Error resetting simulation: {e}")

def move_robot_relative(turn_angle, distance):
    """Sends a relative move command."""
    payload = {"turn": float(turn_angle), "distance": float(distance)}
    try:
        requests.post(f"{SERVER_URL}/move_rel", json=payload, timeout=0.1)
    except requests.exceptions.RequestException:
        pass
    except Exception as e:
        print(f"Error moving robot: {e}")

def get_collision_count():
    """Gets the current collision count from the server."""
    try:
        response = requests.get(f"{SERVER_URL}/collisions")
        return response.json().get('count', 0)
    except Exception as e:
        print(f"Error getting collision count: {e}")
        return -1

def set_obstacle_motion(enabled, speed=0.05):
    """Enables or disables obstacle motion."""
    print(f"Setting obstacle motion: Enabled={enabled}, Speed={speed}")
    payload = {"enabled": enabled, "speed": speed}
    try:
        requests.post(f"{SERVER_URL}/obstacles/motion", json=payload)
    except Exception as e:
        print(f"Error setting obstacle motion: {e}")

# --- Main Autonomous Run Function (FINAL STATE MACHINE LOGIC) ---
def run_autonomous_mission(goal_corner):
    """Completes one full run using a state machine with randomized escape."""
    reset_simulation()
    set_goal(goal_corner)
    time.sleep(1)

    target_headings = {"NE": 45, "NW": 315, "SE": 135, "SW": 225}
    target_heading = target_headings[goal_corner]

    current_heading = 0.0

    run_duration_seconds = 90
    start_time = time.time()
    last_collision_count = get_collision_count()

    while time.time() - start_time < run_duration_seconds:
        current_collision_count = get_collision_count()

        if current_collision_count > last_collision_count:
            # --- COLLISION DETECTED: Execute Randomized Escape ---
            print(f"Collision! Executing randomized escape maneuver.")
            
            # 1. Back up to get clear
            move_robot_relative(turn_angle=0, distance=AVOIDANCE_BACKUP)
            time.sleep(0.4)
            
            # 2. Perform a large, unpredictable turn
            random_turn = random.uniform(120, 180)
            print(f"Performing a random {random_turn:.1f} degree turn.")
            move_robot_relative(turn_angle=random_turn, distance=0)
            current_heading = (current_heading + random_turn) % 360
            time.sleep(0.4)

            # 3. Move forward a little to fully escape the trap
            move_robot_relative(turn_angle=0, distance=FORWARD_SPEED)
            time.sleep(0.4)
        
        else:
            # --- NO COLLISION: Actively seek the goal ---
            heading_error = (target_heading - current_heading + 180) % 360 - 180
            turn_correction = np.clip(heading_error, -TURN_SPEED, TURN_SPEED)
            
            print(f"SEEKING... Heading: {current_heading:.1f}, Target: {target_heading}, Turning: {turn_correction:.1f}")
            move_robot_relative(turn_angle=turn_correction, distance=FORWARD_SPEED)
            current_heading = (current_heading + turn_correction) % 360
        
        last_collision_count = get_collision_count()
        time.sleep(0.3)

    move_robot_relative(0, 0)
    final_collisions = get_collision_count()
    print(f"Mission to {goal_corner} finished. Collisions: {final_collisions}")
    return final_collisions

# --- Challenge Level Functions ---
def run_level_1():
    print("\n--- Starting LEVEL 1: Static Obstacles ---")
    goal_corners = ["NE", "NW", "SE", "SW"]
    total_collisions = 0
    set_obstacle_motion(enabled=False)
    for i, corner in enumerate(goal_corners):
        print(f"\n--- Starting Run {i+1}/4: Goal = {corner} ---")
        collisions_for_run = run_autonomous_mission(corner)
        if collisions_for_run != -1:
            total_collisions += collisions_for_run
    average_collisions = total_collisions / len(goal_corners)
    print(f"\n--- LEVEL 1 COMPLETE ---\nAverage collisions: {average_collisions:.2f}\n------------------------")

def run_level_2():
    print("\n--- Starting LEVEL 2: Moving Obstacles ---")
    goal_corners = ["NE", "NW", "SE", "SW"]
    total_collisions = 0
    set_obstacle_motion(enabled=True, speed=0.05)
    for i, corner in enumerate(goal_corners):
        print(f"\n--- Starting Run {i+1}/4: Goal = {corner} ---")
        collisions_for_run = run_autonomous_mission(corner)
        if collisions_for_run != -1:
            total_collisions += collisions_for_run
    average_collisions = total_collisions / len(goal_corners)
    print(f"\n--- LEVEL 2 COMPLETE ---\nAverage collisions: {average_collisions:.2f}\n------------------------")

def run_level_3():
    print("\n--- Starting LEVEL 3: Speed vs. Collisions Analysis ---")
    obstacle_speeds = [0.02, 0.04, 0.06, 0.08, 0.10]
    average_collisions_per_speed = []
    goal_corners = ["NE", "NW", "SE", "SW"]
    for speed in obstacle_speeds:
        print(f"\n--- Testing Obstacle Speed: {speed:.2f} ---")
        set_obstacle_motion(enabled=True, speed=speed)
        total_collisions_for_speed = 0
        for corner in goal_corners:
            collisions_for_run = run_autonomous_mission(corner)
            if collisions_for_run != -1:
                total_collisions_for_speed += collisions_for_run
        avg_for_speed = total_collisions_for_speed / len(goal_corners)
        average_collisions_per_speed.append(avg_for_speed)
        print(f"Average collisions at speed {speed:.2f}: {avg_for_speed:.2f}")
    print("\n--- LEVEL 3 COMPLETE ---")
    plt.figure(figsize=(10, 6))
    plt.plot(obstacle_speeds, average_collisions_per_speed, marker='o', linestyle='-')
    plt.title('Obstacle Speed vs. Average Collisions')
    plt.xlabel('Obstacle Speed')
    plt.ylabel('Average Number of Collisions')
    plt.grid(True)
    plt.savefig('speed_vs_collisions_graph.png')
    print("Graph saved as 'speed_vs_collisions_graph.png'")
    plt.show()

# --- Main Execution ---
if __name__ == "__main__":
    # CHOOSE WHICH CHALLENGE TO RUN: "LEVEL_1", "LEVEL_2", or "LEVEL_3"
    CHALLENGE_TO_RUN = "LEVEL_1"

    if CHALLENGE_TO_RUN == "LEVEL_1":
        run_level_1()
    elif CHALLENGE_TO_RUN == "LEVEL_2":
        run_level_2()
    elif CHALLENGE_TO_RUN == "LEVEL_3":
        run_level_3()
    else:
        print("Invalid challenge selected. Please choose 'LEVEL_1', 'LEVEL_2', or 'LEVEL_3'.")
