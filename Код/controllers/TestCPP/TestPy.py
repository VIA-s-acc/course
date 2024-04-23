# Import necessary libraries
import time
from webots import robot, distance_sensor, motor

# Define wheel motors (assuming they're named "left wheel motor" and "right wheel motor")
LEFT_WHEEL_MOTOR = "left wheel motor"
RIGHT_WHEEL_MOTOR = "right wheel motor"

# Define time step in milliseconds
TIME_STEP = 64

# Function to calculate wheel speeds based on desired linear and angular velocities
def set_wheel_speeds(linear_velocity, angular_velocity):
  left_speed = linear_velocity + (angular_velocity * robot.get_wheel_radius(LEFT_WHEEL_MOTOR)) / 2.0
  right_speed = linear_velocity - (angular_velocity * robot.get_wheel_radius(RIGHT_WHEEL_MOTOR)) / 2.0
  robot.set_position_velocity(LEFT_WHEEL_MOTOR, INFINITY, left_speed)
  robot.set_position_velocity(RIGHT_WHEEL_MOTOR, INFINITY, right_speed)

# Function to calculate the difference between the robot's and goal's positions
def calculate_position_difference(current_position, goal_position):
  x_diff = current_position[0] - goal_position[0]
  y_diff = current_position[1] - goal_position[1]
  return distance_sensor.distance(x_diff, y_diff)  # Use distance sensor for more accurate calculations

# Function to get the goal's position from the scene
def get_goal_position():
  goal_node = robot.get_node("Goal")  # Assuming "Goal" is the node name
  if goal_node:
    return robot.get_node_position(goal_node)
  else:
    print("Error: Goal object not found!")
    return [0, 0, 0]  # Default position if goal not found

def main():
  # Initialize Webots
  robot.init(TIME_STEP)

  # Get motor and distance sensor devices
  left_motor = robot.get_device(LEFT_WHEEL_MOTOR)
  right_motor = robot.get_device(RIGHT_WHEEL_MOTOR)
  distance_sensor = robot.get_device("DistanceSensor-IR0")  # Assuming sensor name

  # Enable motor position and velocity control
  left_motor.set_position_control(enable=True)
  right_motor.set_position_control(enable=True)

  # Enable distance sensor
  distance_sensor.enable(TIME_STEP)

  current_position = [0, 0, 0]  # Initialize current position

  while robot.step(TIME_STEP) != -1:
    # Get distance sensor reading
    distance = distance_sensor.get_value()

    # Get current robot position (using distance sensor for more accurate calculations)
    current_position = distance_sensor.estimate_position(current_position)

    # Get goal position
    goal_position = get_goal_position()

    # Calculate distance to goal
    distance_to_goal = calculate_position_difference(current_position, goal_position)

    # Simple proportional control logic (can be replaced with more sophisticated algorithms)
    linear_velocity = 0.2  # Adjust based on desired speed
    angular_velocity = 0.0  # Adjust for turning behavior

    if distance < 0.1:  # Obstacle avoidance threshold (adjust as needed)
      angular_velocity = -1.0  # Turn away from obstacle
    elif distance_to_goal > 0.05:  # Tolerance for reaching goal (adjust as needed)
      # Calculate direction towards goal
      goal_direction = math.atan2(goal_position[1] - current_position[1], goal_position[0] - current_position[0])
      current_direction = robot.get_orientation()[2]  # Assuming Z-axis for orientation
      angular_velocity = -1.0 * (goal_direction - current_direction)  # Turn towards goal

    # Set wheel speeds based on calculated velocities
    set_wheel_speeds(linear_velocity, angular_velocity)

    # Simulate a small delay (optional)
    time.sleep(TIME_STEP / 1000)  # Delay in milliseconds

  # Cleanup
  distance_sensor.disable()
  robot.cleanup()

# Run
