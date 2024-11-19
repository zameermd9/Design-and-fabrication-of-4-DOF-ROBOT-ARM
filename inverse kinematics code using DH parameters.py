import numpy as np

# Define the DH parameters for a 4-DOF robot (with a gripper)
# DH Parameters: [theta, d, a, alpha]
dh_params = [
    [0, 144.15, 0, np.pi / 2],   # Joint 1
    [0, 243.81, 0, 0],           # Joint 2
    [0, 124.75, 0, 0],           # Joint 3
    [0, 84.75, 0, -np.pi / 2]    # Joint 4 (gripper)
]

# Target position (x, y, z)
target_position = np.array([0.135,-0.181,0.351])  # (x, y, z)

# Extract link lengths from DH parameters
L1 = dh_params[0][1]  # Length of link 1
L2 = dh_params[1][1]  # Length of link 2
L3 = dh_params[2][1]  # Length of link 3
L4 = dh_params[3][1]  # Length of link 4

# Step 1: Calculate theta1 (Base angle)
# theta1 is the angle in the x-y plane
x, y, z = target_position
theta1 = np.arctan2(y, x)  # atan2 ensures correct quadrant

# Step 2: Calculate r and the vertical offset (d)
r = np.sqrt(x**2 + y**2)  # Horizontal projection distance
d = z - L1                # Vertical distance from the base to the end-effector

# Step 3: Use the law of cosines to calculate theta3 (elbow angle)
# Calculate r3 (distance between joints 2, 3, and 4)
r3 = np.sqrt(r**2 + d**2)
cos_theta3 = (r3**2 - L2**2 - L3**2) / (2 * L2 * L3)
cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # Ensure within valid range for arccos
theta3 = np.arccos(cos_theta3)

# Step 4: Calculate theta2 using geometry of the robot
phi = np.arctan2(d, r)  # Angle between r and z axis
psi = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))  # Angle for elbow
theta2 = phi - psi

# Step 5: Calculate theta4 (Gripper orientation)
# Assuming the gripper is aligned with the rest of the arm, theta4 is often a fixed offset or related to other angles.
# For simplicity, we assume theta4 is 0 or based on a specific orientation required.
theta4 = 0  # Set theta4 as 0 for simplicity or adjust for desired orientation

# Printing the calculated joint angles (in radians)
print("Inverse Kinematics Solution (in radians):")
print(f"Theta1: {theta1:.4f} rad")
print(f"Theta2: {theta2:.4f} rad")
print(f"Theta3: {theta3:.4f} rad")
print(f"Theta4: {theta4:.4f} rad")
