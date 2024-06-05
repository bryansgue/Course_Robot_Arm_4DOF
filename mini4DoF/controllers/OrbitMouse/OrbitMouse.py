"""OrbitMouse controller."""

from controller import Robot
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Status
status = 0

# u, v 2D mouse position
u_now = 0
u_old = 0
v_now = 0
v_old = 0

# u, v factors & max value
u_fac = 0
v_fac = 0
max_v = 4

# Keyboard 
key = -1

# Initialize Motors
m1_continuous = robot.getDevice('m1_continuous')
m1_continuous.setPosition(float('inf'))
m1_s = 1
m1_d = 0
# m1_continuous.setVelocity(m1_s * m1_d)

m2 = robot.getDevice('m2')
m2.setPosition(float('inf'))
m2_s = 1
m2_d = 0
# m2.setVelocity(m2_s * m2_d)

m3 = robot.getDevice('m3')
m3.setPosition(float('inf'))
m3_s = 1
m3_d = 0
# m3.setVelocity(m3_s * m3_d)

m4 = robot.getDevice('m4')
m4.setPosition(float('inf'))
m4_s = 1
m4_d = 0
# m4.setVelocity(m4_s * m4_d)

m5 = robot.getDevice('m5')
m5.setPosition(float('inf'))
m5_s = 1
m5_d = 0
# m5.setVelocity(m5_s * m5_d)

# Initialize Sensors
m1_p = robot.getDevice('m1_continuous_sensor')
m1_p.enable(timestep)

m2_p = robot.getDevice('m2_sensor')
m2_p.enable(timestep)

m3_p = robot.getDevice('m3_sensor')
m3_p.enable(timestep)

m4_p = robot.getDevice('m4_sensor')
m4_p.enable(timestep)

m5_p = robot.getDevice('m5_sensor')
m5_p.enable(timestep)

# camera enable
# camera = robot.getDevice('camera')
# camera.enable(timestep*2)

# keyboard enable
robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

# mouse enable
robot.mouse.enable(timestep)
robot.mouse.enable3dPosition()

# message for human in the loop
print("Hello, this is a Demo for controlling the miniRobot Arm using the keyboard & mouse \n")
print("Select the main window and use the keyboard:\n")
print("z + Drag mouse horizontal = move the 1 Dof\n")
print("z + Drag mouse vertical   = move the 2 Dof\n")
print("x + Drag mouse vertical   = move the 3 Dof\n")
print("c + Drag mouse vertical   = move the 4 Dof\n")
print("c + Drag mouse horizontal = move the gripper\n")

# Main loop:
while robot.step(timestep) != -1:

    # Get sensors value
    m1a_p = m1_p.getValue()
    m2a_p = m2_p.getValue()
    m3a_p = m3_p.getValue()
    m4a_p = m4_p.getValue()
    m5a_p = m5_p.getValue()
    
    # Get pressed key
    key = robot.keyboard.getKey()

    # Get mouse state
    mouse_state = robot.mouse.getState()
    
    # u, v factor estimation
    if mouse_state.left == 0 and mouse_state.middle == 0 and mouse_state.right == 0:
        u_now = mouse_state.u
        v_now = mouse_state.v
        if math.isnan(u_now):
            u_now = 0
            v_now = 0
            
        if u_now != u_old:
            u_fac = 60 * (u_now - u_old)
            if u_fac > max_v:
                u_fac = max_v
            if u_fac < -max_v:
                u_fac = -max_v
            u_old = u_now
        else:
            u_fac = 0
        if v_now != v_old:
            v_fac = 60 * (v_now - v_old)
            if v_fac > max_v:
                v_fac = max_v
            if v_fac < -max_v:
                v_fac = -max_v
            v_old = v_now
        else:
            v_fac = 0
    else:
        u_old = u_now
        v_old = v_now
        
    # Reset Status
    if status == 0:
        if m2a_p > -0.6:
            m2_d = -1
        else:
            m2_d = 0
            status = 1

        if m3a_p < 0.663225:
            m3_d = 1
        else:
            m3_d = 0        

    # Manual Status
    if status == 1:

        # z key waist & shoulder
        if key == 90:
            m1_d = -u_fac
            m2_d = -v_fac
            if (m1_d > 0 and m1a_p > 3.141593) or (m1_d < 0 and m1a_p < -3.141593):
                m1_d = 0
            if (m2_d > 0 and m2a_p > 3.141593) or (m2_d < 0 and m2a_p < -3.141593):
                m2_d = 0
        else:
             m1_d = 0
             m2_d = 0

        # x key elbow
        if key == 88:
            m3_d = -v_fac
            if (m3_d > 0 and m3a_p > 3.141593) or (m3_d < 0 and m3a_p < -3.141593):
                m3_d = 0
        else:
             m3_d = 0
       
        # c key wrist
        if key == 67:
            m4_d = -v_fac
            if (m4_d > 0 and m4a_p > 3.141593) or (m4_d < 0 and m4a_p < -3.141593):
                m4_d = 0
            m5_d = u_fac
            if (m5_d > 0 and m5a_p > 3.141593) or (m5_d < 0 and m5a_p < -3.141593):
                m5_d = 0
        else:
             m4_d = 0
             m5_d = 0
             
    # Setting velocity of every motor        
    m1_continuous.setVelocity(m1_s * m1_d)
    m2.setVelocity(m2_s * m2_d)
    m3.setVelocity(m3_s * m3_d)
    m4.setVelocity(m4_s * m4_d)
    m5.setVelocity(m5_s * m5_d)

    pass

