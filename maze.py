import pybullet as p
import pybullet_data
import time
import imageio

frames = []

lateral_friction_coefficient = 0.5
rolling_friction_coefficient = 0.5
spinning_friction_coefficient = 0.5


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.createCollisionShape(p.GEOM_PLANE)
plane = p.createMultiBody(0, plane_id)

sphere_radius = 0.1
sphere_mass = 1
restitution = 0.6

yellow_color = [1, 1, 0, 1]
blue_color = [0, 0, 1, 1]

mass = 0.0
inertia = [0, 0, 0]

box1_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 3, 3.5])
box2_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 3, 3.5])
box3_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 3])
box4_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 3.5])
box5_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 0.5])
box6_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 0.5])
box7_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 0.5])
box8_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 0.5])

box9_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 2.5])
box10_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 2.5])
box11_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.5, 2.5])
box12_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 2.5])




box1_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box1_id, basePosition=[0.5, 0, 3.5], baseOrientation=[0, 0, 0, 1])
box2_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box2_id, basePosition=[2.5, 0, 3.5], baseOrientation=[0, 0, 0, 1])
box3_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box3_id, basePosition=[1.5, -2.5, 4], baseOrientation=[0, 0, 0, 1])
box4_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box4_id, basePosition=[1.5, 2.5, 3.5], baseOrientation=[0, 0, 0, 1])
box5_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box5_id, basePosition=[1.5, -0.5, 6.5], baseOrientation=[0, 0, 0, 1])
box6_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box6_id, basePosition=[1.5, 0.5, 4.5], baseOrientation=[0, 0, 0, 1])
box7_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box7_id, basePosition=[1.5, -0.5, 2.5], baseOrientation=[0, 0, 0, 1])
box8_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box8_id, basePosition=[1.5, 0.5, 0.5], baseOrientation=[0, 0, 0, 1])

box9_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box9_id, basePosition=[0.5, 1.5, 9.5], baseOrientation=[0, 0, 0, 1])
box10_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box10_id, basePosition=[1.5, 0.5, 9.5], baseOrientation=[0, 0, 0, 1])
box11_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box11_id, basePosition=[2.5, 1.5, 9.5], baseOrientation=[0, 0, 0, 1])
box12_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box12_id, basePosition=[1.5, 2.5, 9.5], baseOrientation=[0, 0, 0, 1])

p.changeDynamics(box1_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box2_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box3_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box4_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box5_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=spinning_friction_coefficient)
p.changeDynamics(box6_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box7_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box8_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box9_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box10_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box11_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)
p.changeDynamics(box12_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=spinning_friction_coefficient, rollingFriction=rolling_friction_coefficient)


p.changeVisualShape(box3_body, -1, rgbaColor=yellow_color)
p.changeVisualShape(box4_body, -1, rgbaColor=yellow_color)
p.changeVisualShape(box5_body, -1, rgbaColor=yellow_color)
p.changeVisualShape(box6_body, -1, rgbaColor=yellow_color)
p.changeVisualShape(box7_body, -1, rgbaColor=yellow_color)
p.changeVisualShape(box8_body, -1, rgbaColor=yellow_color)


p.changeVisualShape(box1_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])
p.changeVisualShape(box2_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])
p.changeVisualShape(box9_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])
p.changeVisualShape(box10_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])
p.changeVisualShape(box11_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])
p.changeVisualShape(box12_body, -1, rgbaColor=[1.0, 1.0, 1.0, 0.0])

for x in range(5):
    for y in range(5):
        for z in range(20):
            sphere_position = [1 + x * sphere_radius * 2, 1 + y * sphere_radius * 2, 7 + z * sphere_radius * 2]
            sphere_orientation = [0, 0, 0, 1]
            sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
            sphere_body = p.createMultiBody(sphere_mass, sphere_collision_shape, basePosition=sphere_position, baseOrientation=sphere_orientation)
            p.changeDynamics(sphere_body, -1, lateralFriction=lateral_friction_coefficient,
                             spinningFriction=spinning_friction_coefficient,
                             rollingFriction=rolling_friction_coefficient)

            p.changeDynamics(plane, -1, restitution=restitution)
            p.changeDynamics(sphere_body, -1, restitution=restitution)
            p.changeVisualShape(sphere_body, -1, rgbaColor=blue_color)

p.setGravity(0, 0, -9.81)

time_step = 1 / 240
num_steps = 2500

for _ in range(num_steps):
    p.stepSimulation()
    time.sleep(time_step)
    frame = p.getCameraImage(640, 480)
    img = frame[2]
    frames.append(img)
imageio.mimsave('output4.mp4', frames, fps=30)
p.disconnect()
