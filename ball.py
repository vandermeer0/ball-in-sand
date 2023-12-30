import pybullet as p
import pybullet_data
import time
import imageio
import numpy

frames = []

# коэффиценты трения
lateral_friction_coefficient = 0.4 #трение скольжения
rolling_friction_coefficient = 0.4 #трение качения
spinning_friction_coefficient = 0.4 #трение вращения

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.createCollisionShape(p.GEOM_PLANE)
plane = p.createMultiBody(0, plane_id)

# размеры поля песка
width = 25
length = 25
height = 16

# параметры песка
sphere_radius = 0.02
sphere_mass = 0.005
restitution = 0.6

# параметры большой сферы
huge_sphere_radius = 0.4
huge_sphere_mass = 25

red_color = [0.1, 0.85, 0, 1]
black_color = [0, 0, 0, 1]
sand_color = [0.83, 0.58, 0.33, 1]
brown_color = [0.55, 0.27, 0.08, 1]
blue_color = [0, 0.75, 1, 1]

# характеристики сферы в пространстве
huge_sphere_orientation = [0, 0, 0, 1] #ориентация
huge_sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=huge_sphere_radius) #геометрическая форма
huge_sphere_position1 = [0, 0, 3] #первоначальная координата

# создание сферы:
huge_sphere_body = p.createMultiBody(huge_sphere_mass, huge_sphere_collision_shape, basePosition=huge_sphere_position1, baseOrientation=huge_sphere_orientation)
p.changeDynamics(huge_sphere_body, -1, restitution=restitution) #упругость сферы
p.changeVisualShape(huge_sphere_body, -1, rgbaColor=black_color) #цвет сферы

# cоздание емкости:
box_length = 1.5
box_width = 0.1
box_height = 0.4

# емкость неподвижна - зафиксирована:
mass = 0.0
inertia = [0, 0, 0]

# создание емкости:
box1_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length / 2, box_width / 2, box_height / 2])
box2_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_width / 2, (box_length + 0.1)/ 2, box_height / 2])

box1_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box1_id, basePosition=[0, -0.75, box_height / 2], baseOrientation=[0, 0, 0, 1])
p.changeDynamics(box1_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=rolling_friction_coefficient, rollingFriction=spinning_friction_coefficient)
box2_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box1_id, basePosition=[0, 0.75, box_height / 2], baseOrientation=[0, 0, 0, 1])
p.changeDynamics(box2_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=rolling_friction_coefficient, rollingFriction=spinning_friction_coefficient)

box3_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box2_id, basePosition=[-0.8, 0, box_height / 2], baseOrientation=[0, 0, 0, 1])
p.changeDynamics(box3_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=rolling_friction_coefficient, rollingFriction=spinning_friction_coefficient)

box4_body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=box2_id, basePosition=[0.8, 0, box_height / 2], baseOrientation=[0, 0, 0, 1])
p.changeDynamics(box4_body, -1, lateralFriction=lateral_friction_coefficient,
                 spinningFriction=rolling_friction_coefficient, rollingFriction=spinning_friction_coefficient)

p.changeVisualShape(box1_body, -1, rgbaColor=black_color)
p.changeVisualShape(box2_body, -1, rgbaColor=black_color)
p.changeVisualShape(box3_body, -1, rgbaColor=black_color)
p.changeVisualShape(box4_body, -1, rgbaColor=black_color)

# спавн песка:
for x in range(length):
    for y in range(width):
        for z in range(height):
            # подбираем коэффиценты так, чтобы весь песок поместился в коробку
            sphere_position = [z * 0.025 + x * sphere_radius * 2 - 0.7, z * 0.025 + y * sphere_radius * 2 - 0.7, (z + 20 * sphere_radius) * sphere_radius * 2]
            sphere_orientation = [0, 0, 0, 1]
            sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
            sphere_body = p.createMultiBody(sphere_mass, sphere_collision_shape, basePosition=sphere_position, baseOrientation=sphere_orientation)
            p.changeDynamics(plane, -1, restitution=restitution)
            p.changeDynamics(sphere_body, -1, restitution=restitution)
            p.changeDynamics(sphere_body, -1, lateralFriction=lateral_friction_coefficient, spinningFriction=rolling_friction_coefficient, rollingFriction=spinning_friction_coefficient)
            # а теперь чередуем цвета)))))
            if x % 2 == 0:
                p.changeVisualShape(sphere_body, -1, rgbaColor=sand_color)
            else:
                p.changeVisualShape(sphere_body, -1, rgbaColor=sand_color)

# задаем гравитацию
p.setGravity(0, 0, -9.81)

# шаг времени
time_step = 1/240
num_steps = 500

# открываем файл в режиме записи
file = open('total_kek12.txt', 'w')

# считаем кинетическую энергию + записываем ее в файлик + съемка видео
for i in range(num_steps):
    # берем линейную и угловую скорость большого шара (кортеж из трех проекций)
    linear_velocity, angular_velocity = p.getBaseVelocity(huge_sphere_body)
    # момент инерции (кортеж из трех проекций)
    moment_of_inertia = p.getDynamicsInfo(huge_sphere_body, -1)[2]
    # подсчет кин энергии поступательного движения по трем осям суммарно mv^2/2
    linear_ke = 0.5 * huge_sphere_mass * (linear_velocity[0]**2 + linear_velocity[1]**2 + linear_velocity[2]**2)
    angular_velocity = (float(angular_velocity[0]), float(angular_velocity[1]), float(angular_velocity[2]))
    # подсчет кин энергии вращения по трем осям суммарно Iw^2/2
    angular_ke = 0.5 * numpy.sqrt(moment_of_inertia[0]**2 + moment_of_inertia[1]**2 + moment_of_inertia[2]**2) * (angular_velocity[0]**2 + angular_velocity[1]**2 + angular_velocity[2] ** 2)
    # полная кин энергия
    total_ke = linear_ke + angular_ke
    # определение положения тела в пространстве
    position, orientation = p.getBasePositionAndOrientation(huge_sphere_body)
    # извлечение высоты
    sphere_height = position[2]
    # подсчёт времени
    time_now = num_steps * 2 * time_step
    # запись в файлик))))
    file.write(f'{round(total_ke, 3)} {round(sphere_height, 3)} {time_now} ')
    print(f"Kinetic Energy {huge_sphere_body}: {total_ke}")
    # съемка видео))))
    p.stepSimulation()
    time.sleep(time_step)
    frame = p.getCameraImage(1920, 1088)
    img = frame[2]
    frames.append(img)


# Закрываем файл
file.close()

# сохрание видео)
imageio.mimsave('output12.mp4', frames, fps=60)
p.disconnect()

# обязательно проверьте сохранились ли видео и ПЕРЕИМЕНУЙТЕ ВИДЕО!!!
