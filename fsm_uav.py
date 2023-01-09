import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time

MAP_SIZE = (10, 10, 10) # m x m x m

N_OBSTACLES = 10
N_UAVS = 4

K_GROUP = 1.5
K_REPULSION = 1.0
K_FRICTION = 0.25
K_OBSTACLES = 1.0

VEL_MAX = 1.0   # m/s
MIN_FORCE = 0.1

# Función generica para graficar puntos en 3D
def plot_points(ax, points, color='b'):
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=color)

# Función que grafica las fuerzas de una key en particular
def plot_forces(ax, uavs, uavs_forces, key, color='b'):
    for id, uav_pose in enumerate(uavs):
        ax.quiver(uav_pose[0], uav_pose[1], uav_pose[2], uavs_forces[id][key][0], uavs_forces[id][key][1], uavs_forces[id][key][2], color=color)

# Generador de puntos tridimensionales aleatorios
def generate_points(n):
    points = np.random.rand(n, 3) * MAP_SIZE/2  * np.random.choice([-1, 1], size=(n, 3))
    return points

# Calculo del centro de masas 
def center_of_mass(points):
    return np.mean(points, axis=0)

# Funcion que calcula la fuerza de grupo hacia el centro de masas
def group_forces(uavs, cm):
    for id, uav_pose in enumerate(uavs):
        dir = cm - uav_pose
        dir = dir / np.linalg.norm(dir)
        dist = np.linalg.norm(cm - uav_pose)
        if dist < 0.1:
            uavs_forces[id]['f_group'] = np.zeros(3)
        else:
            uavs_forces[id]['f_group'] = K_GROUP * dir

# Función que calcula la fuerza de repulsión al resto de los UAVs y al centro de masas
def repulsion_forces(uavs, cm):
    B = 2.0
    for id1, uav_pose1 in enumerate(uavs):
        f_repulsion = np.zeros(3)
        for id2, uav_pose2 in enumerate(uavs):
            if id1 == id2:
                continue
            
            dir = uav_pose2 - uav_pose1
            dir = dir / np.linalg.norm(dir)
            dist = np.linalg.norm(uav_pose2 - uav_pose1)
            f_repulsion += - dir / math.pow(dist,B)

        dir = cm - uav_pose1
        dir = dir / np.linalg.norm(dir)
        dist = np.linalg.norm(cm - uav_pose1)
        f_repulsion +=  - dir / math.pow(dist,B)
        uavs_forces[id1]['f_repulsion'] = K_REPULSION * f_repulsion

# Función que calcula la fuerza de fricción, constante y contraria a la velocidad
def friction_forces(uavs, uavs_velocities):
    for id, uav_pose in enumerate(uavs):
        if np.linalg.norm(uavs_velocities[id]) < 0.05:
            uavs_forces[id]['f_friction'] = np.zeros(3)
        else:
            uavs_forces[id]['f_friction'] = - K_FRICTION * uavs_velocities[id]/np.linalg.norm(uavs_velocities[id])

# Función que calcula la fuerza de repulsión hacia los obstáculos
def obstacles_forces(uavs, obstacles):
    B = 4.0
    for id, uav_pose in enumerate(uavs):
        f_obstacles = np.zeros(3)
        for obstacle in obstacles:
            dir = obstacle - uav_pose
            dir = dir / np.linalg.norm(dir)
            dist = np.linalg.norm(obstacle - uav_pose)
            f_obstacles += - dir / math.pow(dist,B)
        uavs_forces[id]['f_obstacles'] = K_OBSTACLES * f_obstacles

# Función que calcula la fuerza resultante a aplicar a cada UAV
def resultant_forces(uavs_forces, forces_keys):
    for id, uav_forces in enumerate(uavs_forces):
        f_resultant = np.zeros(3)
        for key in forces_keys:
            if key != 'f_friction':
                f_resultant += uav_forces[key]
        if np.linalg.norm(f_resultant) < MIN_FORCE:
            f_resultant = np.zeros(3)
        
        f_resultant += uav_forces['f_friction']
        uavs_forces[id]['f_resultant'] = f_resultant

# Aplicamos las fuerzas a los UAVs
def apply_forces(uavs, uavs_forces, dt):
    for id, uav_pose in enumerate(uavs):
        uavs_velocities[id] = uavs_velocities[id] + uavs_forces[id]['f_resultant'] * dt
        if np.linalg.norm(uavs_velocities[id]) > VEL_MAX:
            uavs_velocities[id] = uavs_velocities[id]/np.linalg.norm(uavs_velocities[id]) * VEL_MAX

        uavs[id] = uavs[id] + uavs_velocities[id] * dt

##############################################
uavs = generate_points(N_UAVS)
obstacles = generate_points(N_OBSTACLES)

uavs_velocities = [np.zeros(3) for _ in range(len(uavs))]
uavs_forces = [{} for i in range(len(uavs))]
forces_keys = ['f_group','f_repulsion','f_friction','f_obstacles']
dt = 0.1
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Descomentar para añadir 4 obstáculos alrededor del origen
# origin = np.array([0.0,0.0,0.0])
# obstacles = []
# obstacles.append(origin + np.array([0.0,1,0.0]))
# obstacles.append(origin + np.array([0.0,-1,0.0]))
# obstacles.append(origin + np.array([1,0.0,0.0]))
# obstacles.append(origin + np.array([-1,0.0,0.0]))
# obstacles = np.array(obstacles)

i = 0
while True:
    ax.clear()

    # Sustituir por un punto de interés si se desea
    cm = center_of_mass(uavs)

    # Descomentar para mover el punto de interés 
    # Nota: sacar fuera de bucle la inicialización de cm
    # if i < 200:
    #     cm += np.array([0.025,0.00,0.00])

    group_forces(uavs, cm)
    repulsion_forces(uavs, cm)
    friction_forces(uavs, uavs_velocities)
    obstacles_forces(uavs, obstacles)
    resultant_forces(uavs_forces, forces_keys)
    plot_points(ax, uavs, color='b')
    plot_points(ax, np.array([cm]), color='r')
    plot_points(ax, obstacles, color='purple')
    plot_forces(ax, uavs, uavs_forces, 'f_repulsion', color='r')
    plot_forces(ax, uavs, uavs_forces, 'f_group', color='g')
    plot_forces(ax, uavs, uavs_forces, 'f_obstacles', color='purple')
    plot_forces(ax, uavs, uavs_forces, 'f_resultant', color='k')
    ax.set_xlim(-0.5*MAP_SIZE[0], 0.5*MAP_SIZE[0])
    ax.set_ylim(-0.5*MAP_SIZE[1], 0.5*MAP_SIZE[1])
    ax.set_zlim(-0.5*MAP_SIZE[2], 0.5*MAP_SIZE[2])
    apply_forces(uavs, uavs_forces, dt)

    # Leyenda
    ax.scatter([], [], c='b', label='UAVs')
    ax.scatter([], [], c='r', label='TARGET Center of mass')
    ax.scatter([], [], c='purple', label='Obstacles')
    ax.quiver([], [], [], [], [], [], color='r', label='Repulsion force')
    ax.quiver([], [], [], [], [], [], color='g', label='Group force')
    ax.quiver([], [], [], [], [], [], color='purple', label='Obstacles force')
    ax.quiver([], [], [], [], [], [], color='k', label='Resultant force')
    ax.legend()

    plt.pause(dt)

    if plt.get_fignums() == []:
        break

    i+=1


    


    
    

