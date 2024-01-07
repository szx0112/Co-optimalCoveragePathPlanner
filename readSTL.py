import numpy as np
from stl import mesh
from matplotlib import pyplot as plt
from math import *
import copy

def checkSTL_obj(obj, norm):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(0, len(obj)):
        v1 = obj[i][0:3]
        v2 = obj[i][3:6]
        v3 = obj[i][6:9]
        vc = (np.array(v1) + np.array(v2) + np.array(v3)) / 3
        vn = list(vc + 0.5 * np.array(norm[i]))
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], 'g')
        ax.plot([v1[0], v3[0]], [v1[1], v3[1]], [v1[2], v3[2]], 'g')
        ax.plot([v2[0], v3[0]], [v2[1], v3[1]], [v2[2], v3[2]], 'g')
        ax.plot([vc[0], vn[0]], [vc[1], vn[1]], [vc[2], vn[2]], 'k')
    plt.show()

# Using an existing stl file:
def readSTL(filename):
    mesh_3d = mesh.Mesh.from_file(filename)
    norm = mesh_3d.normals

    # The mesh points
    tria = mesh_3d.points
    tria_list = copy.copy(tria)
    flat_x = sum([[tr[0], tr[3], tr[6]] for tr in tria], [])
    flat_y = sum([[tr[1], tr[4], tr[7]] for tr in tria], [])
    flat_z = sum([[tr[2], tr[5], tr[8]] for tr in tria], [])
    x_min = min(flat_x)
    x_max = max(flat_x)
    y_min = min(flat_y)
    y_max = max(flat_y)
    z_min = min(flat_z)
    z_max = max(flat_z)

    norm_list = copy.copy(norm)
    obs_range = [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
    for i in range(len(norm)):
        v1 = tria[i][0:3]
        v2 = tria[i][3:6]
        v3 = tria[i][6:9]

        vec_12 = np.array(v2) - np.array(v1)
        vec_13 = np.array(v3) - np.array(v1)

        norm_ = np.cross(vec_12, vec_13)

        if np.all(norm_ == 0):
           np.delete(tria_list, i)
           np.delete(norm_list, i)
        else:
            norm_list[i] = norm_ / np.array((sqrt(norm_[0]**2 + norm_[1]**2 + norm_[2]**2)))

    return tria, norm_list, obs_range


def checkSTL(filename):
 fig = plt.figure()
 ax = fig.add_subplot(111, projection='3d')
 obj, norm, obs_range = readSTL(filename)
 for i in range(0,len(obj)):
     v1 = obj[i][0:3]
     v2 = obj[i][3:6]
     v3 = obj[i][6:9]
     vc = (np.array(v1) + np.array(v2) + np.array(v3))/3
     vn = list(vc + 0.5*np.array(norm[i]))
     ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], 'g')
     ax.plot([v1[0], v3[0]], [v1[1], v3[1]], [v1[2], v3[2]], 'g')
     ax.plot([v2[0], v3[0]], [v2[1], v3[1]], [v2[2], v3[2]], 'g')
     ax.plot([vc[0], vn[0]], [vc[1], vn[1]], [vc[2], vn[2]], 'k')
 plt.show()


