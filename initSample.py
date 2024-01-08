from math import *
from random import *
from checkCollision import *
import numpy as np
import math
from params import *

# degree to radiance
def deg2rad(degree):
    return math.radians(degree)

def distance(p0, p1):
    xd = p0[0] - p1[0]
    yd = p0[1] - p1[1]
    zd = p0[2] - p1[2]
    return round(math.sqrt(xd*xd + yd*yd + zd*zd), 2)

# radiance to degree
def rad2deg(radiance):
    return math.degrees(radiance)


def path_length(path):
    dis = 0
    for i in range(len(path)-1):
        p0 = path[i][0:3]
        p1 = path[i+1][0:3]
        dis += distance(p0, p1)
    return dis


# given global coordinates of p_, based on c and n, computes the local coordinate with c as origin, v1 as +x and n as +z.
# input p in global coordinates, return p in local coordinates
def local_coordinates(p, c, n, v1):
    c_ = np.array(c)  # origin of local coordinates
    z_ = np.array(n)  # z axis of local coordinates
    z_ = z_ / np.sqrt(z_.dot(z_))  # normalization of z
    # print 'z_',z_
    v1_ = np.array(v1)
    x_ = v1_ - c_  # x axis of local coordinate
    x_ = x_ / np.sqrt(x_.dot(x_))  # normalization of x
    # print 'x_',x_
    y_ = np.cross(z_, x_)  # y axis
    y_ = y_ / np.sqrt(y_.dot(y_))  # normalization of y
    # print 'y_',y_
    # homogeneous coordinates
    x_ = np.append(x_, 0)
    y_ = np.append(y_, 0)
    z_ = np.append(z_, 0)

    # c_ = -1*c_
    d_ = np.append(c_, 1)
    # print d_
    T = np.array([x_, y_, z_, d_])
    T = np.transpose(T)
    T = np.linalg.inv(T) # inverse of T
    # print 'T',T
    p = np.array(p)  # point p in global coordinates
    p = np.append(p, 1)
    # print p
    # compute p in local coordinates
    p_ = np.dot(T, p)
    # print 'p_',p_
    p_ = p_[0:3]
    p_[0] = round(p_[0], 2)
    p_[1] = round(p_[1], 2)
    p_[2] = round(p_[2], 2)
    return p_
    # return np.dot(T, p_c)

# p is the vertice (x,y,z) in local frame
# return the p in global frame
def global_coordinates(p, c, n, v1):
    c_ = np.array(c)  # origin of local coordinates in global
    z_ = np.array(n)  # z axis of local coordinates in global

    if z_.dot(z_) != 0:
        z_ = z_ / np.sqrt(z_.dot(z_))  # normalization of z
    else:
        print(z_)
        z_ = z_ / np.sqrt(z_.dot(z_))

    v1_ = np.array(v1)  # point in global
    x_ = v1_ - c_  # x axis of local coordinate
    if np.sqrt(x_.dot(x_)) != 0:
        x_ = x_ / np.sqrt(x_.dot(x_))  # normalization of x
    else:
        x_ = np.array([1.0,0.0,0.0])

    y_ = np.cross(z_, x_)  # y axis
    if np.sqrt(y_.dot(y_)) != 0:
        y_ = y_ / np.sqrt(y_.dot(y_))  # normalization of y
    else:
        y_ = np.array([0.0,1.0,0.0])

    x_ = np.append(x_, 0)
    y_ = np.append(y_, 0)
    z_ = np.append(z_, 0)

    # c_ = -1*c_
    d_ = np.append(c_, 1)
    T = np.array([x_, y_, z_, d_])
    T = np.transpose(T)
    # T = np.linalg.inv(T)
    p = np.array(p)  # point p in local coordinates
    p = np.append(p, 1)
    # compute p in global coordinates
    p_ = np.dot(T, p)
    p_ = p_[0:3]
    return p_

# compute norm of triangle based on v1 and v2
# v1 and v2 are two vectors that forms a surface
def surface_norm(v1, v2):
    n = np.cross(v1, v2)
    # length of norm
    ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** (0.5)
    # normalized
    if ln != 0:
        return n / ln
    else:
        print(v1)
        print(v2)
        return n/ln

# rotation matrix
def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


# based on the generate viewpoint and triangle (v1,v2,v3), generate roll,pitch,yaw angles
def get_rpy(vp, v1, v2, v3, n):
    # vp is the location of viewpoint (x,y,z)
    # v1,v2,v3, n are three corners of a triangle and the norm
    vc = (v1 + v2 + v3) / 3
    ray = vc - vp
    ray = ray / math.sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2])
    # yaw follow right hand rule with the world coordinates
    # yaw 0/360 -> x positive: x is the world coordiantes
    # yaw 180 -> x negative:
    # yaw 90 -> y positive
    # yaw 270 -> y negative
    if ray[0] != 0:
        yaw = math.degrees(math.atan(math.fabs(ray[1] / ray[0])))
        if ray[0] > 0 and ray[1] >= 0:
            yaw_ = yaw
        elif ray[0] < 0 and ray[1] < 0:
            yaw_ = yaw + 180.0
        elif ray[0] < 0 and ray[1] >= 0:
            yaw_ = yaw + 90.0
        elif ray[0] > 0 and ray[1] < 0:
            yaw_ = yaw + 270.0
        else:
            yaw_ = 0.0
    else:
        if ray[1] > 0:
            yaw_ = 90.0
        elif ray[1] < 0:
            yaw_ = -90.0
        else:
            yaw_ = 0.0

    # pitch 0 -> along xy plane
    # pitch 30 -> above xy plane
    # pitch -90 -> point downward to the ground
    xy = math.sqrt(ray[0] ** 2 + ray[1] ** 2)
    if xy != 0:
        pitch = math.degrees(math.atan(math.fabs(ray[2] / xy)))
        if ray[2] >= 0:
            pitch_ = pitch
        else:
            pitch_ = -1 * pitch
    else:
        if ray[2] < 0:
            pitch_ = -90.0
        else:
            pitch = 90.0
    wp = np.array([vp[0], vp[1], vp[2], 0, round(pitch_, 2), round(yaw_, 2)])
    return wp, ray


# check if pitch angle is within the range of gimbal
def check_pitch(wp):
    if wp[4] >= GIMBAL_PITCH_UPPER_LIMIT or wp[4] <= GIMBAL_PITCH_LOWER_LIMIT:
        return False
    else:
        return True

# define the search space of each triangle (v1,v2,v3), c is the center of triangle and n is the norm
def search_space(v1, v2, v3, c, n):
    v1_ = local_coordinates(v1, c, n, v1)
    v2_ = local_coordinates(v2, c, n, v1)
    v3_ = local_coordinates(v3, c, n, v1)
    c_ = local_coordinates(c, c, n, v1)

    # magnitude of vectors
    d1 = np.linalg.norm(v1_ - c_)
    d2 = np.linalg.norm(v2_ - c_)
    d3 = np.linalg.norm(v3_ - c_)

    if (d1 >= d2 and d1 >= d3):
        d = d1
    elif (d2 >= d1 and d2 >= d3):
        d = d2
    else:
        d = d3
    x_range = [-1 * d, d]
    y_range = [-1 * d, d]
    z_range = [DISTANCE_MIN, DISTANCE_MAX]
    return x_range, y_range, z_range


# return cosine of view angle between camera and norm, distance between camera and center of triangle
def get_view_angle(p, c, n):
    c = np.array(c)
    n = np.array(n)
    p = p[0:3]
    p = np.array(p)
    pc = p - c
    dis = distance(p, c)  # distance of point to center of triangle
    va_cos = np.dot(pc, n) / dis

    if va_cos > 1:
        va_cos = 1.
    elif va_cos < -1:
        va_cos = -1.

    return va_cos, dis


def get_view_quality(p, c, n, v1, v2, v3):
    va_cos, dis = get_view_angle(p, c, n)
    v1 = np.array(v1)
    v2 = np.array(v2)
    v3 = np.array(v3)

    rad = tan(radians(FOV / 2)) * dis * va_cos   # get radius of cone projected on triangle
    rad_max = tan(radians(FOV/2)) * DISTANCE_MAX

    d1 = np.linalg.norm(v1 - c)
    d2 = np.linalg.norm(v2 - c)
    d3 = np.linalg.norm(v3 - c)
    dd = [d1, d2, d3]
    d_max = max(dd)

    vq = abs(rad - d_max) / (rad_max-d_max)
    va = acos(va_cos) / (pi/2 - deg2rad(INCIDENCE_ANGLE))
    return vq, va


def triangle(v1, v2, v3, n, num, obstacle):
    vc = [0.0, 0.0, 0.0]
    l12 = [0.0, 0.0, 0.0]
    l31 = l12[:]
    l23 = l12[:]
    c12 = l12[:]
    c13 = l12[:]
    c23 = l12[:]

    for i in range(3):
        vc[i] = (v1[i] + v2[i] + v3[i]) / 3
        l12[i] = v2[i] - v1[i]
        l23[i] = v3[i] - v2[i]
        l31[i] = v1[i] - v3[i]
        c12[i] = (v1[i] + v2[i]) / 2
        c13[i] = (v1[i] + v3[i]) / 2
        c23[i] = (v2[i] + v3[i]) / 2

    norm_ = n[:]
    # center of norm end point
    vn = norm_ + vc
    # norm of incidence angles along three edges of triangle
    n1 = np.dot(rotation_matrix(l12, radians(-1 * (90 - INCIDENCE_ANGLE))), norm_)
    n2 = np.dot(rotation_matrix(l23, radians(-1 * (90 - INCIDENCE_ANGLE))), norm_)
    n3 = np.dot(rotation_matrix(l31, radians(-1 * (90 - INCIDENCE_ANGLE))), norm_)

    valid = []
    non_valid = False
    # search space of x,y,z in local coordinates
    x_range, y_range, z_range = search_space(v1, v2, v3, vc, norm_)

    count = 0
    while len(valid) < num:  # do iteration until find # of feasible viewpoint or iteration which comes first
        count += 1

        value_z = uniform(z_range[0], z_range[1])
        dd = math.tan(deg2rad(90 - INCIDENCE_ANGLE)) * value_z
        radius = x_range[1] + dd

        # uniform sampling between 0-radius
        r = uniform(0, 1) * radius

        # uniform sampling between 0-2pi
        phi = uniform(0, 1) * 2 * pi
        value_x = cos(phi) * r
        value_y = sin(phi) * r

        if count >= 5000:
            non_valid = True
            print('cannot find feasible viewpoint in 5000 iterations, create nadir views ')
            valid = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * num
            break

        pp = [value_x, value_y, value_z]  # p (x,y,z) in local coordinates
        p = global_coordinates(pp, vc, norm_, v1)  # p in global coordinates
        d = list(np.array(p) - np.array(vc))
        dis_ = np.dot(d, norm_)
        vp, ray = get_rpy(p, v1, v2, v3, norm_)

        if p[2] < Z_MIN:
            continue
        #check_collision_l, wp = check_collisions_with_ObbTree_wp(obstacle, vp[0:3], rad=DIS_TOLERANCE)
        #if check_collision_l:
        #    continue
        if dis_ >= DISTANCE_MIN and dis_ <= DISTANCE_MAX:
            if distance(p, vc) <= DISTANCE_SHARPNESS:
                if np.dot(list(np.array(p) - np.array(c12)), n1) >= 0 and np.dot(list(np.array(p) - np.array(c23)), n2) >= 0 and np.dot(list(np.array(p) - np.array(c13)), n3) >= 0:
                    if check_pitch(vp):
                        valid.append(vp)

    return vc, vn, valid, non_valid, n1, n2, n3, c12, c13, c23