import random as rand
import operator
import InvokeLKH_m as lkh
import math
from initSample import triangle, distance
from readSTL import *
import copy

def compute_neighbor_distance(vp_lists, vp, index_path):
    cost, _ = compute_vp_dist(vp, index_path)
    vp_set = list(map(list, zip(*vp_lists))) # transpose, each list of vp_set is vertices in a triangle
    best = vp[:]    # assume best vp is the input vp, will update it.
    # iterate through the path based on index
    perm = np.random.permutation(len(index_path)-2)
    for i in range(len(index_path)-2):  # -2 is the deduct of first (start) point and last point
            ii = perm[i]
            ind_curr = index_path[ii+1]  # start from 1 instead of 0
            ind_next = index_path[ii+2]  # start from 2
            ind_accs = index_path[ii]    # start from 0

            vert_curr = vp[ind_curr]   # get vertex in current feaisble vp
            vert_next = vp[ind_next]
            vert_accs = vp                                                                                                                                                                                                                                                                                                                                                                                                                                           [ind_accs]

            cost_curr_next = distance(vert_curr, vert_next)
            cost_curr_accs = distance(vert_curr, vert_accs)
            cost_vp = cost_curr_next + cost_curr_accs   # cost of current neighbors

            # three set of vertices
            vps_currs = vp_set[ind_curr]

            for j in range(len(vps_currs)): # number of samples in each triangle
                vps_curr = vps_currs[j]

                cost_curr_next_vp = math.sqrt((vps_curr[0]-vert_next[0])**2+(vps_curr[1]-vert_next[1])**2+(vps_curr[2]-vert_next[2])**2)
                cost_curr_accs_vp = math.sqrt((vps_curr[0]-vert_accs[0])**2+(vps_curr[1]-vert_accs[1])**2+(vps_curr[2]-vert_accs[2])**2)
                cost_curr_vp = cost_curr_next_vp + cost_curr_accs_vp

                if cost_curr_vp < cost_vp:  # found a better neighborhood
                    cost_vp = cost_curr_vp
                    best[ind_curr] = vps_curr

    best = list(best)
    cost_new, _ = compute_vp_dist(best, index_path)

    if cost_new < cost:
        return best, cost_new
    else:
        return vp, cost


def compute_neighbor_loss(vp_lists, vp, index_path, vs, vn, w_vq, w_dis, dis_base):
    loss, loss_ = compute_vp_loss(vp, index_path, w_vq, w_dis, dis_base)
    vp_set = list(map(list, zip(*vp_lists)))  # transpose, each list of vp_set is vertices in a triangle
    #new_index = index_path[:]
    best = copy.copy(vp)  # assume best vp is the input vp, will update it.
    for i in range(1, len(index_path) - 1):  # -2 is the deduct of first (start) point and last point
        ind_curr = index_path[i]  # start from 1 instead of 0
        ind_next = index_path[i + 1]  # start from 2
        ind_accs = index_path[i - 1]  # start from 0

        vert_next = vp[ind_next]
        vert_accs = vp[ind_accs]

        # set of vertices
        vps_currs = vp_set[ind_curr]
        the_loss = loss_[ind_curr]
        for j in range(len(vps_currs)):  # number of samples in each triangle
                vps_curr = vps_currs[j]
                dis = distance(vert_accs, vps_curr) + distance(vps_curr, vert_next)
                dis = dis / 2
                dis = dis * (len(index_path)-1) / dis_base

                vq_list = vp_lists[j].vq_list
                vq = vq_list[ind_curr]
                vq_dis_list = vp_lists[j].vq_dis_list
                vq_dis = vq_dis_list[ind_curr]
                vq_ang_list = vp_lists[j].vq_ang_list
                vq_ang = vq_ang_list[ind_curr]

                curr_loss = w_vq * vq + w_dis * dis

                if curr_loss < the_loss:  # found a better neighborhood
                    the_loss = curr_loss
                    best[ind_curr] = vps_curr
                    best.vq_list[ind_curr] = vq
                    best.vq_dis_list[ind_curr] = vq_dis
                    best.vq_ang_list[ind_curr] = vq_ang
    loss_new, loss_list_new = compute_vp_loss(best, index_path, w_vq, w_dis, dis_base)
    if loss_new < loss:
        return best, loss_new
    else:
        return vp, loss

def compute_vp_loss(vp, index_path, w_vq, w_dis, dis_base):
    loss = 0
    loss_ = [0]*len(index_path)
    vq = vp.vq_list
    for i in range(1, len(index_path)):
        ind_accs = index_path[i-1]
        ind_curr = index_path[i]
        best_accs = vp[ind_accs]
        best_curr = vp[ind_curr]

        if i != len(index_path) - 1:
            ind_next = index_path[i+1]
            best_next = vp[ind_next]

            dis = distance(best_accs, best_curr) + distance(best_curr, best_next)
            dis = dis / 2
        else:
            dis = distance(best_accs, best_curr)

        dis = dis * (len(index_path) - 1) / dis_base
        loss += w_vq * vq[ind_curr] + w_dis * dis
        loss_[ind_curr] = w_vq * vq[ind_curr] + w_dis * dis

    loss = loss / (len(index_path) - 1)
    return loss, loss_

# compute the cost of a path 
def compute_vp_dist(vp, index_path):
    cost = 0
    for i in range(len(index_path)-1):
        ind_accs = index_path[i]
        ind_next = index_path[i+1]
        best_accs = vp[ind_accs]
        best_next = vp[ind_next]
        cost += distance(best_accs, best_next)

    cost_ = [0] * len(index_path)
    for i in range(1, len(index_path)):

        ind_accs = index_path[i-1]
        ind_curr = index_path[i]
        vp_accs = vp[ind_accs]
        vp_curr = vp[ind_curr]
        if i != len(index_path) - 1:
            ind_next = index_path[i+1]
            vp_next = vp[ind_next]
            cost_[ind_curr] = (distance(vp_accs, vp_curr) + distance(vp_curr, vp_next)) / 2
        else:
            cost_[ind_curr] = distance(vp_accs, vp_curr)
    return cost, cost_

# input the viewpoint sets, output the Eulerian distance matrix
def find_best_vp_distance_list(vp_lists, best_path, index_path):
    vp_costs = []
    cost_best, _ = compute_vp_dist(best_path, index_path)

    for i in range(len(vp_lists)):  # list all paths (vp_lists)
        cost_vp, _ = compute_vp_dist(vp_lists[i], index_path)
        vp_costs.append(cost_vp)
        if cost_vp != 0 and cost_vp < cost_best:
            best_path = vp_lists[i]
            cost_best = cost_vp
    return best_path, round(cost_best, 2), vp_costs

# find path with least loss based on path index: first step of graph search
def find_best_vp_loss_list(vp_lists, best_path, index_path, w_vq, w_dis, dis_base):
    best_loss, _ = compute_vp_loss(best_path, index_path, w_vq, w_dis, dis_base)
    vp_losses = []
    for i in range(len(vp_lists)):  # list all paths
        vp_loss, _ = compute_vp_loss(vp_lists[i], index_path, w_vq, w_dis, dis_base)
        vp_losses.append(vp_loss)
        if vp_loss != 0 and vp_loss < best_loss:
            best_path = vp_lists[i]
            best_loss = vp_loss
    return best_path, round(best_loss,4), vp_losses

# given the set of feasible sampling viewpoints, compute the new sample set such that the order is the shortest path generated with TSP and return the min cost
# make it able to handle None value vps
def find_path(vps):
    # input is the list of viewpoints (x,y,z,r,p,y) with start and finish position
    new_vps = vps[:]
    pos = [x[0:3] for x in vps]   # just need x , y, z to calcuate shortest path
    # edge list
    edge_list = []
    node_len = len(pos) # number of points, start + end + mesh size
    for i in range(node_len):
        for j in range(node_len):
            index1 = i+1
            index2 = j+1
            if i != j:
                edge_list.append([index1, index2])
    # distance matrix
    dis_scale = 10
    large_value = 99999 * dis_scale
    dis_mat = [large_value]*(len(pos)+1)  # add one dummy point as the last point (n+1) in the matrix to make it between start (0) and end (n) points
    for i in range(len(pos)):
        line = [large_value]*(len(pos)+1)
        for j in range(len(pos)):
            line[j] = int(round(distance(pos[i], pos[j]) * dis_scale))
        dis_mat[i] = line
    dis_mat[-1] = [large_value]*(len(pos)+1)
    # make the first and last vp as the start and end points
    dis_mat[-1][-1] = 0
    dis_mat[-1][0] = 1
    dis_mat[-1][-2] = 1
    dis_mat[0][-1] = 1
    dis_mat[-2][-1] = 1
    index, cost = lkh.main(dis_mat)
    cost = cost*1.0/dis_scale
    index = index[0:len(index)-1]   # eliminate the end symbol in lkh: -1
    index = [int(ind)-1 for ind in index] # in lkh index start from 1, we make it back to start from 0
    if index[1] == len(index) - 1 and index[2] == len(index) - 2:
        index = index[::-1]
        index = index[-1:] + index[:-1]
    index = index[:-1]      # elminate the dummy point
    return new_vps, index, cost


# generate the initial viewpoint set based on the target mesh model, output includes the computed feasible viewpoints set, triangle vertice and vector sets
def generate_vp(num, obj, obstacle, norm):
    vps = [None] * len(obj)
    vs = [None] * len(obj)
    ns = [None] * len(obj)
    index = [0] * len(obj)

    obj_pymp = obj
    norm_pymp = norm
    index_pymp = index
    ns_pymp = ns
    vs_pymp = vs
    vps_pymp = vps

    count = 0
    for i in range(len(obj_pymp)):
                tri = obj_pymp[i]
                v1 = tri[0:3]
                v2 = tri[3:6]
                v3 = tri[6:9]
                n = norm_pymp[i]
                vc_, vn_, valid_, non_valid_, n1, n2, n3, c12, c13, c23 = triangle(v1,v2,v3,n,num,obstacle)

                count += 1
                if len(valid_) == num:    # a list, check if it contains at least one vertice, which means a single
                    vp_ = [list(vp) for vp in valid_]
                    vps_pymp[i] = vp_
                    # check if the viewpoints are found
                    index_pymp[i] = 1
                elif len(valid_) == 0 and non_valid_ is True:
                    print('No able to sample feasible viewpoint at: ' + str(i))
                    vps_pymp[i] = [[None]]*num
                else: # len(valid_) < num and > 0
                    len_valid = len(valid_)
                    index_pymp[i] = 1
                    while len_valid < num: # until valid reach number
                        rand_ind = rand.randint(0, len_valid-1)
                        valid_.append(valid_[rand_ind])
                    vp_ = [list(vp) for vp in valid_]
                    vps_pymp[i] = vp_

                vs_pymp[i] = [v1, v2, v3, vc_]
                ns_pymp[i] = [list(n1), list(n2), list(n3), list(n)]

    vs = list(vs_pymp)
    ns = list(ns_pymp)
    vps = list(vps_pymp)
    index = list(index_pymp)

    return vps, vs, ns, index
