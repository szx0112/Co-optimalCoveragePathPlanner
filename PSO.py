from deap import base, creator
from localPath import *
from initSample import *
import numpy as np

def sigmoid(x):
  return 1 / (1.5 + math.exp(-2.6*x))

# Particle Swarm Optimization
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Particle", list, fitness=creator.FitnessMin, speed=list, smin=None, smax=None, bcost=None, best=list, bindex=list, dis=None, dis_list=list, dis_norm=None, vq=None, vq_list=list, vq_dis_list=list, vq_ang_list=list)

# num: number of samples at each triangle
# size: dimension of speed: 3
# generte num of vp_lists
def population(start_, end_, obj, obstacle, norm, num, size, smin, smax):
    # generate num of lists of vp list based on mesh size
    vps, vs, vn, ind_ = generate_vp(num, obj, obstacle, norm)

    # ind_ is an array with length equal to obj, if ele == 1, find vp (including duplicated vp), else 0, not found
    # find the number of triangles not visible
    num_non_vis = sum(ind_)

    vp_lists = list(zip(*vps))
    pop_ = [None] * len(vp_lists)

    for i in range(len(vp_lists)):
        vps_ = vp_lists[i]
        vps_ = [list(vp) for vp in vps_]
        vp_lists[i] = vps_

    for i in range(len(vp_lists)):
        if type(vp_lists[i]) is not list:
            vp_lists[i] = list(vp_lists[i])
        vp_lists[i].insert(0, start_)
        vp_lists[i].insert(len(vp_lists[i]), end_)   # add end point

    for i in range(len(vp_lists)):
        part = creator.Particle(vp_lists[i])  # each list of vertice is a particle
        part.speed = []

        speeds = [0]*len(vp_lists[i])
        for j in range(len(vp_lists[i])):
            if j == 0 or j == len(vp_lists[i])-1:  # when i is 0 and end. the first vertice of vp list is the start point
                speed = [0.0, 0.0, 0.0]
            else:
                speed = [rand.uniform(smin, smax) for _ in range(size)]
            speeds[j] = speed
        part.speed = speeds
        part.smin = smin
        part.smax = smax
        part.dis_list = [None]*len(part)
        pop_[i] = part

    return pop_, vs, vn, ind_, num_non_vis


def get_headings(part, vs, vn, obj):
    len_part = len(part)
    rays = [0, 0, 0]*len_part
    for i in range(1, len(part)-1):
        pos = part[i]
        _, _, ray = check_constraints(pos, vs[i-1], vn[i-1])
        rays[i] = ray
    return rays

# each part is a list of vertices with 6 dimensions
# size: is the updated dimension number, here we only update x,y,z
# vs: list of triangles [v1,v2,v3,vc]
# vn: list of norms [n1,n2,n3,nc]
# site obstacle
def updateParticle_(part, best, size, vs, vn, obstacle):
    part_new = copy.copy(part)
    speed = part_new.speed
    pbest = part_new.best

    omga = [0.729]*3
    C1 = [2., 2., 2.]
    C2 = [2., 2., 2.]

    count_no_updates = 0
    no_updates_speeds = []
    for i in range(len(part)):  # part[i] is a 6-d vector
        if i > 0 and i<len(part)-1:
            u1 = [round(rand.uniform(0, 1), 4) for _ in range(size)]
            u2 = [round(rand.uniform(0, 1), 4) for _ in range(size)]

            v_u1 = list(map(operator.mul, u1, map(operator.sub, pbest[i][0:3], part_new[i][0:3])))  # v_u1  = u1*(lbest-part)
            v_u1 = [round(v1, 4) for v1 in v_u1]
            v_u2 = list(map(operator.mul, u2, map(operator.sub, best[i][0:3], part_new[i][0:3])))  # v_u2 = u2*(best-part)
            v_u2 = [round(v2, 4) for v2 in v_u2]

            #speed = w*speed + C1*v_u1 + C2*v_u2
            speed_p = list(map(operator.add, map(operator.mul, speed[i], omga), map(operator.add, map(operator.mul, C1, v_u1), map(operator.mul, C2, v_u2))))
            for j in range(len(speed_p)):
                if speed_p[j] < part.smin:
                    speed_p[j] = round(part.smin, 4)
                elif speed_p[j] > part.smax:
                    speed_p[j] = round(part.smax, 4)

            speed_update_ = copy.copy(speed_p)
            count = 0
            check_ = False
            temp_part = None
            not_update = False
            while not check_:
                # decaying function
                speed_update_ = [round((0.5**count)*su, 4) for su in speed_update_]
                # number of attempts reach upper limit or speed is too small
                if count > 6 or max(np.absolute(speed_update_)) < 0.001:
                    #print(count, speed_update_)
                    speed_update_ = [0., 0., 0.]
                    temp_part = copy.copy(part_new[i])
                    not_update = True
                else:
                    # part_new = part_old + new_speed
                    temp_part_ = list(map(operator.add, part_new[i][0:3], speed_update_))
                    temp_part_ = [round(t, 4) for t in temp_part_]
                    check_1, temp_part, _ = check_constraints(temp_part_, vs[i-1], vn[i-1])  # check is true, then no collision
                    #check_2, _ = check_collisions_with_ObbTree_wp(obstacle, temp_part[0:3], rad=DIS_TOLERANCE)
                    if check_1:
                        check_ = True
                    count += 1

                if not_update:
                    count_no_updates += 1
                    break

            #if not_update:
            #    no_updates_speeds.append(speed_update_)
            part_new[i] = list(temp_part)      # update location
            part_new.speed[i] = speed_update_    # update speed
    return part_new, count_no_updates


# input is 6-d vector as a viewpoint
# vp: new viewpoint: loc + ori
# vs: triangle vertices (v1,v2,v3,vc)
# vn: triangle norms (n1,n2,n3,vn)
# return: True if no constraint is violated, False if constraints failed
def check_constraints(p, vs, vn):
    p = p[0:3]
    norm_ = vn[3]
    vc = vs[3]
    v1 = vs[0]
    v2 = vs[1]
    v3 = vs[2]
    n1 = vn[0]
    n2 = vn[1]
    n3 = vn[2]
    c12 = (np.array(v1) + np.array(v2))/2
    c23 = (np.array(v2) + np.array(v3))/2
    c13 = (np.array(v1) + np.array(v3))/2

    vp, ray = get_rpy(p, v1, v2, v3, norm_)
    d = list(np.array(p) - np.array(vc))
    dis_ = np.dot(d, norm_)

    check_ = False
    check_space = False
    # within feasible space
    if vp[2] >= Z_MIN:
        check_space = True

    if dis_ >= DISTANCE_MIN and dis_ <= DISTANCE_MAX and check_space is True:
        if distance(p, vc) <= DISTANCE_SHARPNESS:
            if np.dot(list(np.array(p) - np.array(c12)), n1) >= 0 and np.dot(list(np.array(p) - np.array(c23)), n2) >= 0 and np.dot(list(np.array(p) - np.array(c13)), n3) >= 0:
                if check_pitch(vp):
                    check_ = True
    return check_, vp, ray


def update_vq(part, vs, vn, w_vq1, w_vq2):
    if type(part) is list:
        part = creator.Particle(part)
    len_part = len(part)
    vq_dis = [0] * len_part
    vq_ang = [0] * len_part
    vq = [0] * len_part
    vq_ = 0
    for i in range(len_part):
        if i > 0 and i < len_part - 1:  # skip the starting vp
            p = part[i]
            vs_ = vs[i - 1]
            vn_ = vn[i - 1]

            vc = vs_[3]
            v1 = vs_[0]
            v2 = vs_[1]
            v3 = vs_[2]
            n = vn_[3]
            vq_d, vq_a = get_view_quality(p, vc, n, v1, v2, v3)
            vq_dis[i] = vq_d
            vq_ang[i] = vq_a
            vq[i] = w_vq1 * vq_d + w_vq2 * vq_a
            vq_ += vq[i]
    part.vq_dis_list = vq_dis
    part.vq_ang_list = vq_ang
    part.vq_list = vq
    part.vq = vq_ / (len_part - 2)  # start and end points
    return part

def pso(start_, end_, obj, obstacle, norm, num, num_of_iteration):
    # initialization
    size = 3
    smin = -0.08 * (DISTANCE_MAX - DISTANCE_MIN)
    smax = 0.08 * (DISTANCE_MAX - DISTANCE_MIN)

    pop_, vs, vn, index_tri_covered, _ = population(start_, end_, obj, obstacle, norm, num, size, smin, smax)

    pops = [None] * num_of_iteration
    bests = [None] * num_of_iteration
    best_costs = [1.] * num_of_iteration
    best = []
    gbest_cost = np.inf
    best_index = None
    dis_init = np.inf

    for g in range(num_of_iteration):
        print('=================================')
        print('iteration: ' + str(g))

        for j in range(len(pop_)):
            part = pop_[j]
            len_part = len(part)
            vq_dis = [0]*len_part
            vq_ang = [0]*len_part
            vq = [0]*len_part
            vq_sum = 0

            for i in range(len_part):
                # skip the first and last view
                if i > 0 and i < len_part - 1:
                    p = part[i]
                    vs_ = vs[i-1]
                    vn_ = vn[i-1]
                    vc = vs_[3]
                    v1 = vs_[0]
                    v2 = vs_[1]
                    v3 = vs_[2]
                    n = vn_[3]

                    vq_d, vq_a = get_view_quality(p, vc, n, v1, v2, v3)
                    vq_dis[i] = vq_d
                    vq_ang[i] = vq_a
                    vq[i] = w_vq1 * vq_d + w_vq2 * vq_a
                    vq_sum += vq[i]

                part.vq_dis_list = vq_dis
                part.vq_ang_list = vq_ang
                part.vq_list = vq
                part.vq = vq_sum / (len_part - 2)
                pop_[j] = part

        # random seed
        ran_index = rand.randint(0, len(pop_)-1)
        index_part = pop_[ran_index]
        # find a feasible path with TSP
        _, index_of_path, _ = find_path(index_part)
        dis_1, _ = compute_vp_dist(index_part, index_of_path)

        if g == 0:
            dis_init = dis_1
            print("init dis: " + str(dis_init))

        # particles
        dis_total = 0
        for j in range(len(pop_)):
            part = pop_[j]
            dis_, dists_ = compute_vp_dist(part, index_of_path)
            dis_norm = (dis_ - 0.2*dis_init) / dis_init*(1-0.2)
            part.dis_norm = dis_norm
            part.dis = dis_
            part.dis_list = dists_
            pop_[j] = part
            dis_total += dis_
        print('average path dis of particles: ', np.round(dis_total/len(pop_), 3))

        # greedy search (gs)
        if w_dis == 1:
            dis_, dists_ = compute_vp_dist(index_part, index_of_path)
            # find min path in all pops
            short_path, _, _ = find_best_vp_distance_list(pop_, index_part, index_of_path)
            # find min path with neighbor vertices
            shortest_path, shortest_dist = compute_neighbor_distance(pop_, short_path, index_of_path)
            lbest_loss = copy.copy(shortest_dist)
            lbest_path = copy.copy(shortest_path)
            print('lbest before gs: ' + str(dis_))
            print('lbest after gs: ' + str(lbest_loss))
        else:
            loss, loss_ = compute_vp_loss(index_part, index_of_path, w_vq, w_dis, dis_init)
            index_part = update_vq(index_part, vs, vn, w_vq1, w_vq2)
            short_path, _, _ = find_best_vp_loss_list(pop_, index_part, index_of_path, w_vq, w_dis, dis_init)
            lbest_path = short_path
            best_path, best_loss = compute_neighbor_loss(pop_, lbest_path, index_of_path, vs, vn, w_vq, w_dis, dis_init)
            lbest_loss = best_loss
            lbest_path = best_path
            print('lbest before gs: ' + str(loss))
            print('lbest loss after gs: ' + str(lbest_loss))

        # update local best
        for j in range(len(pop_)):
            part = pop_[j]
            if w_dis == 1:
                loss = part.dis
            else:
                loss = w_dis*part.dis + w_vq*part.vq  # loss function
            # update local best
            if not part.best or loss < part.bcost:
                part.bcost = loss
                #if j == ran_index:
                #    part.best = copy.copy(lbest_path)
                #else:
                part.best = part
                part.bindex = index_of_path
                pop_[j] = part

        # update global best
        if lbest_loss < gbest_cost or len(best) == 0:
            best = copy.copy(lbest_path)
            gbest_cost = copy.copy(lbest_loss)
            best_index = copy.copy(index_of_path)
            print('---------------------------------------')
            print("Update gbest_cost " + str(gbest_cost))
            print('---------------------------------------')

            best_costs[g] = gbest_cost
            bests[g] = best
        else:
            best_costs[g] = best_costs[g-1]
            bests[g] = bests[g-1]

        for j in range(len(pop_)):
            part = pop_[j]
            for jj in range(len(part)):
                vp = list(part[jj])
                part[jj] = vp
            pop_[j] = part

        for b in range(len(best)):
           best[b] = list(best[b])

        count_no_updates = [0]*len(pop_)
        for jj in range(len(pop_)):
            part = pop_[jj]
            part_new, _ = updateParticle_(part, best, 3, vs, vn, obstacle)
            pop_[jj] = part_new
        pop_ = list(pop_)
        best_headings = get_headings(best, vs, vn, obj)
    return pop_, best, best_index, vn, vs, best_costs, bests, pops, best_headings, index_tri_covered

toolbox = base.Toolbox()
toolbox.register("population", population)