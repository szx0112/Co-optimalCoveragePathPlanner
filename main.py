from PSO import *
import time


# get the viewpoint angle range and distance range for the best path
def get_path_vq_parameters(path, vs, vn):
    va_list = []
    vd_list = []
    for i in range(len(path)):
        if i > 0 and i < len(path)-1:  # skip the starting vp
            p = path[i]
            vs_ = vs[i - 1]
            vn_ = vn[i - 1]

            vc = vs_[3]
            n = vn_[3]
            va_cos, vd = get_view_angle(p, vc, n)
            va = round(rad2deg(math.acos(va_cos)), 4)
            vd_n = round(vd * va_cos, 4)
            va_list.append(va)
            vd_list.append(vd_n)
            #vq_d, vq_a = get_view_quality(p, vc, n, v1, v2, v3)
    max_va = max(va_list)
    min_va = min(va_list)

    max_vd = max(vd_list)
    min_vd = min(vd_list)

    ave_va = round(sum(va_list)/len(va_list),2)
    ave_vd = round(sum(vd_list)/len(vd_list),2)
    return [min_va, max_va, ave_va], [min_vd, max_vd, ave_vd], va_list, vd_list

def run_ccpp(filename, start_, end_, pop_size, iter_num):
    # load model
    tri, norm, obs_range = readSTL(filename)
    print("size of mesh: " + str(len(tri)))
    # collision detection
    vtk_obstacles = get_vtk_meshes([filename])

    # PSO with pop size and generation
    _, path, index, vn, vs, bests, best_paths, all_paths, headings, index_tri_covered = pso(start_, end_, tri, vtk_obstacles, norm, pop_size, iter_num)
    the_dis, _ = compute_vp_dist(path, index)
    print("optima path distance: " + str(the_dis))
    rrt_paths = []
    new_path = []
    new_index = []
    new_headings = []

    for i in range(len(path)-1):
        vp0 = path[index[i]]
        heading0 = headings[index[i]]
        new_path.append(vp0)
        new_headings.append(heading0)
        new_index.append(index[i])
    new_path.append(path[index[len(path)-1]])
    new_index.append(index[len(path)-1])
    new_headings.append(headings[index[len(path)-1]])
    cost = path_length(new_path)
    return tri, new_path, new_index, new_headings, cost, path, index, rrt_paths, bests, index_tri_covered

def run(mesh_folder, filename, start_, end_, pop_size, iter_num, save=True, show=False):
    start_time = time.time()
    mesh_filename = mesh_folder + "/" + filename
    obj, new_path, new_index, headings, cost, path, index, rrt_paths, bests, index_obj_covered = run_ccpp(
        mesh_filename, start_, end_, pop_size, iter_num)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print("total cost: " + str(cost))
    print("Elapsed time: " + str(elapsed_time / 60.0) + " minutes")
    if show:
        print('show results')
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        # plot stl model
        for i in range(len(obj)):
            v1 = obj[i][0:3]
            v2 = obj[i][3:6]
            v3 = obj[i][6:9]
            n = np.array(surface_norm(v2 - v1, v3 - v2))
            vc = (np.array(v1) + np.array(v2) + np.array(v3)) / 3
            vn = 0.2 * n + vc
            if index_obj_covered[i] == 1:
                ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], color='k')
                ax.plot([v1[0], v3[0]], [v1[1], v3[1]], [v1[2], v3[2]], color='k')
                ax.plot([v2[0], v3[0]], [v2[1], v3[1]], [v2[2], v3[2]], color='k')
            else:
                ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], color='r')
                ax.plot([v1[0], v3[0]], [v1[1], v3[1]], [v1[2], v3[2]], color='r')
                ax.plot([v2[0], v3[0]], [v2[1], v3[1]], [v2[2], v3[2]], color='r')

        # plot main path
        for i in range(len(new_path) - 1):
            wp0 = new_path[i][0:3]
            wp1 = new_path[i + 1][0:3]
            ax.plot([wp0[0], wp1[0]], [wp0[1], wp1[1]], [wp0[2], wp1[2]], 'g', alpha=1)
        # plot rrt paths
        for i in range(len(rrt_paths)):
            sub_path = rrt_paths[i]
            for j in range(len(sub_path) - 1):
                wp0 = sub_path[j][0:3]
                wp1 = sub_path[j + 1][0:3]
                ax.plot([wp0[0], wp1[0]], [wp0[1], wp1[1]], [wp0[2], wp1[2]], 'b-.')

        # plot arrow of vp
        for i in range(len(new_path)):
            p = new_path[i]
            ind = new_index[i]
            if ind > 0 and ind < len(new_index) - 1:
                v1 = obj[ind - 1][0:3]
                v2 = obj[ind - 1][3:6]
                v3 = obj[ind - 1][6:9]
                vc = (np.array(v1) + np.array(v2) + np.array(v3)) / 3
                vec = vc - np.array(p[0:3])
                ax.quiver(p[0], p[1], p[2], vec[0], vec[1], vec[2], length=1.0, normalize=True)
        plt.figure(2)
        plt.plot(bests)
        plt.xlabel("iteration")
        plt.ylabel("cost")
        plt.title("w_dis: " + str(w_dis) + ", w_vq(vq_dis, vq_ang): " + str(w_vq) + "(" + str(w_vq1) + ", " + str(w_vq2) + ")")
        plt.show()

    # save figure
    if save:
        print('save results')
        save_filename = filename + '_' + '[' + str(w_vq) + ', ' + str(w_dis) + ']' + '_' + str(pop_size) + '_' + str(iter_num) + '_' + str(FOV) + '_' + '[' + str(DISTANCE_MIN) + ', ' + str(DISTANCE_MAX) + ']' + '_' + str(DISTANCE_SHARPNESS) + '_' + '[' + str(GIMBAL_PITCH_LOWER_LIMIT) + ', ' + str(GIMBAL_PITCH_UPPER_LIMIT) + ']' + '.txt'
        save_figure = 'result/figure/' + save_filename
        plt.savefig(save_figure+'_path.pdf', dpi=1200)

    if save:
        plt.savefig(save_figure+'_cost.pdf', dpi=1200)

        # save path to file
        headings[0] = [0,0,0]
        save_path = 'result/path/' + save_filename
        with open(save_path, 'w') as fp:
            for i in range(len(new_path)):
                pos = new_path[i][0:3]
                for p in pos:
                    fp.write("%s," % p)
                if i == 0 or i == len(new_path)-1:
                    head = [0,0,0]
                else:
                    head = headings[i]

                for h in head:
                    fp.write("%s," % h)

                ori = new_path[i][3:6]
                for o in ori:
                    fp.write("%s," % o)
                fp.write("\n")
            fp.close()
        print("path saved to file: {}".format(save_path))

        # save cost to file
        save_cost = 'result/cost/' + save_filename
        with open(save_cost, 'w') as fc:
            for item in bests:
                fc.write("%s\n" % item)
            fc.close()
        print("cost saved to file: {}".format(save_cost))
    return path_length(new_path)

def main():
    run(mesh_folder, filename, start_point, end_point, pop_size, iter_num, save=True, show=True)
    print('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

if __name__ == "__main__":
    main()