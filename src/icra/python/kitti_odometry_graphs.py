import numpy as np
from numpy import cos, sin
import cv2
# Prevents cv2 bugs (3.10.0 problem)
cv2.ocl.setUseOpenCL(False)
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pykitti
import itertools
from kitti_graphs import plot_traj, get_edges_from_traj_and_weights, vis_graph, vis_graph_paper

basedir_odom = '/drive1/Datasets/kitti/odometry/dataset'

def plot_odometry_trajectory(sequence, downsample=10):
    dataset = pykitti.odometry(basedir_odom, sequence)
    # pose = next(iter(itertools.islice(dataset.poses, 1, None)))
    # print(dataset.calib.T_cam0_velo)
    T_calib = dataset.calib.T_cam2_velo
    T_calib = np.linalg.inv(T_calib)
    x = []
    y = []
    z = []
    # yaw = []
    camera_normal = np.zeros((0, 3))
    for T in dataset.poses:
        T = T_calib.dot(T)
        x.append(T[0,3])
        y.append(T[1,3])
        z.append(T[2,3])
        # Yaw not useful here (it's the camera's orientation)
        # yaw.append(np.arctan2(-T[0,1], T[0,0]))
        camera_normal = np.vstack((camera_normal, T[0:3,2].T))
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    # yaw = np.array(yaw)
    yaw = np.arctan2(camera_normal[:,1], camera_normal[:,0])
    plot_traj(x,y,yaw,downsample)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x,y,z)
    inds = range(0, len(x), downsample)
    # ax.quiver(x[inds], y[inds], z[inds], cos(yaw[inds]), sin(yaw[inds]), 0*yaw[inds])
    ax.quiver(x[inds], y[inds], z[inds], camera_normal[inds,0], camera_normal[inds,1], camera_normal[inds,2], color='r')
    plt.axis('equal')
    plt.show()


def gen_weights_and_poses(sequence, fast_threshold=150):
    dataset = pykitti.odometry(basedir_odom, sequence, imformat='cv2')
    print("Dataset loaded")
    T_calib = dataset.calib.T_cam0_velo
    T_calib = np.linalg.inv(T_calib)
    x = []
    y = []
    z = []
    w = []
    camera_normal = np.zeros((0, 3))
    # print("Dateset lengths: ", sum(1 for _ in dataset.cam2), sum(1 for _ in dataset.poses))
    count = 0
    for T in dataset.poses:
        T = T_calib.dot(T)
        x.append(T[0,3])
        y.append(T[1,3])
        z.append(T[2,3])
        camera_normal = np.vstack((camera_normal, T[0:3,2].T))

    # Didn't like the zip, had to do them separately
    for img in dataset.cam2:
        # find the keypoints with ORB
        orb = cv2.ORB_create(nfeatures=10000, fastThreshold=fast_threshold)
        kp = orb.detect(img,None)
        w.append(len(kp))
        # count += 1
        # if count % 100 == 0:
        #     print("Count: ", count)
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    yaw = np.arctan2(camera_normal[:,1], camera_normal[:,0])
    return x,y,z,yaw,w

if __name__=='__main__':

    ## Params
    data_save_path = '../data/odometry/'
    fast_threshold = 150
    r = 30.0
    fov = np.pi/2
    n_diff = 200
    n_poly = 10
    overlap_min = 0.8
    # 6 and 7 are not as good as the others
    good_sequences = [0,2,5,6,7,8]

    ########################################
    ## Visualize all the sequences in turn
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     plot_odometry_trajectory(sequence)


    #########################################
    ## Get weights and poses, save as files
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     x,y,z,yaw,w = gen_weights_and_poses(sequence, fast_threshold=fast_threshold)
    #     poses = np.vstack((x,y,z,yaw)).T
    #     print("Poses shape: ", poses.shape)
    #     np.savetxt(data_save_path+'poses_'+sequence+'.csv', poses, delimiter=',')
    #     np.savetxt(data_save_path+'weights_'+sequence+'.csv', w, delimiter=',')

    #########################################
    ## Save sweep of overlaps
    ## Added check for bipartite preservation
    overlap_list = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     # Load data
    #     poses = np.loadtxt(data_save_path+'poses_'+sequence+'.csv', delimiter=',')
    #     w = np.loadtxt(data_save_path+'weights_'+sequence+'.csv', delimiter=',')
    #     # Sweep overlap
    #     for overlap in overlap_list:
    #         print("Sequence, overlap: ", sequence, ", ", str(overlap))
    #         E = get_edges_from_traj_and_weights(poses[:,0], poses[:,1], poses[:,3], w, r=r, fov=fov,
    #                                         n_poly=n_poly, overlap_min=overlap, ind_dist=n_diff)
    #         np.savetxt(data_save_path+'edges_'+sequence+'_r_'+str(int(r))+'_overlap_'+str(overlap).replace('.','')+'.csv', E, delimiter=',')

    ##########################################
    ## Save sweep of downsample at overlap_min=0.4
    downsample_list = [2,5,10,15,20]
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     # Load data
    #     poses = np.loadtxt(data_save_path+'poses_'+sequence+'.csv', delimiter=',')
    #     n_poses = poses.shape[0]
    #     w = np.loadtxt(data_save_path+'weights_'+sequence+'.csv', delimiter=',')
    #     for overlap in overlap_list:
    #         E = np.loadtxt(data_save_path+'edges_'+sequence+'_r_'+str(int(r))+'_overlap_'+str(overlap).replace('.','')+'.csv', delimiter=',')
    #         for rate in downsample_list:
    #             ind = np.arange(0,n_poses, rate)
    #             E_idx = np.zeros((0,2))
    #             for i in xrange(0,E.shape[0]):
    #                 if E[i,0] in ind and E[i,1] in ind:
    #                     E_idx = np.vstack((E_idx, E[i,:]))
    #             np.savetxt(data_save_path+'edges_'+sequence+'_r_'+str(int(r))+'_overlap_'+str(overlap).replace('.','')+'_downsample_'+str(rate)+'.csv', E_idx, delimiter=',')



    ##########################################
    # ## Save sweep of radius with no_fov and downsample
    # radius_list = [5,10,15,20,25,30]
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     # Load data
    #     poses = np.loadtxt(data_save_path+'poses_'+sequence+'.csv', delimiter=',')
    #     n_poses = poses.shape[0]
    #     w = np.loadtxt(data_save_path+'weights_'+sequence+'.csv', delimiter=',')
    #     # Sweep overlap
    #     for r_sweep in radius_list:
    #         print("Sequence, radius: ", sequence, ", ", str(r_sweep))
    #         E = np.loadtxt(data_save_path+'edges_'+sequence+'_r_'+str(r_sweep)+'_no_fov.csv', delimiter=',')
    #         for rate in downsample_list:
    #             ind = np.arange(0,n_poses, rate)
    #             E_idx = np.zeros((0,2))
    #             for i in xrange(0,E.shape[0]):
    #                 if E[i,0] in ind and E[i,1] in ind:
    #                     E_idx = np.vstack((E_idx, E[i,:]))
    #             np.savetxt(data_save_path+'edges_'+sequence+'_r_'+str(int(r_sweep))+'no_fov_downsample_'+str(rate)+'.csv', E_idx, delimiter=',')


    ##########################################
    # ## Save sweep of weight vectors for varying thresholds
    # threshold_list = [20, 30, 50, 100, 200]
    # for idx in good_sequences:    
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     for thresh in threshold_list:
    #         x,y,z,yaw,w = gen_weights_and_poses(sequence, fast_threshold=thresh)
    #         np.savetxt(data_save_path+'weights_'+sequence+'_threshold_'+str(thresh)+'.csv', w, delimiter=',')

    ##########################################
    ## Visualize graphs
    # overlap_list = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    # for idx in good_sequences:
    #     sequence = str(idx)
    #     if idx < 10:
    #         sequence = '0' + sequence
    #     # Load data
    #     poses = np.loadtxt(data_save_path+'poses_'+sequence+'.csv', delimiter=',')
    #     w = np.loadtxt(data_save_path+'weights_'+sequence+'.csv', delimiter=',')
    #     x = poses[:,0]
    #     y = poses[:,1]
    #     yaw = poses[:,3]
    #     # Sweep overlap
    #     for overlap in overlap_list:
    #         E = np.loadtxt(data_save_path+'edges_'+sequence+'_r_'+str(int(r))+'_overlap_'+str(overlap).replace('.','')+'.csv', delimiter=',')
    #         vis_graph(x, y, yaw, E, n_split=-1, downsample=20)
    #         plt.show()


    ###########################################
    ## Load DBoW2 data
    # dbow_sequences = [0, 6]
    # dbow2_splits = [2270, 550]
    # dbow2_directory = "../data/odometry/dbow2/"
    # alphas = np.arange(0.1, 1.0, 0.1)
    # # edge_sets = []
    # n_bests = [1, 2, 3, 4, 5, 10, 20, 40, 50]
    # downsample_list = [2,5,10,15,20]
    # match_dist_threshold = 10.0
    # match_counts = np.zeros((len(dbow_sequences), len(alphas), len(n_bests)))
    # candidate_counts = np.zeros(match_counts.shape)
    # # score_threshold = 0.2
    # for idx, seq in enumerate(dbow_sequences):
    #     sequence = str(seq)
    #     if seq < 10:
    #         sequence = '0'+sequence
    #     ids_file = dbow2_directory + 'ids_sequence_' + sequence + '_n_split_' + str(dbow2_splits[idx]) + '_corrected.csv'
    #     norm_score_file = dbow2_directory + 'score_norm_sequence_' + sequence + '_n_split_' + str(dbow2_splits[idx]) + '_corrected.csv'
    #     ids = np.loadtxt(ids_file, delimiter=',')
    #     norm_scores = np.loadtxt(norm_score_file, delimiter=',')
    #     poses = np.loadtxt(data_save_path+'poses_' + sequence + '.csv', delimiter=',')

    #     for adx, alpha in enumerate(alphas):
    #         for ndx, n_best in enumerate(n_bests):

    #             ## Create the edges
    #             # E_idx = np.zeros((0,2))
    #             # match_count = 0
    #             # candidate_count = 0
    #             # for jdx in xrange(0, ids.shape[0]):
    #             #     for kdx in xrange(0, n_best): #ids.shape[1]):
    #             #         if norm_scores[jdx, kdx] >= alpha:
    #             #             edge_jk = (jdx + dbow2_splits[idx], ids[jdx, kdx])
    #             #             E_idx = np.vstack((E_idx, edge_jk))

    #             #             # Check for 'geometric match'
    #             #             p1 = poses[edge_jk[0],:]
    #             #             p2 = poses[edge_jk[1],:]
    #             #             if np.linalg.norm(p1-p2) <= match_dist_threshold:
    #             #                 match_count += 1
    #             #             candidate_count += 1

    #             #         else:
    #             #             break
    #             # # edge_sets.append(E_idx)
    #             # match_counts[idx, adx, ndx] = match_count
    #             # candidate_counts[idx, adx, ndx] = candidate_count
    #             # np.savetxt(dbow2_directory + 'edges/edges_dbow2_norm_sequence_' + sequence + 
    #             #     '_n_split_' + str(dbow2_splits[idx]) + '_threshold_'+str(alpha).replace('.','')+'_n_best_'+str(n_best)+'_corrected.csv', E_idx, delimiter=',')
    

    #             ## Downsample existing edge files
    #             load_str = dbow2_directory + 'edges/edges_dbow2_norm_sequence_' + sequence + \
    #                 '_n_split_' + str(dbow2_splits[idx]) + '_threshold_'+str(alpha).replace('.','')+'_n_best_'+str(n_best)+'_corrected.csv'
    #             E_idx = np.loadtxt(load_str, delimiter=',')
    #             np.savetxt(load_str.replace('_corrected.csv', '_downsample_1_corrected.csv'), E_idx, delimiter=',')
    #             n_poses = poses.shape[0]
    #             for downsample in downsample_list:
    #                 ind = np.arange(0,n_poses, downsample)
    #                 E_d = np.zeros((0,2))
    #                 for i in xrange(0,E_idx.shape[0]):
    #                     if E_idx[i,0] in ind and E_idx[i,1] in ind:
    #                         E_d = np.vstack((E_d, E_idx[i,:]))
    #                 np.savetxt(load_str.replace('_corrected.csv', '_downsample_'+str(downsample)+'_corrected.csv'), E_d, delimiter=',')


    # View precision/recall stuff
    # np.savetxt(dbow2_directory+'pr_data/match_counts_'+str(match_dist_threshold)+'.csv', match_counts, delimiter='.csv')
    # np.savetxt(dbow2_directory+'pr_data/candidate_counts_'+str(match_dist_threshold)+'.csv', candidate_counts, delimiter='.csv')

    # plt.figure()
    # plt.plot(alphas, match_counts.T, '-')
    # plt.plot(alphas, candidate_counts.T, '--')
    # plt.legend(('Match 00', 'Match 02', 'Match 05', 'Match 06', 'Match 08', 'Candidates 00', 'Candidates 02', 'Candidates 05', 'Candidates 06', 'Candidates 08'))

    # plt.figure()
    # plt.plot(alphas, match_counts.T/candidate_counts.T)
    # plt.legend(('00', '02', '05', '06', '08'))
    # plt.title('DBoW2 Precision, n_best = ' + str(n_best))
    # plt.grid()
    # plt.show()

    ## Visualize 
    # for idx, seq in enumerate(dbow_sequences):
    #     sequence = str(seq)
    #     if seq < 10:
    #         sequence = '0'+sequence
    #     poses = np.loadtxt(data_save_path+'poses_' + sequence + '.csv', delimiter=',')
    #     x = poses[:,0]
    #     y = poses[:,1]
    #     yaw = poses[:,3]
    #     E = edge_sets[idx]
    #     print("Sequence: ", sequence)
    #     vis_graph(x,y,yaw,E,n_split=dbow2_splits[idx], downsample=1)
    #     plt.axis('equal')
    #     plt.show()






    # ###########################################
    ## Visualize for the paper
    # Investigating sequence 06's jump in the results
    # Looks like the width of the road is between 15-20m, so cross-track edges get added
    # after a certain threshold
    from matplotlib import rc
    import matplotlib as mpl
    rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
    rc('text', usetex=True)
    mpl.rcParams.update({'font.size': 15})
    
    poses = np.loadtxt(data_save_path+'poses_06.csv', delimiter=',')
    x = poses[:,0]
    y = poses[:,1]
    yaw = poses[:,3]
    E05 = np.loadtxt(data_save_path+'edges_06_r_30_overlap_05.csv', delimiter=',')
    E04 = np.loadtxt(data_save_path+'edges_06_r_30_overlap_04.csv', delimiter=',')
    E03 = np.loadtxt(data_save_path+'edges_06_r_30_overlap_03.csv', delimiter=',')
    # vis_graph(x,y,yaw,E05,n_split=round(len(x)*0.55), downsample=5)
    # plt.axis('equal')


    # vis_graph(x,y,yaw,E03,n_split=round(len(x)*0.55), downsample=10)
    ax6, fig6 = vis_graph_paper(x,y,yaw,E04,n_split=round(len(x)*0.55), downsample=5)
    plt.xlim([-160, 305])
    plt.ylim([-22.5, 39])
    # plt.axis('equal')
    fig6.set_size_inches(12, 1.16, forward=True)
    plt.savefig('../../paper/figures/sequence_06_downsample5_overlap04.eps', 
                format='eps', bbox_inches='tight')
    # print(E04)

    # Seq 0
    poses0 = np.loadtxt(data_save_path+'poses_00.csv', delimiter=',')
    x0 = poses0[:,0]
    y0 = poses0[:,1]
    yaw0 = poses0[:,3]
    E04_0 = np.loadtxt(data_save_path+'edges_00_r_30_overlap_04.csv', delimiter=',')

    ax0, fig0 = vis_graph_paper(x0,y0,yaw0,E04_0, n_split=round(len(x0)*0.55), downsample=5)
    plt.axis('equal')
    fig0.set_size_inches(9.5, 9.5, forward=True)
    plt.legend(('Robot 1', 'Robot 2', 'Loop Closure\nCandidates'), loc='upper left')
    plt.savefig('../../paper/figures/sequence_00_downsample5_overlap04.eps', 
                format='eps', bbox_inches='tight')
    plt.show()
