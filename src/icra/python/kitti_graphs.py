import numpy as np
from numpy import cos, sin
import cv2
# Prevents cv2 bugs (3.10.0 problem)
cv2.ocl.setUseOpenCL(False)
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import pykitti
from scipy import spatial
from shapely.geometry import Polygon
import networkx as nx
from networkx import bipartite

basedir = '/drive1/Datasets/kitti'


def get_fov_poly(x, y, t, r=30.0, fov=np.pi/2, n=10):
    ext = [(x,y)]
    d_fov = fov/(n-1)
    for idx in xrange(0, n):
        dx = r*cos(t - fov/2 + idx*d_fov)
        dy = r*sin(t - fov/2 + idx*d_fov)
        ext.append((x+dx, y+dy))
    ext.append((x,y))
    return Polygon(ext)

def downsample_edges(E, ind):
    E_out = np.zeros((0,2))
    for i in xrange(0,E.shape[0]):
        if E[i,0] in ind and E[i,1] in ind:
            E_out = np.vstack((E_out, E[i,:]))
    return E_out

def vis_graph(x, y, yaw, E, n_split=-1, downsample=1):
    ax, fig = plot_trajectory(x,y,yaw, n_split=n_split, downsample=downsample)
    ind = np.arange(0, len(x), downsample)
    E_down = downsample_edges(E, ind)
    for idx in xrange(0, E_down.shape[0]):
        ax.plot(x[E_down[idx,:].astype(int)], y[E_down[idx,:].astype(int)], 'g-')
    return ax, fig

def vis_graph_paper(x, y, yaw, E, n_split=-1, downsample=1):
    ax, fig = plot_trajectory_paper(x,y,n_split, downsample)
    ind = np.arange(0, len(x), downsample)
    E_down = downsample_edges(E, ind)
    for idx in xrange(0, E_down.shape[0]):
        ax.plot(x[E_down[idx,:].astype(int)], y[E_down[idx,:].astype(int)], 'g-', linewidth=0.6, alpha=0.6)
    return ax, fig

def meters_deglon(ang):
  ''' Takes latitude as argument ang.
  '''
  return 111415.13*cos(ang) - 94.55*cos(3.0*ang) + (0.12 * cos(5.0*ang)) 

def meters_deglat(ang):
  ''' Takes latitude as argument ang.
  '''
  return 111132.09 - 566.05*cos(2.0*ang) + 1.20*cos(4.0*ang) - 0.002*cos(6.0*ang)


def plot_traj(x,y,yaw,downsample):
    plt.figure()
    plt.plot(x,y, 'b--')
    ax = plt.axes()
    for i in range(0, len(yaw), downsample):
        ax.arrow(x[i],y[i], cos(yaw[i]), sin(yaw[i]), 
            head_width=0.1, head_length=0.15, fc='r', ec='r')
    plt.show()

def lat_lon_to_xy(lat, lon, x0=0.0, y0=0.0):
  ''' Assumes lat, lon are in radians already.
  '''
  # Note that latitude is used for both as the Earth is modelled as symmetric
  # about its rotational axis 
  dlon = lon - lon[0]
  dlat = lat - lat[0]
  x = dlon*meters_deglon(lat)*180.0/np.pi + x0
  y = dlat*meters_deglat(lat)*180.0/np.pi + y0
  return x, y

def get_combined_global_trajectory_and_weights(dates, drives, fastThreshold=150):
    ''' Combine multiple drives using the OXTS GPS lat/lon/alt measurements. Yaw
    is sloppily assuming flat orientation (i.e. small roll and pitch) for now.
    '''
    lat = []
    lon = []
    alt = []
    yaw = []
    w = []
    orb = cv2.ORB_create(nfeatures=10000, fastThreshold=fastThreshold)
    for date, drive in zip(dates, drives):
        data = pykitti.raw(basedir, date, drive, imformat='cv2')
        for img, p in zip(data.cam0, data.oxts):
            lat.append(p[0].lat)
            lon.append(p[0].lon)
            # Assumes a small enough region 
            alt.append(p[0].alt)
            # find the keypoints with ORB
            kp = orb.detect(img,None)
            w.append(len(kp))
            yaw.append(p[0].yaw)

    x,y = lat_lon_to_xy(np.pi*np.array(lat)/180.0, np.pi*np.array(lon)/180.0)
    alt = np.array(alt)
    alt = alt - alt[0]
    yaw = np.array(yaw)
    return x, y, alt, yaw, np.array(w)

def plot_drive_trajectory(date, drive, downsample=1):
    ''' Plot a single drive's trajectory using the local frame odometry of the
    poses in KITTI.
    '''
    data = pykitti.raw(basedir, date, drive)
    x = []
    y = []
    z = []
    yaw = []
    for p in data.oxts:
        # print len(p)
        # print p[0]
        T = p[1]
        x.append(T[0,3])
        y.append(T[1,3])
        z.append(T[2,3])
        yaw.append(np.arctan2(-T[0,1], T[0,0]))
    plot_traj(x,y,yaw,downsample)

def plot_trajectory_paper(x, y, n_split=-1, downsample=1):
    fig = plt.figure()
    inds = np.arange(0, len(x), downsample)
    plt.plot(x[0:n_split], y[0:n_split], 'b-', linewidth=2)
    plt.plot(x[n_split:-1], y[n_split:-1], 'r-', linewidth=2)
    ax = plt.axes()
    # plt.grid()

    return ax, fig

def plot_trajectory(x,y,yaw, z=None, n_split=-1, downsample=1):
    fig = plt.figure()
    inds = np.arange(0, len(x), downsample)
    if z == None:
        if n_split == -1:
            # plt.plot(x[inds],y[inds], 'b--')
            pass
        else:
            plt.plot(x[0:n_split], y[0:n_split], 'b--')
            plt.plot(x[n_split:-1], y[n_split:-1], 'k-.')
        ax = plt.axes()
        for i in range(0, len(yaw), downsample):
            c = 'b'
            if n_split != -1:
                if i >= n_split:
                    c = 'r'
            ax.arrow(x[i],y[i], 4*cos(yaw[i]), 4*sin(yaw[i]), 
                     head_width=2, head_length=3, fc=c, ec=c)
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        # xlabel('normalized BoW L$_1$ score threshold $\alpha$','FontSize',20,'interpreter','latex');
        # ylabel('communication cost (kB)','FontSize',20,'interpreter','latex');
        # set(gca, ...
        #    'Box'         , 'on'     , ...
        #    'TickDir'     , 'out'     , ...
        #    'XMinorTick'  , 'on'     , ...
        #    'YMinorTick'  , 'on'      , ...
        #    'YGrid'       , 'on'      , ...
        #    'XGrid'       , 'on'      , ...
        #    'XColor'      , [.3 .3 .3], ...
        #    'YColor'      , [.3 .3 .3], ...
        #    'LineWidth'   , 1.3         );

    else:
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x,y,z)
        inds = range(0, len(yaw), 10)
        ax.quiver(x[inds], y[inds], z[inds], cos(yaw[inds]), sin(yaw[inds]), 0*yaw[inds])

    return ax, fig

def preserve_bipartite(G, new_edge):
    G.add_edge(new_edge[0], new_edge[1])
    if bipartite.is_bipartite(G):
        return True
    else:
        G.remove_edge(new_edge[0], new_edge[1])
        return False

def get_edges_from_traj_and_weights(x, y, yaw, w, r=30, fov=np.pi/2, n_poly=10, overlap_min=0.3, 
                                    n_split=-1, ind_dist=150, use_fov=True, check_bipartite=True):
    # Create graph with poses that contain some overlapping FOV  
    xy_data = np.vstack((x, y)).T
    # print("KD tree data shape: {:}".format(xyz_data.shape))
    kd_xy = spatial.KDTree(xy_data)
    # Get edges within r
    if use_fov:
        res_list = kd_xy.query_ball_tree(kd_xy, 2*r)
    else:
        res_list = kd_xy.query_ball_tree(kd_xy, r)
    edges = np.zeros((0,2))

    if check_bipartite:
        G = nx.Graph()

    if use_fov:
        fov_dict = {}
        area_min = overlap_min*r**2*fov/2
        for idx, l in enumerate(res_list):
            if idx in fov_dict:
                poly_idx = fov_dict[idx]
            else:
                poly_idx = get_fov_poly(x[idx], y[idx], yaw[idx], r=r, fov=fov, n=n_poly)
                fov_dict[idx] = poly_idx
            for jdx in l:
                # Only want loop overlap to be included
                if idx < jdx:
                    if n_split != -1:
                        test_condition = ((idx <= n_split and jdx > n_split) or (idx > n_split and jdx <= n_split))
                    else:
                        test_condition = (np.abs(idx -jdx) > ind_dist)

                    if test_condition:
                        if jdx in fov_dict:
                            poly_jdx = fov_dict[jdx]
                        else: 
                            poly_jdx = get_fov_poly(x[jdx], y[jdx], yaw[jdx], r=r, fov=fov, n=n_poly)
                            fov_dict[jdx] = poly_jdx

                        if poly_jdx.intersection(poly_idx).area >= area_min:
                            if check_bipartite:
                                if not preserve_bipartite(G, (idx, jdx)):
                                    continue
                            edges = np.vstack((edges, (idx, jdx)))

    else:
        for idx, l in enumerate(res_list):
            for jdx in l:
                if idx < jdx:
                    if n_split != -1:
                        test_condition = ((idx <= n_split and jdx > n_split) or (idx > n_split and jdx <= n_split))
                    else:
                        test_condition = (np.abs(idx -jdx) > ind_dist)
                    if test_condition:
                        if check_bipartite:
                                if not preserve_bipartite(G, (idx, jdx)):
                                    continue
                        edges = np.vstack((edges, (idx, jdx)))
    return edges


def gen_scan_exchange_graph(date, drive, r=30, fov=np.pi/2, fastThreshold=150, 
                            n_poly=10, overlap_min=0.3, n_split=-1, ind_dist=150):
    data = pykitti.raw(basedir, date, drive, imformat='cv2')
    w = []
    x = []
    y = []
    z = []
    yaw = []

    # n_cam = sum(1 for _ in data.cam0)
    # n_oxts = sum(1 for _ in data.oxts)
    # print("{:} cam images, {:} oxts packets".format(n_cam, n_oxts))
    for img, p in zip(data.cam0, data.oxts):
        # Reasonable fastThreshold values in range [20, 150]
        orb = cv2.ORB_create(nfeatures=10000, fastThreshold=fastThreshold)
        # find the keypoints with ORB
        kp = orb.detect(img,None)
        w.append(len(kp))
        T = p[1]
        x.append(T[0,3])
        y.append(T[1,3])
        z.append(T[2,3])
        # First attempt at yaw
        # yaw.append(np.arctan2(-T[0,1], T[0,0]))
        # Second yaw (directly from GPS, I believe? But given as RPY, not true heading,
        # would need to project)
        # THIS ONE WORKS FINE
        yaw.append(p[0].yaw)
        # Third yaw: heading with North and East velocity components
        # yaw.append(np.arctan2(p[0].vn, p[0].ve))

    # Create graph with r in KD tree
    # xyz_data = np.vstack((x, y, z)).T
    # print("KD tree data shape: {:}".format(xyz_data.shape))
    # kd_xyz = spatial.KDTree(xyz_data)

    # # Get edges within r
    # res_list = kd_xyz.query_ball_tree(kd_xyz, r)
    # edges = np.zeros((0,2))
    # for idx, l in enumerate(res_list):
    #     for jdx in l:
    #         # Only want loop overlap to be included
    #         if idx < jdx and np.abs(idx - jdx) > 500:
    #             edges = np.vstack((edges, (idx, jdx))) 

    edges = get_edges_from_traj_and_weights(x, y, yaw, w, r=r, fov=fov, n_poly=n_poly, 
                            overlap_min=overlap_min, n_split=n_split, ind_dist=ind_dist)

    return edges, np.array(w), x, y, z, yaw

if __name__=='__main__':

    ## Test datasets
    date_loop = '2011_09_30'
    drive_loop = '0020'
    # date1 = '2011_09_26'
    # drive1 = '0061'
    # data = pykitti.raw(basedir, date1, drive1, imformat='cv2')

    # plot_drive_trajectory(date_loop, drive_loop)
    drive_list1 = ['0027', '0028']
    drive_list2 = ['0033', '0034']
    date_list = ['2011_09_30']*len(drive_list2)
    # x, y, alt, yaw, w = get_combined_global_trajectory_and_weights(date_list, drive_list2,
    #                                                                 fastThreshold=150)

    # poses = np.vstack((x,y,alt,yaw)).T
    # np.savetxt('./data/poses_kitti_2011_09_26_0033_0034.txt', poses, delimiter=',')
    # np.savetxt('./data/weights_kitti_2011_09_26_0033_0034_thresh_150.txt', w, delimiter=',')
    
    # E, W, x, y, z, yaw = gen_scan_exchange_graph(date_loop, drive_loop, 
    #                                         dist_min=2, ang_min=np.pi/12, fastThreshold=150)

    # np.savetxt('../data/edges_kitti_2011_09_26_0020_dist_2_thresh_150.txt', E, delimiter=',')
    # np.savetxt('../data/weights_kitti_2011_09_26_0020_dist_2_thresh_150.txt', W, delimiter=',')
    # poses = np.vstack((x,y,z,yaw)).T
    # np.savetxt('../data/poses_kitti_2011_09_26_0020_dist_2_thresh_150.txt', poses, delimiter=',')



    ## Try FOV for 0027 nd 0028 - not enough overlap!! Just 28 overlaps nicely
    # Get FOV scan exchange graph for drive_list1
    r = 30.0
    fov = np.pi/2
    n_diff = 150
    n_poly = 10
    overlap_min = 0.8
    
    # data = pykitti.raw(basedir, date_list[0], drive_list1[0])
    # n_split = 0
    # for im in data.cam0:
    #     n_split += 1
    # print("N split: {:}".format(n_split))
    
    # poses = np.loadtxt('./data/poses_kitti_2011_09_26_0027_0028.txt', delimiter=',')
    # W = np.loadtxt('./data/weights_kitti_2011_09_26_0027_0028_thresh_150.txt', delimiter=',')
    # E = get_edges_from_traj_and_weights(poses[:,0], poses[:,1], poses[:,3], W, n_split, r=r, fov=fov, n_poly=n_poly, overlap_min=overlap_min)
    # np.savetxt('../data/edges_kitti_2011_09_26_0027_0028_r_30_overlap_01.txt', E, delimiter=',')
    # E = np.loadtxt('../data/edges_kitti_2011_09_26_0027_0028_r_30_overlap_01.txt', delimiter=',')
    # E = np.loadtxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_08.txt', delimiter=',')
    
    ## Try just drive 0028 for different overlap_min values
    # E, w, x, y, z, yaw = gen_scan_exchange_graph(date_list[0], drive_list1[1], r=r, fov=fov, fastThreshold=150, 
    #                         n_poly=n_poly, overlap_min=overlap_min, n_split=-1, ind_dist=n_diff)

    # np.savetxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_08.txt', E, delimiter=',')
    # np.savetxt('../data/weights_kitti_2011_09_26_0028_thresh_150.txt', w, delimiter=',')

    overlap_min_list = np.linspace(0.2, 0.9, 8)
    # poses = np.vstack((x,y,z,yaw)).T
    W = np.loadtxt('../data/weights_kitti_2011_09_26_0028_thresh_150.txt', delimiter=',')
    poses = np.loadtxt('../data/poses_kitti_2011_09_26_0028.txt', delimiter=',')
    # for om in overlap_min_list:
    #     E = get_edges_from_traj_and_weights(poses[:,0], poses[:,1], poses[:,3], W, r=r, fov=fov, 
    #                                         n_poly=n_poly, overlap_min=om, ind_dist=n_diff)
    #     np.savetxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_'+str(om).replace('.','')+'.txt', E, delimiter=',')
    # np.savetxt('../data/poses_kitti_2011_09_26_0028_xy_heading.txt', poses, delimiter=',')
    # Vis the graph and poses


    ## Get downsampled versions of overlap_min = 0.5    
    # downsample_rates = [2,5,10,20]
    E = np.loadtxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_05.txt', delimiter=',')
    # n_poses = poses.shape[0]
    # for rate in downsample_rates:
    #     ind = np.arange(0,n_poses, rate)
    #     # poses_idx = poses[ind,:]
    #     # w_idx = poses[ind,:]
    #     E_idx = np.zeros((0,2))
    #     for i in xrange(0,E.shape[0]):
    #         if E[i,0] in ind and E[i,1] in ind:
    #             E_idx = np.vstack((E_idx, E[i,:]))
    #     np.savetxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_05_downsample_'+str(rate)+'.txt', E_idx, delimiter=',')


    ## Try distance ONLY (no FOV overlap computation) 
    # rad_list = [5,10,15,20,25,30,40,50,60,80]
    # for rad in rad_list:
    #     E = get_edges_from_traj_and_weights(poses[:,0], poses[:,1], poses[:,3], W, r=rad, fov=fov, 
    #                                         n_poly=n_poly, ind_dist=n_diff, use_fov=False)
    #     np.savetxt('../data/edges_kitti_2011_09_26_0028_r_30_overlap_05_rad_only_'+str(rad)+'.txt', E, delimiter=',')


    # plot_drive_trajectory(date_list[0], '0034', downsample=10)

    ## TODO: add 'match number' to edges! 
    vis_graph(poses[:,0], poses[:,1], poses[:,3], E, downsample=10)
    plt.show()
    # ax, fig = vis_graph(poses[:,0], poses[:,1], poses[:,3], E)
    # plt.show()
