# cslam-resource
Resource efficient/adaptive communication policies for multi-robot cooperative SLAM. This is joint work between the [Aerospace Controls Lab](http://acl.mit.edu/) at MIT and [STARS Lab](http://www.starslab.ca/) at UofT. If you use any of this code in your work, please cite the relevant publication: 

```bibtex
@article{giamou2017talk,
  title   = {Talk Resource-Efficiently to Me: 
             Optimal Communication Planning for Distributed SLAM Front-Ends},
  author  = {Giamou, Matthew and Khosoussi, Kasra and How, Jonathan P},
  journal = {IEEE} Int. Conf. Robot. Autom. ({ICRA})},
  year    = {2018}
}

@article{tian2018near,
  title   = {Near-Optimal Budgeted Data Exchange for Distributed Loop Closure Detection},
  author  = {Tian, Yulun and Khosoussi, Kasra and Giamou, Matthew and How, Jonathan P and Kelly, Jonathan},
  journal = {Robotics: Science and Systems},
  year    = {2018}
}
```

## Installation and Dependencies 

All code and experiments were developed and run on Ubuntu 14.04 with MATLAB R2017b, Python 2.7.6, and GCC/G++ 4.8.4.

### MATLAB 
Sergii Iglin's [grTheory toolbox](https://www.mathworks.com/matlabcentral/fileexchange/4266-grtheory-graph-theory-toolbox) is used, as well as Guillaume Jacquenot's [polygon intersection code](https://www.mathworks.com/matlabcentral/fileexchange/18173-polygon-intersection). Both packages are included in the source code (`src/icra/matlab`).

### Python
Python is used to parse [KITTI odometry benchmark data](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) and create the graph instances tested in our paper. We have included these graph instances in this repository (`src/icra/data/odometry`) but also include the code that generates them. This code depends on the following packages:

- [NumPy and SciPy](https://www.scipy.org/scipylib/download.html)
- [Matplotlib](https://matplotlib.org/)
- [Shapely](https://toblerity.org/shapely/index.html)
- [NetworkX](https://networkx.github.io/)
- [OpenCV](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_tutorials.html)
- [PyKITTI](https://github.com/utiasSTARS/pykitti)

### C++
To generate sample measurement exchange graphs, we used [DBoW2](https://github.com/dorian3d/DBoW2). The C++ code used to 
generate the graphs in our paper is included but not needed to run the core optimal communication algorithms. 

## Usage 

### Optimal Data Exchange Policies
The code for our [ICRA 2018 paper](https://arxiv.org/abs/1709.06675) listed above is in the folder `src/icra`. The core functionality is the simple Matlab function `src/icra/matlab/solve_odep.m`. This function can be called on a weighted graph by passing in a set of vertex pairs representing edges as an `(N, 2)` matrix and a size `N` vector of vertex weights. The Matlab [Gurobi](http://www.gurobi.com/) interface was used to solve the ODEP problem instances as it is faster than the built-in Matlab `linprog` function. If you have access to Gurobi, `solve_odep.m` takes a third optional boolean argument that indicates the function should use Gurobi if true.

To generate the plots in our paper, run the script `src/icra/matlab/generate_icra2018_paper_plots.m`. This script uses the `.mat` files in `src/icra/data/`, which can be re-generated with the script `src/icra/matlab/generate_icra2018_paper_results.m`. Run the scripts from within `src/icra` to use the file paths as they are currently written.

See the scripts `src/icra/matlab/simple_example_graph.m` and `src/icra/matlab/simple_random_example_graph.m` for small examples that use `solve_odep.m` on toy problems. 

### Budgeted Data Exchange Policies
The code for our [RSS 2018 paper](https://arxiv.org/abs/1806.00188) listed above is in the folder `src/rss`. To reproduce the plots run `src/rss/matlab/generate_KITTI_plot.m`, which uses data generated by `src/rss/matlab/generate_KITTI_result.m`. 
