### Open3D pipeline

The reconstruction system pipeline of Open3D

#### 1. Make fragments

This step creates fragments from n_frames_per_fragment images. Before all, Open3D will first create pose graph for aligning point cloud from (or RGBD) frames. The pose graph contains nodes and edges. The node is a piece of geometry $P_{i}$ associated with a pose matrix $T_{i}$ which transforms $P_{i}$ into the global space. The set $T_{i}$ are the unknown variables to be optimized.

When create the pose graph, the `register_one_rgbd_pair()` function is used to compute the transformation matrix between two frames. For the non-adjacent RGBD frames, the function `pose_estimation()` which computes OpenCV ORB feature and performs 5-point RANSAC to estimate a rough alignment (transformation), which is used as the initialization of `compute_rgbd_odometry()`. For the adjacent RGBD frames, an identity matrix is used as initialization. Once a pose graph is created, multi-way registration is performed by calling function `optimize_posegraph_for_fragment()`. The function calls `global_optimization()` to estimate poses of the RGBD frames. For efficiency, only key frames are used.

**More details about pose graph of multi-way registration:**

[[Choi2015\]](http://www.open3d.org/docs/release/tutorial/reference.html#choi2015) has observed that pairwise registration is error-prone. False pairwise alignments can outnumber correctly aligned pairs. Thus, they partition pose graph edges into two classes. **Odometry edges** connect temporally close, neighboring nodes. A local registration algorithm such as ICP can reliably align them. **Loop closure edges** connect any non-neighboring nodes. The alignment is found by [global registration](http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html#global-registration) and is less reliable.

**The code pipeline of - Make fragments:**
- run_system.py
  - make_fragments.py
    - run()
      - process_single_fragment()
      - make_posegraph_for_fragment()
      - optimize_posegraph_for_fragment()
      - make_pointcloud_for_fragment()


#### 2. Register fragments
This step align the fragments created from step1 in a global space. At first, Open3D read the `.ply` and `.json` file pairs which contain fragments and pose graphs, then we compute the alignment between any two fragments using the function `compute_initial_registration`.If two fragments are adjacent, the rough alignments is determined by an aggregating RGBD odometry obtained from step1 using [ICP registration](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#icp-registration), on the contrary, we call `register_point_cloud_fpfh` using RANSAC or [Fast global registration](http://www.open3d.org/docs/release/tutorial/Advanced/fast_global_registration.html#fast-global-registration) which is less reliable to perform global registration.

After we compute the transformations between fragments, the function `update_posegrph_for_scene` builds a pose graph for multiway registration of all fragments.The nodes is fragments which is typically geometries (e.g., point clouds or RGBD images)  $\{P_{i}\}$ associated with a pose matrix $\{T_{i}\}$ which transforms $\{P_{i}\}$ into the global space.

Once a pose graph is built the fuction `optimize_posegraph_for_scene` is called for multiway registration

**The code pipeline of - Register fragments:**
- run_system.py
  - register_fragments.py
    - run()
      - make_posegraph_for_scene()
        - register_point_cloud_pair()
          - compute_initial_registration()
            - if adjacent:ICP
            - if not adjacent:RANSAC or Fast global registration
        - update_posegrph_for_scene()
      - optimize_posegraph_for_scene()

#### 3. Refine registration

#### 4. Integrate scene
