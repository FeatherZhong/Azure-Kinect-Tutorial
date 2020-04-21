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
          - integrate_rgb_frames_for_fragment()

| 参数 | 含义 | 默认值 | 可选参数及含义 |
|  :----:  | ----  | :----: | ---- |
| n_frames_per_fragment | 创建fragments时将多少帧连续画面划分到一个fragment里 | 100 |
| n_keyframes_per_n_frame | 每多少帧抽取一个关键帧 | 5 |
| max_depth  | 超过此深度值的深度将被设为零  | 3  |
| max_depth_diff  | 最大深度差<br>Maximum depth difference to be considered as correspondence. In depth image domain, if two aligned pixels have a depth difference less than specified value, they are considered as a correspondence. Larger value induce more aggressive search, but it is prone to unstable result.  | 0.07 |
| optimization method   | pose graph 优化算法| GlobalOptimizationLevenbergMarquardt | GlobalOptimizationLevenbergMarquardt<br>GlobalOptimizationConvergenceCriteria|
| tsdf_cubic_size | a single voxel size for TSDF volume |  3.0 |  Lowering this value makes a high-resolution TSDF volume |

| 算法 | 用在哪了 | 含义 |
|  ----  | ----  | ---- |
| ORB 特征 | 若两帧不是连续帧，先求解帧之间RT | 计算帧之间的关键点匹配 |
| 5-point RANSAC | 若两帧不是连续帧，先求解帧之间RT | 求解帧之间的相机运动 |
| compute rgbd odometry  | 计算帧之间RT | 计算帧之间RT [[Park2017]](http://www.open3d.org/docs/release/tutorial/reference.html#park2017)  |
| RGBD integration 算法  | 生成每个fragment的3d mesh  |  一种利用rgbd图像和相机位姿RT生成3d mesh的算法[[Curless1996]](http://www.open3d.org/docs/release/tutorial/reference.html#curless1996) |


#### 2. Register fragments
This step align the fragments created from step1 in a global space. At first, Open3D read the `.ply` and `.json` file pairs which contain fragments and pose graphs created from step1, then we compute the alignment between any two fragments using the function `compute_initial_registration`.If two fragments are adjacent, the rough alignments is determined by an aggregating RGBD odometry obtained from step1 using [ICP registration](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#icp-registration), on the contrary, we call `register_point_cloud_fpfh` using RANSAC or [Fast global registration](http://www.open3d.org/docs/release/tutorial/Advanced/fast_global_registration.html#fast-global-registration) which is less reliable to perform global registration.

After we compute the transformations between fragments, the function `update_posegrph_for_scene` builds a pose graph for multiway registration of all fragments.Each graph node represents a fragment and its pose which transforms the geometry to the global space.

Once a pose graph is built the fuction `optimize_posegraph_for_scene` is called for multiway registration

**The code pipeline of - Register fragments:**
- run_system.py
  - register_fragments.py
    - run()
      - make_posegraph_for_scene()
        - register_point_cloud_pair()
          - preprocess_point_cloud()
          - compute_initial_registration()
            - if adjacent:ICP
            - if not adjacent:RANSAC or Fast global registration
        - update_posegrph_for_scene()
      - optimize_posegraph_for_scene()

| 参数 | 含义 | 默认值 | 可选参数及含义 |
|  :----:  | ----  | :----: | ---- |
| voxel_size  |  voxel_size defines the coarsest resolution for registration. You need to make sure the coarsest resolution is large enough, in order to assure a larger probability to converge to the global optimal. The usual practice is to start with a large number, and shrink it iteratively.[[解释]](https://github.com/intel-isl/Open3D/issues/1273#issuecomment-555125638) | 0.05  | 0.05/2^n 作者推荐n从0开始增加尝试  |
| global_registration  | 计算不相邻fragment之间的RT用到的方法  | ransac  |ransac [RANSAC](http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html#feature-matching)<br>fgr [Fast global registration](http://www.open3d.org/docs/release/tutorial/Advanced/fast_global_registration.html#fast-global-registration)<br>Fast global registration基于RANSAC但是计算速度快很多 |
| icp_method  | 计算相邻fragment之间的RT  |  color |  color[Colored point cloud registration](http://www.open3d.org/docs/release/tutorial/Advanced/colored_pointcloud_registration.html?highlight=registration_colored_icp#multi-scale-geometric-color-alignment)<br> point_to_point[Point-to-point ICP](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#point-to-point-icp)<br> point_to_plane[Point-to-plane ICP](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#point-to-plane-icp)  |

| 算法 | 用在哪了 | 含义 |
|  ----  | ----  | ---- |
| FPFH  | preprocess point cloud 下采样点云并提取特征，用于三维点云之间的配准 | 快速点特征直方图，三维点的一种特征，类似二维图像中的ORB特征。  |
| RANSAC   | 在此处用来计算不相邻fragment之间的RT  | 一种通过迭代获取最优模型算法思想  |
| Fast global registration | 计算不相邻fragment之间的RT  | 一种基于RANSAC但是计算速度快很多的点集对点集配准方法  |
| Point-to-point ICP | 计算相邻fragment之间的RT  | 一种ICP配准算法 [[beslandmckay1992]](http://www.open3d.org/docs/release/tutorial/reference.html#beslandmckay1992)  |
| Point-to-plane ICP | 计算相邻fragment之间的RT  | 一种ICP配准算法 [[ChenAndMedioni1992]](http://www.open3d.org/docs/release/tutorial/reference.html#chenandmedioni1992)  |
| Colored point cloud registration  |  计算相邻fragment之间的RT | 一种ICP配准算法[[Park2017]](http://www.open3d.org/docs/release/tutorial/reference.html#park2017)  |

#### 3. Refine registration
This step refine the fragments pairs detected from step2. At first Open3D read the pose graph created from step2, then we use function `local_refinement` to refine the transformation between fragments linked by edge using ICP .
After we refine the transformations between fragments, the function `update_posegrph_for_refined_scene` builds a pose graph for multiway registration of all fragments just like step2.

Once a pose graph is built, function `optimize_posegraph_for_scene` is called for multiway registration.



**The code pipeline of - Refine registration:**
- run_system.py
  - refine_registration.py
    - run()
      - make_posegraph_for_refined_scene()
        - register_point_cloud_pair()
          - local_refinement()
            - multiscale_icp()
      - optimize_posegraph_for_refined_scene()

| 参数 | 含义 | 默认值 | 可选参数及含义 |
|  :----:  | ----  | :----: | ---- |
| icp_method  | 计算fragment之间的RT  |  color |  color[Colored point cloud registration](http://www.open3d.org/docs/release/tutorial/Advanced/colored_pointcloud_registration.html?highlight=registration_colored_icp#multi-scale-geometric-color-alignment)<br> point_to_point[Point-to-point ICP](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#point-to-point-icp)<br> point_to_plane[Point-to-plane ICP](http://www.open3d.org/docs/release/tutorial/Basic/icp_registration.html#point-to-plane-icp)  |


| 算法 | 用在哪了 | 含义 |
|  ----  | ----  | ---- |
| Point-to-point ICP | 计算相邻fragment之间的RT  | 一种ICP配准算法 [[beslandmckay1992]](http://www.open3d.org/docs/release/tutorial/reference.html#beslandmckay1992)  |
| Point-to-plane ICP | 计算相邻fragment之间的RT  | 一种ICP配准算法 [[ChenAndMedioni1992]](http://www.open3d.org/docs/release/tutorial/reference.html#chenandmedioni1992)  |
| Colored point cloud registration  |  计算相邻fragment之间的RT | 一种ICP配准算法[[Park2017]](http://www.open3d.org/docs/release/tutorial/reference.html#park2017)  |
#### 4. Integrate scene
This step Integrate all RGBD images into a single TSDF volume. At first open3d read the fragment pose graphs created from steps above, then for each fragment we compute the frames associated with it. For each frames we compute camera pose $P_{c}$ by $P_{c}=P_{fragment}*P_{rgbd}$, and integrate into TSDF volume uing function `volume.integrate`.
Finally, open3d extract a mesh and trajectory log as the result.


**The code pipeline of - Integrate scene:**
- run_system.py
  - integrate_scene.py
    - run()
      - scalable_integrate_rgb_frames()
      - read_pose_graph()
      - extract_triangle_mesh()
      - write_poses_to_log()


| 参数 | 含义 | 默认值 | 可选参数及含义 |
|  :----:  | ----  | :----: | ---- |
| tsdf_cubic_size | a single voxel size for TSDF volume  |  3.0 |  Lowering this value makes a high-resolution TSDF volume |

| 算法 | 用在哪了 | 含义 |
|  ----  | ----  | ---- |
| RGBD integration 算法  | 最终生成整个重建场景的3d mesh  |  一种利用rgbd图像和相机位姿RT生成3d mesh的算法[[Curless1996]](http://www.open3d.org/docs/release/tutorial/reference.html#curless1996) |
