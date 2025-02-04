#ifndef LIDAR_SELECTION_H_
#define LIDAR_SELECTION_H_

#include <common_lib.h>
#include <vikit/abstract_camera.h>
#include <frame.h>
#include <map.h>
#include <feature.h>
#include <point.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

namespace lidar_selection {

class LidarSelector {
  public:
    int grid_size;
    vk::AbstractCamera *cam;
    SparseMap *sparse_map;
    StatesGroup *state;
    StatesGroup *state_propagat;
    M3D Rli, Rci, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
    V3D Pli, Pci, Pcw;
    int *align_flag;
    int *grid_num;      //; 网格里面存储的3D点的类型，如果没有点是UNKNOWN，地图点是TYPE_MAP，
    int *map_index;
    float *map_dist;    //; 网格中深度最近的点的深度，是在for循环遍历中不断变小的值
    float *map_value;   //; 网格中角点的 shiTomasiScore 得分的最大值，是在for循环遍历中不断变大的值
    float *patch_cache; //; 当前帧图像中的某个图像点周围的patch像素，每个点都不一样
    float *patch_with_border_;
    //; 图像宽，图像高，划分网格之后横向有几个网格，划分网格只有纵向有几个网格，网格的总数
    int width, height, grid_n_width, grid_n_height, length;

    //; 存储当前帧子地图的最终结果，主要就是和当前帧patch匹配的历史帧的patch
    SubSparseMap *sub_sparse_map;  

    double fx, fy, cx, cy;
    bool ncc_en;
    int debug, patch_size, patch_size_total, patch_size_half;
    int count_img, MIN_IMG_COUNT;
    int NUM_MAX_ITERATIONS;
    vk::robust_cost::WeightFunctionPtr weight_function_;
    float weight_scale_;
    double img_point_cov, outlier_threshold, ncc_thre;
    size_t n_meas_; //!< Number of measurements
    deque<PointPtr> map_cur_frame_;      //; 这个变量定义了实际没有使用
    deque<PointPtr> sub_map_cur_frame_;  //; 当前帧最终用到的地图点
    double computeH, ekf_time;
    double ave_total = 0.01;
    int frame_cont = 1;
    vk::robust_cost::ScaleEstimatorPtr scale_estimator_;

    Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H; // 18*18
    MatrixXd H_sub, K;
    cv::flann::Index Kdtree;
    /* add by crz */
    int eigenValueThreshold;

    LidarSelector(const int grid_size, SparseMap* sparse_map);

    ~LidarSelector();

    void detect(cv::Mat img, PointCloudXYZI::Ptr pg);
    float CheckGoodPoints(cv::Mat img, V2D uv);
    void addFromSparseMap(cv::Mat img, PointCloudXYZI::Ptr pg);
    void addSparseMap(cv::Mat img, PointCloudXYZI::Ptr pg);
    void FeatureAlignment(cv::Mat img);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void init();
    void getpatch(cv::Mat img, V3D pg, float* patch_tmp, int level);
    void getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level);
    void dpi(V3D p, MD(2,3)& J);
    float UpdateState(cv::Mat img, float total_residual, int level);
    double NCC(float* ref_patch, float* cur_patch, int patch_size);//ncc全称叫做 归一化互相关

    void ComputeJ(cv::Mat img);
    void reset_grid();
    void addObservation(cv::Mat img);
    void reset();
    bool initialization(FramePtr cur_frame, PointCloudXYZI::Ptr pg);   
    void createPatchFromPatchWithBorder(float* patch_with_border, float* patch_ref);
    void getWarpMatrixAffine(
      const vk::AbstractCamera& cam,
      const Vector2d& px_ref,
      const Vector3d& f_ref,
      const double depth_ref,
      const SE3& T_cur_ref,
      const int level_ref,    // px_ref对应特征点的金字塔层级
      const int pyramid_level,
      const int halfpatch_size,
      Matrix2d& A_cur_ref);
    bool align2D(
      const cv::Mat& cur_img,
      float* ref_patch_with_border,
      float* ref_patch,
      const int n_iter,
      Vector2d& cur_px_estimate,
      int index);
    void AddPoint(PointPtr pt_new);
    int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level);
    void display_keypatch(double time);
    void updateFrameState(StatesGroup state);
    V3F getpixel(cv::Mat img, V2D pc);

    void warpAffine(
      const Matrix2d& A_cur_ref,
      const cv::Mat& img_ref,
      const Vector2d& px_ref,
      const int level_ref,
      const int search_level,
      const int pyramid_level,
      const int halfpatch_size,
      float* patch);
    
    PointCloudXYZI::Ptr Map_points;
    PointCloudXYZI::Ptr Map_points_output;
    PointCloudXYZI::Ptr pg_down;
    pcl::VoxelGrid<PointType> downSizeFilter;
    unordered_map<VOXEL_KEY, VOXEL_POINTS *> feat_map;  //; 这个feat_map就是整个视觉地图，通过hash_key索引对应的体素
    unordered_map<VOXEL_KEY, float> sub_feat_map; //; 当前帧图像用到的子地图，每次都会重新构造
    //; 当前帧图像找到的地图点patch所在图像id，和这个图像跟当前帧图像之间的affine变换，每次都会重新构造
    unordered_map<int, Warp *> Warp_map; 

    vector<VOXEL_KEY> occupy_postions;
    set<VOXEL_KEY> sub_postion;
    vector<PointPtr> voxel_points_;   //; 当前帧图像用到的子地图的3D点，是网格中深度最近的那个点，每次都会重新构造
    vector<V3D> add_voxel_points_;    //; 当前帧图像加入的新的地图点，是前帧图像新观测到的patch，每次都会重新构造

    cv::Mat img_cp, img_rgb;
    std::vector<FramePtr> overlap_kfs_;
    FramePtr new_frame_;
    FramePtr last_kf_;
    Map map_;
    enum Stage {
      STAGE_FIRST_FRAME,
      STAGE_DEFAULT_FRAME
    };
    Stage stage_;
    enum CellType {
      TYPE_MAP = 1,
      TYPE_POINTCLOUD,
      TYPE_UNKNOWN
    };

  private:
    struct Candidate 
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      PointPtr pt;       
      Vector2d px;    
      Candidate(PointPtr pt, Vector2d& px) : pt(pt), px(px) {}
    };
    typedef std::list<Candidate > Cell;
    typedef std::vector<Cell*> CandidateGrid;

    /// The grid stores a set of candidate matches. For every grid cell we try to find one match.
    struct Grid
    {
      CandidateGrid cells;
      vector<int> cell_order;
      int cell_size;
      int grid_n_cols;
      int grid_n_rows;
      int cell_length;
    };

    Grid grid_;
};
  typedef boost::shared_ptr<LidarSelector> LidarSelectorPtr;

} // namespace lidar_detection

#endif // LIDAR_SELECTION_H_