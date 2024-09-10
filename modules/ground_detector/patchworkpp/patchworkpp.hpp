#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <mutex>
#include <numeric>
#include <queue>

#define MARKER_Z_VALUE -2.2
#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

using namespace std;

struct GroundParams
{
    bool verbose;
    bool enable_RNR;
    bool enable_RVPF;
    bool enable_TGR;

    int num_iter;
    int num_lpr;
    int num_min_pts;
    int num_zones;
    int num_rings_of_interest;

    double RNR_ver_angle_thr;
    double RNR_intensity_thr;

    double sensor_height;
    double th_seeds;
    double th_dist;
    double th_seeds_v;
    double th_dist_v;
    double max_range;
    double min_range;
    double uprightness_thr;
    double adaptive_seed_selection_margin;
    double intensity_thr;

    vector<int> num_sectors_each_zone;
    vector<int> num_rings_each_zone;

    int max_flatness_storage;
    int max_elevation_storage;

    vector<double> elevation_thr;
    vector<double> flatness_thr;


    GroundParams()
    {
        verbose     = false;
        enable_RNR  = true;
        enable_RVPF = true;
        enable_TGR  = true;

        num_iter = 3;    // Number of iterations for ground plane estimation using PCA.
        num_lpr  = 20;   // Maximum number of points to be selected as lowest points representative.
        num_min_pts =
            10;          // Minimum number of points to be estimated as ground plane in each patch.
        num_zones = 4;   // Setting of Concentric Zone Model(CZM)
        num_rings_of_interest =
            4;   // Number of rings to be checked with elevation and flatness values.

        RNR_ver_angle_thr = -15.0;   // Noise points vertical angle threshold. Downward rays of
                                     // LiDAR are more likely to generate severe noise points.
        RNR_intensity_thr = 0.2;     // Noise points intensity threshold. The reflected points have
                                     // relatively small intensity than others.

        sensor_height = 1.723;
        th_seeds = 0.125;     // threshold for lowest point representatives using in initial seeds
                              // selection of ground points.
        th_dist    = 0.125;   // threshold for thickenss of ground.
        th_seeds_v = 0.25;    // threshold for lowest point representatives using in initial seeds
                              // selection of vertical structural points.
        th_dist_v = 0.1;      // threshold for thickenss of vertical structure.
        max_range = 80.0;     // max_range of ground estimation area
        min_range = 2.7;      // min_range of ground estimation area
        uprightness_thr =
            0.707;   // threshold of uprightness using in Ground Likelihood Estimation(GLE). Please
                     // refer paper for more information about GLE.
        adaptive_seed_selection_margin = -1.2;   // parameter using in initial seeds selection

        num_sectors_each_zone = {16, 32, 54, 32};   // Setting of Concentric Zone Model(CZM)
        num_rings_each_zone   = {2, 4, 4, 4};       // Setting of Concentric Zone Model(CZM)

        max_flatness_storage  = 1000;   // The maximum number of flatness storage
        max_elevation_storage = 1000;   // The maximum number of elevation storage
        elevation_thr = {0, 0, 0, 0};   // threshold of elevation for each ring using in GLE. Those
                                        // values are updated adaptively.
        flatness_thr = {0, 0, 0, 0};    // threshold of flatness for each ring using in GLE. Those
                                        // values are updated adaptively.
    }
};

template<typename PointT> bool point_z_cmp(PointT a, PointT b)
{
    return a.z < b.z;
}

template<typename PointT> struct RevertCandidate
{
    int                     concentric_idx;
    int                     sector_idx;
    double                  ground_flatness;
    double                  line_variable;
    Eigen::Vector4f         pc_mean;
    pcl::PointCloud<PointT> regionwise_ground;

    RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var,
                    Eigen::Vector4f _pc_mean, pcl::PointCloud<PointT> _ground)
        : concentric_idx(_c_idx)
        , sector_idx(_s_idx)
        , ground_flatness(_flatness)
        , line_variable(_line_var)
        , pc_mean(_pc_mean)
        , regionwise_ground(_ground)
    {
    }
};

template<typename PointT> class PatchWorkpp
{

public:
    typedef std::vector<pcl::PointCloud<PointT>> Ring;
    typedef std::vector<Ring>                    Zone;

    PatchWorkpp(const GroundParams& params)
    {
        verbose_                        = params.verbose;
        sensor_height_                  = params.sensor_height;
        num_iter_                       = params.num_iter;
        num_lpr_                        = params.num_lpr;
        num_min_pts_                    = params.num_min_pts;
        th_seeds_v_                     = params.th_seeds_v;
        th_dist_v_                      = params.th_dist_v;
        max_range_                      = params.max_range;
        min_range_                      = params.min_range;
        uprightness_thr_                = params.uprightness_thr;
        adaptive_seed_selection_margin_ = params.adaptive_seed_selection_margin;
        RNR_ver_angle_thr_              = params.RNR_ver_angle_thr;
        RNR_intensity_thr_              = params.RNR_intensity_thr;
        max_flatness_storage_           = params.max_flatness_storage;
        max_elevation_storage_          = params.max_elevation_storage;
        enable_RNR_                     = params.enable_RNR;
        enable_RVPF_                    = params.enable_RVPF;
        enable_TGR_                     = params.enable_TGR;
        num_zones_                      = params.num_zones;
        num_sectors_each_zone_          = params.num_sectors_each_zone;
        num_rings_each_zone_            = params.num_rings_each_zone;
        elevation_thr_                  = params.elevation_thr;
        flatness_thr_                   = params.flatness_thr;

        if (num_zones_ != 4 || num_sectors_each_zone_.size() != num_rings_each_zone_.size())
        {
            throw invalid_argument(
                "Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
        }
        if (elevation_thr_.size() != flatness_thr_.size())
        {
            throw invalid_argument(
                "Some parameters are wrong! Check the elevation/flatness_thresholds");
        }

        cout << (boost::format("Num. sectors: %d, %d, %d, %d") % num_sectors_each_zone_[0] %
                 num_sectors_each_zone_[1] % num_sectors_each_zone_[2] % num_sectors_each_zone_[3])
                    .str()
             << endl;
        cout << (boost::format("Num. rings: %01d, %01d, %01d, %01d") % num_rings_each_zone_[0] %
                 num_rings_each_zone_[1] % num_rings_each_zone_[2] % num_rings_each_zone_[3])
                    .str()
             << endl;
        cout << (boost::format("elevation_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % elevation_thr_[0] %
                 elevation_thr_[1] % elevation_thr_[2] % elevation_thr_[3])
                    .str()
             << endl;
        cout << (boost::format("flatness_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % flatness_thr_[0] %
                 flatness_thr_[1] % flatness_thr_[2] % flatness_thr_[3])
                    .str()
             << endl;
        num_rings_of_interest_ = elevation_thr_.size();

        revert_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

        min_range_z2_ = (7 * min_range_ + max_range_) / 8.0;
        min_range_z3_ = (3 * min_range_ + max_range_) / 4.0;
        min_range_z4_ = (min_range_ + max_range_) / 2.0;

        min_ranges_   = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
        ring_sizes_   = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                         (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                         (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                         (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
        sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0),
                         2 * M_PI / num_sectors_each_zone_.at(1),
                         2 * M_PI / num_sectors_each_zone_.at(2),
                         2 * M_PI / num_sectors_each_zone_.at(3)};

        cout << "INITIALIZATION COMPLETE" << endl;

        for (int i = 0; i < num_zones_; i++)
        {
            Zone z;
            initialize_zone(z, num_sectors_each_zone_[i], num_rings_each_zone_[i]);
            ConcentricZoneModel_.push_back(z);
        }
    }

    void estimate_ground(pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT>& cloud_ground,
                         pcl::PointCloud<PointT>& cloud_nonground, double& time_taken);

private:
    std::recursive_mutex mutex_;

    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    int num_zones_;
    int num_rings_of_interest_;

    double sensor_height_;
    double th_seeds_;
    double th_dist_;
    double th_seeds_v_;
    double th_dist_v_;
    double max_range_;
    double min_range_;
    double uprightness_thr_;
    double adaptive_seed_selection_margin_;
    double min_range_z2_;   // 12.3625
    double min_range_z3_;   // 22.025
    double min_range_z4_;   // 41.35
    double RNR_ver_angle_thr_;
    double RNR_intensity_thr_;

    bool verbose_;
    bool enable_RNR_;
    bool enable_RVPF_;
    bool enable_TGR_;

    int                 max_flatness_storage_, max_elevation_storage_;
    std::vector<double> update_flatness_[4];
    std::vector<double> update_elevation_[4];

    float d_;

    VectorXf        normal_;
    MatrixXf        pnormal_;
    VectorXf        singular_values_;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;

    vector<int> num_sectors_each_zone_;
    vector<int> num_rings_each_zone_;

    vector<double> sector_sizes_;
    vector<double> ring_sizes_;
    vector<double> min_ranges_;
    vector<double> elevation_thr_;
    vector<double> flatness_thr_;

    queue<int> noise_idxs_;

    vector<Zone> ConcentricZoneModel_;

    pcl::PointCloud<PointT> revert_pc_, reject_pc_, noise_pc_, vertical_pc_;
    pcl::PointCloud<PointT> ground_pc_;

    pcl::PointCloud<pcl::PointXYZINormal> normals_;

    pcl::PointCloud<PointT> regionwise_ground_, regionwise_nonground_;

    void initialize_zone(Zone& z, int num_sectors, int num_rings);

    void flush_patches_in_zone(Zone& patches, int num_sectors, int num_rings);
    void flush_patches(std::vector<Zone>& czm);

    void pc2czm(const pcl::PointCloud<PointT>& src, std::vector<Zone>& czm,
                pcl::PointCloud<PointT>& cloud_nonground);

    void reflected_noise_removal(pcl::PointCloud<PointT>& cloud,
                                 pcl::PointCloud<PointT>& cloud_nonground);

    void temporal_ground_revert(pcl::PointCloud<PointT>&             cloud_ground,
                                pcl::PointCloud<PointT>&             cloud_nonground,
                                std::vector<double>                  ring_flatness,
                                std::vector<RevertCandidate<PointT>> candidates,
                                int                                  concentric_idx);

    void calc_mean_stdev(std::vector<double> vec, double& mean, double& stdev);

    void update_elevation_thr();
    void update_flatness_thr();

    double xy2theta(const double& x, const double& y);

    double xy2radius(const double& x, const double& y);

    void estimate_plane(const pcl::PointCloud<PointT>& ground);

    void extract_piecewiseground(const int zone_idx, const pcl::PointCloud<PointT>& src,
                                 pcl::PointCloud<PointT>& dst,
                                 pcl::PointCloud<PointT>& non_ground_dst);

    void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointT>& p_sorted,
                               pcl::PointCloud<PointT>& init_seeds);

    void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointT>& p_sorted,
                               pcl::PointCloud<PointT>& init_seeds, double th_seed);
};

template<typename PointT>
inline void PatchWorkpp<PointT>::initialize_zone(Zone& z, int num_sectors, int num_rings)
{
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring ring;
    for (int i = 0; i < num_sectors; i++)
    {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++)
    {
        z.emplace_back(ring);
    }
}

template<typename PointT>
inline void PatchWorkpp<PointT>::flush_patches_in_zone(Zone& patches, int num_sectors,
                                                       int num_rings)
{
    for (int i = 0; i < num_sectors; i++)
    {
        for (int j = 0; j < num_rings; j++)
        {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

template<typename PointT> inline void PatchWorkpp<PointT>::flush_patches(vector<Zone>& czm)
{
    for (int k = 0; k < num_zones_; k++)
    {
        for (int i = 0; i < num_rings_each_zone_[k]; i++)
        {
            for (int j = 0; j < num_sectors_each_zone_[k]; j++)
            {
                if (!czm[k][i][j].points.empty()) czm[k][i][j].points.clear();
            }
        }
    }

    if (verbose_) cout << "Flushed patches" << endl;
}

template<typename PointT>
inline void PatchWorkpp<PointT>::estimate_plane(const pcl::PointCloud<PointT>& ground)
{
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    if (normal_(2) < 0)
    {
        for (int i = 0; i < 3; i++) normal_(i) *= -1;
    }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

template<typename PointT>
inline void PatchWorkpp<PointT>::extract_initial_seeds(const int                      zone_idx,
                                                       const pcl::PointCloud<PointT>& p_sorted,
                                                       pcl::PointCloud<PointT>&       init_seeds,
                                                       double                         th_seed)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); i++)
        {
            if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;   // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seed)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template<typename PointT>
inline void PatchWorkpp<PointT>::extract_initial_seeds(const int                      zone_idx,
                                                       const pcl::PointCloud<PointT>& p_sorted,
                                                       pcl::PointCloud<PointT>&       init_seeds)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); i++)
        {
            if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;   // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template<typename PointT>
inline void PatchWorkpp<PointT>::reflected_noise_removal(pcl::PointCloud<PointT>& cloud_in,
                                                         pcl::PointCloud<PointT>& cloud_nonground)
{
    for (int i = 0; i < cloud_in.size(); i++)
    {
        double r = sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y);
        double z = cloud_in[i].z;
        double ver_angle_in_deg = atan2(z, r) * 180 / M_PI;

        if (ver_angle_in_deg < RNR_ver_angle_thr_ && z < -sensor_height_ - 0.8 &&
            cloud_in[i].intensity < RNR_intensity_thr_)
        {
            cloud_nonground.push_back(cloud_in[i]);
            noise_pc_.push_back(cloud_in[i]);
            noise_idxs_.push(i);
        }
    }

    if (verbose_) cout << "[ RNR ] Num of noises : " << noise_pc_.points.size() << endl;
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/

template<typename PointT>
inline void PatchWorkpp<PointT>::estimate_ground(pcl::PointCloud<PointT>  cloud_in,
                                                 pcl::PointCloud<PointT>& cloud_ground,
                                                 pcl::PointCloud<PointT>& cloud_nonground,
                                                 double&                  time_taken)
{

    unique_lock<recursive_mutex> lock(mutex_);

    static double start, t0, t1, t2, end;

    double pca_time_        = 0.0;
    double t_revert         = 0.0;
    double t_total_ground   = 0.0;
    double t_total_estimate = 0.0;

    start = ros::Time::now().toSec();

    cloud_ground.clear();
    cloud_nonground.clear();

    // 1. Reflected Noise Removal (RNR)
    if (enable_RNR_) reflected_noise_removal(cloud_in, cloud_nonground);

    t1 = ros::Time::now().toSec();

    // 2. Concentric Zone Model (CZM)
    flush_patches(ConcentricZoneModel_);
    pc2czm(cloud_in, ConcentricZoneModel_, cloud_nonground);

    t2 = ros::Time::now().toSec();

    int concentric_idx = 0;

    double t_sort = 0;

    std::vector<RevertCandidate<PointT>> candidates;
    std::vector<double>                  ringwise_flatness;

    for (int zone_idx = 0; zone_idx < num_zones_; ++zone_idx)
    {

        auto zone = ConcentricZoneModel_[zone_idx];

        for (int ring_idx = 0; ring_idx < num_rings_each_zone_[zone_idx]; ++ring_idx)
        {
            for (int sector_idx = 0; sector_idx < num_sectors_each_zone_[zone_idx]; ++sector_idx)
            {

                if (zone[ring_idx][sector_idx].points.size() < num_min_pts_)
                {
                    cloud_nonground += zone[ring_idx][sector_idx];
                    continue;
                }

                // --------- region-wise sorting (faster than global sorting method)
                // ---------------- //
                double t_sort_0 = ros::Time::now().toSec();

                sort(zone[ring_idx][sector_idx].points.begin(),
                     zone[ring_idx][sector_idx].points.end(),
                     point_z_cmp<PointT>);

                double t_sort_1 = ros::Time::now().toSec();
                t_sort += (t_sort_1 - t_sort_0);
                // ----------------------------------------------------------------------------------
                // //

                double t_tmp0 = ros::Time::now().toSec();
                extract_piecewiseground(zone_idx,
                                        zone[ring_idx][sector_idx],
                                        regionwise_ground_,
                                        regionwise_nonground_);

                double t_tmp1 = ros::Time::now().toSec();
                t_total_ground += t_tmp1 - t_tmp0;
                pca_time_ += t_tmp1 - t_tmp0;

                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = normal_(2);
                const double ground_elevation   = pc_mean_(2, 0);
                const double ground_flatness    = singular_values_.minCoeff();
                const double line_variable      = singular_values_(1) != 0
                                                      ? singular_values_(0) / singular_values_(1)
                                                      : std::numeric_limits<double>::max();

                double heading = 0.0;
                for (int i = 0; i < 3; i++) heading += pc_mean_(i, 0) * normal_(i);

                double t_tmp2 = ros::Time::now().toSec();

                /*
                    About 'is_heading_outside' condidition, heading should be smaller than 0
                   theoretically. ( Imagine the geometric relationship between the surface normal
                   vector on the ground plane and the vector connecting the sensor origin and the
                   mean point of the ground plane )

                    However, when the patch is far awaw from the sensor origin,
                    heading could be larger than 0 even if it's ground due to lack of amount of
                   ground plane points.

                    Therefore, we only check this value when concentric_idx < num_rings_of_interest
                   ( near condition )
                */
                bool is_upright         = ground_uprightness > uprightness_thr_;
                bool is_not_elevated    = ground_elevation < elevation_thr_[concentric_idx];
                bool is_flat            = ground_flatness < flatness_thr_[concentric_idx];
                bool is_near_zone       = concentric_idx < num_rings_of_interest_;
                bool is_heading_outside = heading < 0.0;

                /*
                    Store the elevation & flatness variables
                    for A-GLE (Adaptive Ground Likelihood Estimation)
                    and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
                */
                if (is_upright && is_not_elevated && is_near_zone)
                {
                    update_elevation_[concentric_idx].push_back(ground_elevation);
                    update_flatness_[concentric_idx].push_back(ground_flatness);

                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (!is_near_zone)
                {
                    cloud_ground += regionwise_ground_;
                }
                else if (!is_heading_outside)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (is_not_elevated || is_flat)
                {
                    cloud_ground += regionwise_ground_;
                }
                else
                {
                    RevertCandidate<PointT> candidate(concentric_idx,
                                                      sector_idx,
                                                      ground_flatness,
                                                      line_variable,
                                                      pc_mean_,
                                                      regionwise_ground_);
                    candidates.push_back(candidate);
                }
                // Every regionwise_nonground is considered nonground.
                cloud_nonground += regionwise_nonground_;

                double t_tmp3 = ros::Time::now().toSec();
                t_total_estimate += t_tmp3 - t_tmp2;
            }

            double t_bef_revert = ros::Time::now().toSec();

            if (!candidates.empty())
            {
                if (enable_TGR_)
                {
                    temporal_ground_revert(cloud_ground,
                                           cloud_nonground,
                                           ringwise_flatness,
                                           candidates,
                                           concentric_idx);
                }
                else
                {
                    for (size_t i = 0; i < candidates.size(); i++)
                    {
                        cloud_nonground += candidates[i].regionwise_ground;
                    }
                }

                candidates.clear();
                ringwise_flatness.clear();
            }

            double t_aft_revert = ros::Time::now().toSec();

            t_revert += t_aft_revert - t_bef_revert;

            concentric_idx++;
        }
    }

    double t_update = ros::Time::now().toSec();

    update_elevation_thr();
    update_flatness_thr();

    end        = ros::Time::now().toSec();
    time_taken = end - start;

    // cout << "Time taken : " << time_taken << endl;
    // cout << "Time taken to sort: " << t_sort << endl;
    // cout << "Time taken to pca : " << pca_time_ << endl;
    // cout << "Time taken to estimate: " << t_total_estimate << endl;
    // cout << "Time taken to Revert: " <<  t_revert << endl;
    // cout << "Time taken to update : " << end - t_update << endl;

    revert_pc_.clear();
    reject_pc_.clear();
    normals_.clear();
    noise_pc_.clear();
    vertical_pc_.clear();
}

template<typename PointT> inline void PatchWorkpp<PointT>::update_elevation_thr(void)
{
    for (int i = 0; i < num_rings_of_interest_; i++)
    {
        if (update_elevation_[i].empty()) continue;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
        if (i == 0)
        {
            elevation_thr_[i] = update_mean + 3 * update_stdev;
            sensor_height_    = -update_mean;
        }
        else
            elevation_thr_[i] = update_mean + 2 * update_stdev;

        // if (verbose_) cout << "elevation threshold [" << i << "]: " << elevation_thr_[i] << endl;

        int exceed_num = update_elevation_[i].size() - max_elevation_storage_;
        if (exceed_num > 0)
            update_elevation_[i].erase(update_elevation_[i].begin(),
                                       update_elevation_[i].begin() + exceed_num);
    }

    if (verbose_)
    {
        cout << "sensor height: " << sensor_height_ << endl;
        cout << (boost::format("elevation_thr_  :   %0.4f,  %0.4f,  %0.4f,  %0.4f") %
                 elevation_thr_[0] % elevation_thr_[1] % elevation_thr_[2] % elevation_thr_[3])
                    .str()
             << endl;
    }

    return;
}

template<typename PointT> inline void PatchWorkpp<PointT>::update_flatness_thr(void)
{
    for (int i = 0; i < num_rings_of_interest_; i++)
    {
        if (update_flatness_[i].empty()) break;
        if (update_flatness_[i].size() <= 1) break;

        double update_mean = 0.0, update_stdev = 0.0;
        calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
        flatness_thr_[i] = update_mean + update_stdev;

        // if (verbose_) { cout << "flatness threshold [" << i << "]: " << flatness_thr_[i] << endl;
        // }

        int exceed_num = update_flatness_[i].size() - max_flatness_storage_;
        if (exceed_num > 0)
            update_flatness_[i].erase(update_flatness_[i].begin(),
                                      update_flatness_[i].begin() + exceed_num);
    }

    if (verbose_)
    {
        cout << (boost::format("flatness_thr_   :   %0.4f,  %0.4f,  %0.4f,  %0.4f") %
                 flatness_thr_[0] % flatness_thr_[1] % flatness_thr_[2] % flatness_thr_[3])
                    .str()
             << endl;
    }

    return;
}

template<typename PointT>
inline void PatchWorkpp<PointT>::temporal_ground_revert(
    pcl::PointCloud<PointT>& cloud_ground, pcl::PointCloud<PointT>& cloud_nonground,
    std::vector<double> ring_flatness, std::vector<RevertCandidate<PointT>> candidates,
    int concentric_idx)
{
    if (verbose_)
        std::cout << "\033[1;34m"
                  << "=========== Temporal Ground Revert (TGR) ==========="
                  << "\033[0m" << endl;

    double mean_flatness = 0.0, stdev_flatness = 0.0;
    calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

    if (verbose_)
    {
        cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
             << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness
             << std::endl;
    }

    for (size_t i = 0; i < candidates.size(); i++)
    {
        RevertCandidate<PointT> candidate = candidates[i];

        // Debug
        if (verbose_)
        {
            cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                 << " / flatness: " << candidate.ground_flatness
                 << " / line_variable: " << candidate.line_variable
                 << " / ground_num : " << candidate.regionwise_ground.size() << "\033[0m" << endl;
        }

        double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
        double prob_flatness =
            1 / (1 + exp((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10)));

        if (candidate.regionwise_ground.size() > 1500 &&
            candidate.ground_flatness < th_dist_ * th_dist_)
            prob_flatness = 1.0;

        double prob_line = 1.0;
        if (candidate.line_variable >
            8.0)   //&& candidate.line_dir > M_PI/4)// candidate.ground_elevation >
                   // elevation_thr_[concentric_idx])
        {
            // if (verbose_) cout << "line_dir: " << candidate.line_dir << endl;
            prob_line = 0.0;
        }

        bool revert = prob_line * prob_flatness > 0.5;

        if (concentric_idx < num_rings_of_interest_)
        {
            if (revert)
            {
                if (verbose_)
                {
                    cout << "\033[1;32m"
                         << "REVERT TRUE"
                         << "\033[0m" << endl;
                }

                revert_pc_ += candidate.regionwise_ground;
                cloud_ground += candidate.regionwise_ground;
            }
            else
            {
                if (verbose_)
                {
                    cout << "\033[1;31m"
                         << "FINAL REJECT"
                         << "\033[0m" << endl;
                }
                reject_pc_ += candidate.regionwise_ground;
                cloud_nonground += candidate.regionwise_ground;
            }
        }
    }

    if (verbose_)
        std::cout << "\033[1;34m"
                  << "===================================================="
                  << "\033[0m" << endl;
}

// For adaptive
template<typename PointT>
inline void PatchWorkpp<PointT>::extract_piecewiseground(const int                      zone_idx,
                                                         const pcl::PointCloud<PointT>& src,
                                                         pcl::PointCloud<PointT>&       dst,
                                                         pcl::PointCloud<PointT>& non_ground_dst)
{

    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. Region-wise Vertical Plane Fitting (R-VPF)
    // : removes potential vertical plane under the ground plane
    pcl::PointCloud<PointT> src_wo_verticals;
    src_wo_verticals = src;

    if (enable_RVPF_)
    {
        for (int i = 0; i < num_iter_; i++)
        {
            extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, th_seeds_v_);
            estimate_plane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < uprightness_thr_)
            {
                pcl::PointCloud<PointT> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                Eigen::MatrixXf points(src_tmp.points.size(), 3);
                int             j = 0;
                for (auto& p : src_tmp.points)
                {
                    points.row(j++) << p.x, p.y, p.z;
                }
                // ground plane model
                Eigen::VectorXf result = points * normal_;

                for (int r = 0; r < result.rows(); r++)
                {
                    if (result[r] < th_dist_v_ - d_ && result[r] > -th_dist_v_ - d_)
                    {
                        non_ground_dst.points.push_back(src_tmp[r]);
                        vertical_pc_.points.push_back(src_tmp[r]);
                    }
                    else
                    {
                        src_wo_verticals.points.push_back(src_tmp[r]);
                    }
                }
            }
            else
                break;
        }
    }

    extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
    estimate_plane(ground_pc_);

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    // pointcloud to matrix
    Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
    int             j = 0;
    for (auto& p : src_wo_verticals.points)
    {
        points.row(j++) << p.x, p.y, p.z;
    }

    for (int i = 0; i < num_iter_; i++)
    {

        ground_pc_.clear();

        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (i < num_iter_ - 1)
            {
                if (result[r] < th_dist_ - d_)
                {
                    ground_pc_.points.push_back(src_wo_verticals[r]);
                }
            }
            else
            {   // Final stage
                if (result[r] < th_dist_ - d_)
                {
                    dst.points.push_back(src_wo_verticals[r]);
                }
                else
                {
                    non_ground_dst.points.push_back(src_wo_verticals[r]);
                }
            }
        }

        if (i < num_iter_ - 1)
            estimate_plane(ground_pc_);
        else
            estimate_plane(dst);
    }
}

template<typename PointT>
inline geometry_msgs::PolygonStamped PatchWorkpp<PointT>::set_polygons(int zone_idx, int r_idx,
                                                                       int theta_idx, int num_split)
{
    geometry_msgs::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::Point32 point;

    // RL
    double zone_min_range = min_ranges_[zone_idx];
    double r_len          = r_idx * ring_sizes_[zone_idx] + zone_min_range;
    double angle          = theta_idx * sector_sizes_[zone_idx];

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);
    // RU
    r_len   = r_len + ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx)
    {
        angle   = angle + sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    r_len   = r_len - ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx)
    {
        angle   = angle - sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}

template<typename PointT>
inline void PatchWorkpp<PointT>::calc_mean_stdev(std::vector<double> vec, double& mean,
                                                 double& stdev)
{
    if (vec.size() <= 1) return;

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i = 0; i < vec.size(); i++)
    {
        stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
    }
    stdev /= vec.size() - 1;
    stdev = sqrt(stdev);
}

template<typename PointT>
inline double PatchWorkpp<PointT>::xy2theta(const double& x, const double& y)
{   // 0 ~ 2 * PI
    // if (y >= 0) {
    //     return atan2(y, x); // 1, 2 quadrant
    // } else {
    //     return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    // }

    double angle = atan2(y, x);
    return angle > 0 ? angle : 2 * M_PI + angle;
}

template<typename PointT>
inline double PatchWorkpp<PointT>::xy2radius(const double& x, const double& y)
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

template<typename PointT>
inline void PatchWorkpp<PointT>::pc2czm(const pcl::PointCloud<PointT>& src, std::vector<Zone>& czm,
                                        pcl::PointCloud<PointT>& cloud_nonground)
{

    for (int i = 0; i < src.size(); i++)
    {
        if ((!noise_idxs_.empty()) && (i == noise_idxs_.front()))
        {
            noise_idxs_.pop();
            continue;
        }

        PointT pt = src.points[i];

        double r = xy2radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_))
        {
            double theta = xy2theta(pt.x, pt.y);

            int zone_idx = 0;
            if (r < min_ranges_[1])
                zone_idx = 0;
            else if (r < min_ranges_[2])
                zone_idx = 1;
            else if (r < min_ranges_[3])
                zone_idx = 2;
            else
                zone_idx = 3;

            int ring_idx =
                min(static_cast<int>(((r - min_ranges_[zone_idx]) / ring_sizes_[zone_idx])),
                    num_rings_each_zone_[zone_idx] - 1);
            int sector_idx = min(static_cast<int>((theta / sector_sizes_[zone_idx])),
                                 num_sectors_each_zone_[zone_idx] - 1);

            czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
        }
        else
        {
            cloud_nonground.push_back(pt);
        }
    }

    if (verbose_) cout << "[ CZM ] Divides pointcloud into the concentric zone model" << endl;
}

#endif
