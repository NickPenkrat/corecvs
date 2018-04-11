#ifndef MESHFLOWALGO_H
#define MESHFLOWALGO_H

#include "core/stereointerface/processor6D.h"
#include "core/stats/calculationStats.h"
#include "xml/generated/meshFlowDrawParameters.h"

#include <vector>
#include <math.h>
#include <numeric>
#include <time.h>

#ifdef WITH_OPENCV_3X
#include <opencv2/core.hpp>    // cv::Mat
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/video/tracking.hpp>
//#include <opencv2/calib3d.hpp>
#else
#include <opencv2/core/core.hpp>    // cv::Mat
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#endif

//using namespace std;
using std::vector;
//using namespace cv;
using cv::Mat;

#define MAX_CHANNEL_SIZE 50 //Maximum size of channel median filter
typedef cv::Vec<float, MAX_CHANNEL_SIZE> Vec512f;
enum _MOTION_PROPAGATION { NORMAL, REGIONWISE };



class MeshFlow : public MeshFlowDrawParameters
{
    //Function
  public:
    enum FlowClass {
        FLOW_UNKNOWN = 0,
        FLOW_KNOWN   = 1,
        FLOW_INLIER  = 2,
        FLOW_OUTLIER = 3
    };

    MeshFlow();

    ~MeshFlow();

    void init(const int image_height,
              const int image_width,
              const int mesh_row = 16,
              const int mesh_col = 16,
              const int mesh_ransac_row = 4,
              const int mesh_ransac_col = 4,
              const int median_filter_size_y = 5,
              const int median_filter_size_x = 5,
              const int num_max_feature = 50,
              const float th_feature = 10.0f);

    void init( const int image_height,
               const int image_width,
               const MeshFlowDrawParameters &params);

    void computeMeshFlow (const Mat image_prev,
                          const Mat image_cur);

    //void ComputeResidualFlow ( Mat& residual_flow_y,
    //                           Mat& residual_flow_x );

    /*  RESULT
        @Name : GetFeatureMatchingResult
        @Date : 2017.01.10
        @Do   : Feature matching result (1 - static background, 2 - moving objects, 3 - outliers)
    */
    Mat getFeatureMatchingResult ();
    /*
        @Name : GetMeshFlowResult
        @Date : 2017.01.10
        @Do   : display meshflow result on image_prev_
    */
    Mat GetMeshFlowResult ( Mat vertex_x,
                            Mat vertex_y,
                            Mat flow_x,
                            Mat flow_y );
    /*
        @Name : GetMeshFlowResult
        @Date : 2017.01.10
        @Do   : display meshflow result on blank image
    */
    Mat GetMeshFlowResult ( Mat img,
                            Mat vertex_x,
                            Mat vertex_y,
                            Mat flow_x,
                            Mat flow_y );


    Mat CombineImages ( Mat img1, Mat img2 );

  private:

    void ResetInternalMemory();
    /*
        @Name : ComputeMeshVertices
        @Date : 2017.01.20
        @Do   : compute vertices position for give size mesh_col & mesh_row
    */
    void ComputeMeshVertices ( Mat& mesh_vertex_y,
                               Mat& mesh_vertex_x,
                               const int mesh_row,
                               const int mesh_col,
                               const float mesh_height,
                               const float mesh_width );
    /*
        @Name : RefineFeatureTracking
        @Date : 2017.01.10
        @Do   : Reject feature matching by bi-directional error,
        intensity difference, matching error
    */
    void RefineFeatureTracking ( Mat img_prev,
                                 Mat img_cur,
                                 vector<cv::Point2f> feat_ref,
                                 vector<cv::Point2f> feat_bwd,
                                 vector<float> feat_error,
                                 vector<unsigned char>& feat_status_ref,
                                 vector<unsigned char> feat_status_bwd );
    /*
        @Name : GetInterpolatedPixel
        @Date : 2017.01.11
        @Do   : Bilinear interpolation for given pixel position
    */
    cv::Vec3b GetInterpolatedPixel ( const float r,
                                     const float c,
                                     Mat data );

    unsigned char GetGrayValue ( cv::Vec3b pix );

    /*
        @Name : MeshGridRANSACFitting
        @Date : 2017.01.15
        @Do   : Mesh-grid RANSAC outlier rejection(2 steps) and return feat_flag & mesh-grid homogrphy
    */
    void MeshGridRANSACFitting ( const vector<cv::Point2f> feat,
                                 const vector<cv::Point2f> feat_track,
                                 vector<unsigned char>& feat_status,
                                 vector<float> feat_error,
                                 vector<Mat>& mesh_homo );
    /*
        @Name : GlobalOutlierRejection
        @Date : 2017.01.15
        @Do   : Outlier rejection using global homography constructed only using
                static background feature mathcing from MeshGridRANSACFitting
    */
    void GlobalOutlierRejection ( const vector<cv::Point2f> feat,
                                  const vector<cv::Point2f> feat_track,
                                  vector<unsigned char>& feat_status,
                                  Mat global_homography );

    void ComputeMeshIndex ( const float r,
                            const float c,
                            const float mesh_size_r,
                            const float mesh_size_c,
                            const float max_mesh_r,
                            const float max_mesh_c,
                            int& mesh_index_r,
                            int& mesh_index_c );

    /*
        @Name : PerspectiveTransform
        @Date : 2017.01.19
        @Do   : Transform features either using mesh-grid homography or global homography
    */
    void PerspectiveTransform ( vector<cv::Point2f> feat1,
                                vector<cv::Point2f>& feat2,
                                Mat H,
                                vector<Mat> Hs,
                                int type );

    bool CompareMatEqual ( const Mat mat1, const Mat mat2 );

    /*
        @Name : InitializeGlobalMeshFlow, SuppressingMotionNoise, GetMedian
        @Date : 2017.01.21
        @Do   : Initialize meshflow using feat and feat_transformed using PerspectiveTransform.
                See more details on MeshFlow: Minimum Latency Online Video Stabilization(ECCV 2016)
    */
    void InitializeGlobalMeshFlow ( const Mat H );
    void SuppressingMotionNoise ( const vector<cv::Point2f> refined_feat_vec,
                                  const vector<cv::Point2f> local_feat_flow );
    float GetMedian ( vector<float> flow_vec );

    //Moving obejct detection
    float GetMeshGridBilinearInterpolatedValue ( int r, int c, Mat data, Mat vertex_x, Mat vertex_y );
    inline float GemanMcLureEstimator ( const float error, const float sigma );

    //Variables
  public:
    //current & previous image
    Mat image_cur_;
    Mat image_prev_;

    //global flow on each vertex of mesh-grid
    Mat mesh_flow_global_x_;
    Mat mesh_flow_global_y_;

    //pixel position on each vertex of mesh-grid
    Mat mesh_vertex_x_;
    Mat mesh_vertex_y_;

    /*
        feature matching information
        for _feat_status;
        0 - wrong matching
        1 - It is highly likely to be static background, meshflow is constructed only using feat tagged 1
        2 - It is highly likely to be moving objects
    */
    vector<cv::Point2f> feat_cur_;
    vector<cv::Point2f> feat_prev_;
    vector<unsigned char> feat_status_;
    //residual feature matching for moving obejct detection
    vector<cv::Point2f> feat_candidates_;
    vector<cv::Point2f> feat_candidates_track_;

  private:
    int image_width_;
    int image_height_;

    float mesh_width_;
    float mesh_height_;

    float mesh_ransac_width_;
    float mesh_ransac_height_;

    //mesh-RANSAC homography, same size with mesh_ransac_row_ x mesh_ransac_col_
    vector<Mat> mesh_homography_;

    //local flow on each vertex of mesh-grid
    Mat mesh_flow_local_x_;
    Mat mesh_flow_local_y_;

    void initInternal();
public:
    corecvs::Statistics *stats = NULL;

};

#endif // MESHFLOWALGO_H
