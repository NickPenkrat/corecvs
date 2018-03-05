#ifndef OPENCVMOVINGOBJECTFLOW_H
#define OPENCVMOVINGOBJECTFLOW_H

#include "core/stereointerface/processor6D.h"
#include "core/stats/calculationStats.h"

#include "math.h"
#include <numeric>
#include "time.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

class OpenCVMovingObjectFlow : public corecvs::Processor6D
{
public:
    OpenCVMovingObjectFlow();
    corecvs::Statistics *stats = NULL;


    corecvs::G12Buffer *first  = NULL;
    corecvs::G12Buffer *second = NULL;

    corecvs::FlowBuffer *result = NULL;


    // Processor6D interface
public:
    virtual int beginFrame() override;
    virtual int clean(int mask) override;
    virtual int setFrameG12(FrameNames frameType, corecvs::G12Buffer *frame) override;
    virtual int setDisparityBufferS16(FrameNames frameType, corecvs::FlowBuffer *frame) override;

    virtual int setStats(corecvs::Statistics *stats) override;

    virtual int endFrame() override;

    virtual std::map<std::string, corecvs::DynamicObject> getParameters() override;
    virtual bool setParameters(std::string name, const corecvs::DynamicObject &param) override;

    virtual int setParameteri(int parameterName, int parameterValue) override;
    virtual int requestResultsi(int parameterValue) override;

    virtual corecvs::FlowBuffer *getFlow() override;

    virtual corecvs::FlowBuffer *getStereo() override;
    virtual int getError(std::string *errorString) override;

};

class OpenCVMovingObjectFlowImplFactory : public corecvs::Processor6DFactory {
public:
    virtual corecvs::Processor6D *getProcessor() override
    {
       OpenCVMovingObjectFlow *processor = new OpenCVMovingObjectFlow;
       return processor;
    }

    virtual std::string getName() override
    {
        return "MeshFlow";
    }

};

using namespace std;
using namespace cv;

#define MAX_CHANNEL_SIZE 50 //Maximum size of channel median filter
typedef Vec<float, MAX_CHANNEL_SIZE> Vec512f;
enum _MOTION_PROPAGATION { NORMAL, REGIONWISE };



class MeshFlow {

    //Function
  public:

    /*
        mesh_row_ & mesh_col_					  : normally set as 16x16 -> vertex size is 17x17 for 720 x 480 input
        mesh_width_ & mesh_height_				  : meshgrid width & height in pixel distance
        mesh_ransac_row_ & mesh_ransac_col_		  : normally set as 4x4
        mesh_vertex_x_ & mesh_vertex_y_			  : horizontal & vertical vertex position in image coordinate
        mesh_flow_global_x_ & mesh_flow_global_y_ : Global mesh-flow at each vertex, global flow = global flow + local flow
        mesh_flow_local_x_ & mesh_flow_local_y_   : Local mesh-flow
        num_max_feature_						  : maximum number of feature extracted on each mesh-grid
    */

    MeshFlow();

    ~MeshFlow();

    void init(const int image_height,
              const int image_width,
              const int mesh_row,
              const int mesh_col,
              const int mesh_ransac_row,
              const int mesh_ransac_col,
              const int median_filter_size_y,
              const int median_filter_size_x,
              const int num_max_feature,
              const float th_feature );

    void ComputeMeshFlow (const Mat image_prev,
                           const Mat image_cur);

    //void ComputeResidualFlow ( Mat& residual_flow_y,
    //                           Mat& residual_flow_x );

    /*  RESULT
        @Name : GetFeatureMatchingResult
        @Date : 2017.01.10
        @Do   : Feature matching result (1 - static background, 2 - moving objects, 3 - outliers)
    */
    Mat GetFeatureMatchingResult ();
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
                                 vector<Point2f> feat_ref,
                                 vector<Point2f> feat_bwd,
                                 vector<float> feat_error,
                                 vector<unsigned char>& feat_status_ref,
                                 vector<unsigned char> feat_status_bwd );
    /*
        @Name : GetInterpolatedPixel
        @Date : 2017.01.11
        @Do   : Bilinear interpolation for given pixel position
    */
    Vec3b GetInterpolatedPixel ( const float r,
                                 const float c,
                                 Mat data );

    unsigned char GetGrayValue ( Vec3b pix );

    /*
        @Name : MeshGridRANSACFitting
        @Date : 2017.01.15
        @Do   : Mesh-grid RANSAC outlier rejection(2 steps) and return feat_flag & mesh-grid homogrphy
    */
    void MeshGridRANSACFitting ( const vector<Point2f> feat,
                                 const vector<Point2f> feat_track,
                                 vector<unsigned char>& feat_status,
                                 vector<float> feat_error,
                                 vector<Mat>& mesh_homo );
    /*
        @Name : GlobalOutlierRejection
        @Date : 2017.01.15
        @Do   : Outlier rejection using global homography constructed only using
                static background feature mathcing from MeshGridRANSACFitting
    */
    void GlobalOutlierRejection ( const vector<Point2f> feat,
                                  const vector<Point2f> feat_track,
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
    void PerspectiveTransform ( vector<Point2f> feat1,
                                vector<Point2f>& feat2,
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
    void SuppressingMotionNoise ( const vector<Point2f> refined_feat_vec,
                                  const vector<Point2f> local_feat_flow );
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
    vector<Point2f> feat_cur_;
    vector<Point2f> feat_prev_;
    vector<unsigned char> feat_status_;
    //residual feature matching for moving obejct detection
    vector<Point2f> feat_candidates_;
    vector<Point2f> feat_candidates_track_;

  private:
    int image_width_;
    int image_height_;

    int mesh_row_;
    int mesh_col_;
    float mesh_width_;
    float mesh_height_;
    int mesh_ransac_row_;
    int mesh_ransac_col_;
    float mesh_ransac_width_;
    float mesh_ransac_height_;
    int median_filter_size_x_;
    int median_filter_size_y_;
    int num_max_feature_;
    float threshold_features_;

    //mesh-RANSAC homography, same size with mesh_ransac_row_ x mesh_ransac_col_
    vector<Mat> mesh_homography_;

    //local flow on each vertex of mesh-grid
    Mat mesh_flow_local_x_;
    Mat mesh_flow_local_y_;
};



#endif // OPENCVMOVINGOBJECTFLOW_H
