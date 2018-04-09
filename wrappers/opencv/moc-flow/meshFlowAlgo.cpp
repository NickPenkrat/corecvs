#include "meshFlowAlgo.h"

/**/

using namespace corecvs;

MeshFlow::MeshFlow() {
}

void MeshFlow::initInternal()
{
    //mesh-flow
    mesh_width_ =  ((float)image_width_ / (float)mGridColumns);
    mesh_height_ =  ((float)image_height_ / (float)mGridRows);


    mesh_ransac_height_ = ((float) image_height_ / (float)mRansacGridRows);
    mesh_ransac_width_ =  ((float) image_width_ / (float)mRansacGridColumns);
    mesh_vertex_x_ = Mat::zeros(mGridRows + 1, mGridColumns + 1, DataType<float>::type);
    mesh_vertex_y_ = Mat::zeros(mGridRows + 1, mGridColumns + 1, DataType<float>::type);

    ComputeMeshVertices(mesh_vertex_y_, mesh_vertex_x_,
                          mGridRows, mGridColumns,
                          mesh_height_, mesh_width_);
}

void MeshFlow::init( const int image_height,
                     const int image_width,
                     const int mesh_row,
                     const int mesh_col,
                     const int mesh_ransac_row,
                     const int mesh_ransac_col,
                     const int median_filter_size_y,
                     const int median_filter_size_x,
                     const int num_max_feature,
                     const float th_feature)
{
    image_height_ = image_height;
    image_width_  = image_width;

    *static_cast<MeshFlowDrawParameters *>(this) = MeshFlowDrawParameters(mesh_row, mesh_col,
                           mesh_ransac_row, mesh_ransac_col,
                           median_filter_size_y, median_filter_size_x,
                           num_max_feature,
                           th_feature);
    initInternal();
}

void MeshFlow::init( const int image_height,
                     const int image_width,
                     const MeshFlowDrawParameters &params)
{
    image_height_ = image_height;
    image_width_ = image_width;
    *static_cast<MeshFlowDrawParameters *>(this) = params;
    initInternal();
}


MeshFlow::~MeshFlow() {
}



void MeshFlow::ResetInternalMemory() {
    mesh_flow_global_x_ = Mat::zeros (mGridRows + 1, mGridColumns + 1, DataType<float>::type);
    mesh_flow_global_y_ = Mat::zeros (mGridRows + 1, mGridColumns + 1, DataType<float>::type);
    mesh_flow_local_x_  = Mat::zeros (mGridRows + 1, mGridColumns + 1, DataType<float>::type);
    mesh_flow_local_y_  = Mat::zeros (mGridRows + 1, mGridColumns + 1, DataType<float>::type);
    feat_prev_.clear();
    feat_cur_.clear();
    feat_status_.clear();
    mesh_homography_.clear();
    feat_candidates_.clear();
    feat_candidates_track_.clear();

    for(int i = 0; i < mRansacGridColumns * mRansacGridRows; ++i) {
        mesh_homography_.push_back(Mat::eye(3, 3, DataType<double>::type));
    }
}

void MeshFlow::computeMeshFlow(const Mat image_prev,
                                 const Mat image_cur) {
    Statistics::startInterval(stats);
    //clock_t s, e;
    //double t_feat_extraction, t_feat_tracking, t_motion_propagation;
    vector<Point2f> features_vec;
    for(size_t p = 0; p < feat_cur_.size(); ++p)
        if(feat_status_[p] == 0)  //Moving objects
            features_vec.push_back(feat_cur_[p]);

    Statistics::resetInterval(stats, "Reset flow status");
    ResetInternalMemory();

    Statistics::resetInterval(stats, "Reset internal memory");
    Mat gray_prev, gray_cur;
    image_prev.copyTo(image_prev_);
    image_cur.copyTo(image_cur_);
    cvtColor(image_prev_, gray_prev, CV_BGR2GRAY);
    cvtColor(image_cur_, gray_cur, CV_BGR2GRAY);

    //1. Fast feature detection
    Statistics::resetInterval(stats, "Convert to grayscale");


    for(int i = 0; i < mGridRows; ++i) {
        for(int j = 0; j < mGridColumns; j++) {
            int tly = (int) mesh_vertex_y_.at<float>( i, j);
            int tlx = (int) mesh_vertex_x_.at<float>( i, j);
            int rby = (int) mesh_vertex_y_.at<float>( i + 1, j);
            int rbx = (int) mesh_vertex_x_.at<float>( i, j + 1);
            int w = rbx - tlx - 1;
            int h = rby - tly - 1;
            Mat image_vertex = gray_prev(Rect(tlx, tly, w, h)); //mesh-grid image patch
            //Detecting features
            vector<KeyPoint> kp;
            cv::FAST (image_vertex, kp,(int) mFeatureTreshold, true);

            for(int p = 0; p < (int)kp.size(); ++p) {
                features_vec.push_back(Point2f( tlx + kp[p].pt.x, tly + kp[p].pt.y));
            }
        }
    }

    Statistics::resetInterval(stats, "FAST features");

    //2. KLT tracking
    vector<Point2f> features_vec_tracked, features_vec_tracked_bwd;
    vector<unsigned char> features_status, features_status_bwd;
    vector<float> features_err, features_err_bwd;
    int window_size = 20;
    cv::calcOpticalFlowPyrLK(gray_prev,
                               gray_cur,
                               features_vec,
                               features_vec_tracked,
                               features_status,
                               features_err,
                               Size(window_size, window_size),
                               5,
                               cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3),
                               0);

    Statistics::resetInterval(stats, "KLT forward");

    //2.1 - Bi-directional tracking
    cv::calcOpticalFlowPyrLK(gray_cur,
                               gray_prev,
                               features_vec_tracked,
                               features_vec_tracked_bwd,
                               features_status_bwd,
                               features_err_bwd,
                               Size(window_size, window_size),
                               5,
                               cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3),
                               0);
    Statistics::resetInterval(stats, "KLT backward");

    //2.2- Bi-directional error, intensity difference, matching error
    RefineFeatureTracking(image_prev_,
                            image_cur_,
                            features_vec,
                            features_vec_tracked_bwd,
                            features_err,
                            features_status,
                            features_status_bwd);
    Statistics::resetInterval(stats, "Bi-diectional matching");

//
// 	//2.3 - 2-steps RANSAC Outliers rejection
    vector<Mat> mesh_homography;
    MeshGridRANSACFitting(features_vec,
                            features_vec_tracked,
                            features_status,
                            features_err,
                            mesh_homography);

    Statistics::resetInterval(stats, "RANSAC");

    //3.1 - Collecting inlier for building mesh-flow
    vector<Point2f> t_refined_features_vec;
    vector<Point2f> t_refined_features_vec_track;

    for(int i = 0; i < (int)features_status.size(); ++i) {
        if(features_status.at(i) == 1) {
            t_refined_features_vec.push_back(features_vec.at(i));
            t_refined_features_vec_track.push_back(features_vec_tracked.at(i));
        }
    }

    //Either global or regionwise homography estimation for initial global motion
    //Normal - global homography, REGIONWISE - using mesh_homography
    //Note : Perspective Transform reflects background more than the case that global homography, but taking more time
    Mat global_homography;
    vector<Point2f> refined_features_vec_reprojected;

    if(t_refined_features_vec.size() >= 4) {
        global_homography = cv::findHomography(t_refined_features_vec,
                                                 t_refined_features_vec_track,
                                                 CV_RANSAC);
    }
    else {
        global_homography = Mat::eye(3, 3, CV_64FC1);
    }

    Statistics::resetInterval(stats, "Constructing homography");

    //GlobalOutlierRejection(features_vec, features_vec_tracked, features_status, global_homography);

    vector<Point2f> refined_features_vec;
    vector<Point2f> refined_features_vec_track;

    for(int i = 0; i <(int) features_status.size(); ++i) {
        if(features_status.at(i) == 1) {
            refined_features_vec.push_back(features_vec.at(i));
            refined_features_vec_track.push_back(features_vec_tracked.at(i));
            feat_candidates_.push_back(features_vec.at(i));
            feat_candidates_track_.push_back(features_vec_tracked.at(i));
        }

        if(features_status.at(i) == 2) {
            feat_candidates_.push_back(features_vec.at(i));
            feat_candidates_track_.push_back(features_vec_tracked.at(i));
        }
    }

    PerspectiveTransform(refined_features_vec,
                           refined_features_vec_reprojected,
                           global_homography,
                           mesh_homography,
                           NORMAL);
    vector<Point2f> local_features_flow(refined_features_vec_track.size());
    vector<Point2f> global_features_flow(refined_features_vec_track.size());

    for(int p = 0; p <(int) refined_features_vec.size(); ++p) {
        local_features_flow.at(p) = refined_features_vec_track.at(p) -
                                       refined_features_vec_reprojected.at(p);
        global_features_flow.at(p) = refined_features_vec_reprojected.at(p) -
                                        refined_features_vec.at(p);
    }

    InitializeGlobalMeshFlow(global_homography);
    SuppressingMotionNoise(refined_features_vec, local_features_flow);

    mesh_flow_global_x_ = mesh_flow_global_x_ + mesh_flow_local_x_;
    mesh_flow_global_y_ = mesh_flow_global_y_ + mesh_flow_local_y_;

    medianBlur(mesh_flow_global_x_, mesh_flow_global_x_, mMedianFilterSizeW);
    medianBlur(mesh_flow_global_y_, mesh_flow_global_y_, mMedianFilterSizeH);

    feat_prev_ = features_vec;
    feat_cur_ = features_vec_tracked;
    feat_status_ = features_status;

    Statistics::endInterval(stats, "Finishing");
}

void MeshFlow::ComputeMeshVertices(Mat& mesh_vertex_y,
                                     Mat& mesh_vertex_x,
                                     const int mesh_row,
                                     const int mesh_col,
                                     const float mesh_height,
                                     const float mesh_width) {

    int u = 0, v = 0;

    for(int i = 0; i <= mesh_row; ++i)
    {
        v = (int)floor((float) i * mesh_height);

        for(int j = 0; j <= mesh_col; j++) {
            u = (int)floor((float) j * mesh_width);

            if(u >= image_width_) {
                u = image_width_ - 1;
            }

            if(v >= image_height_) {
                v = image_height_ - 1;
            }

            mesh_vertex_x.at<float>(i, j) = (float) u;
            mesh_vertex_y.at<float>(i, j) = (float) v;
        }
    }
}

void MeshFlow::RefineFeatureTracking(Mat img_prev,
                                       Mat img_cur,
                                       vector<Point2f> feat_ref,
                                       vector<Point2f> feat_bwd,
                                       vector<float> feat_error,
                                       vector<unsigned char>& feat_status_ref,
                                       vector<unsigned char> feat_status_bwd) {
    int num_feat =(int) feat_ref.size();
    float sum_feat_error =(float)std::accumulate(feat_error.begin(), feat_error.end(), 0.0);
    float mean_feat_error = sum_feat_error /(float)feat_error.size();
    float sq_sum_feat_error =(float)std::inner_product(feat_error.begin(), feat_error.end(), feat_error.begin(), 0.0);
    float stdev_feat_error = std::sqrt(sq_sum_feat_error / feat_error.size() - mean_feat_error * mean_feat_error);
    //Allow 3 sigma
    float error_threshold =(float)3.0 * stdev_feat_error;

    for(int p = 0; p < num_feat; ++p) {
        float bidirectional_error =(float)cv::norm(feat_ref.at(p) - feat_bwd.at(p));
        Vec3b pix_ref, pix_bwd;
        pix_ref = img_prev.at<Vec3b>(( int) feat_ref.at(p).y,(int) feat_ref.at(p).x);

        if(feat_bwd.at(p).y >= 0 && feat_bwd.at(p).y < image_height_ - 1 && feat_bwd.at(p).x >= 0 && feat_bwd.at(p).x < image_width_ - 1 && feat_error.at(p) <= error_threshold) {
            pix_bwd = GetInterpolatedPixel(feat_bwd.at(p).y, feat_bwd.at(p).x, img_prev);
            float intensity_diff =(float)abs(GetGrayValue(pix_ref) - GetGrayValue(pix_bwd));

            if(feat_status_ref.at(p) == 0 || feat_status_bwd.at(p) == 0 || bidirectional_error > 0.1) {
                feat_status_ref.at(p) = false;
            }
        }
        else {
            feat_status_ref.at(p) = false;
        }
    }
}

#if 0
void MeshFlow::RefineFeatureTrackingGV(Mat img_prev_gray,
                                         Mat img_cur_gray,
                                       vector<Point2f> feat_ref,
                                       vector<Point2f> feat_bwd,
                                       vector<float> feat_error,
                                       vector<unsigned char>& feat_status_ref,
                                       vector<unsigned char> feat_status_bwd)
{
    size_t num_feat = feat_ref.size();

    float sum_feat_error  = (float) std::accumulate(feat_error.begin(), feat_error.end(), 0.0);
    float mean_feat_error = sum_feat_error / (float) feat_error.size();

    float sq_sum_feat_error = (float) std::inner_product(feat_error.begin(), feat_error.end(), feat_error.begin(), 0.0);
    float stdev_feat_error = std::sqrt( sq_sum_feat_error / feat_error.size() - mean_feat_error * mean_feat_error);
    //Allow 3 sigma
    float error_threshold = (float) 3.0 * stdev_feat_error;

    for(size_t p = 0; p < num_feat; p++)
    {
        float bidirectional_error = (float) cv::norm(feat_ref.at(p) - feat_bwd.at(p));
        uint8_t pix_ref, pix_bwd;
        pix_ref = img_prev.at<uint8_t>(( int) feat_ref.at(p).y,(int) feat_ref.at(p).x);

        if(feat_bwd.at(p).y >= 0 && feat_bwd.at(p).y < image_height_ - 1 && feat_bwd.at(p).x >= 0 && feat_bwd.at(p).x < image_width_ - 1 && feat_error.at(p) <= error_threshold) {
            pix_bwd = GetInterpolatedPixel(feat_bwd.at(p).y, feat_bwd.at(p).x, img_prev);
            float intensity_diff =(float)abs(GetGrayValue(pix_ref) - GetGrayValue(pix_bwd));

            if(feat_status_ref.at(p) == 0 || feat_status_bwd.at(p) == 0 || bidirectional_error > 0.1) {
                feat_status_ref.at(p) = false;
            }
        }
        else {
            feat_status_ref.at(p) = false;
        }
    }
}
#endif

Vec3b MeshFlow::GetInterpolatedPixel(const float r,
                                       const float c,
                                       Mat data) {
    int idx_r =(int) floor(r);
    int idx_c =(int) floor(c);
    float f11 = 0.0, f12 = 0.0, f21 = 0.0, f22 = 0.0;
    float x = 0.0, x1 = 0.0, x2 = 0.0, y = 0.0, y1 = 0.0, y2 = 0.0;
    Vec3b pix;
    Vec3b f11_ = data.at<Vec3b>(idx_r, idx_c);
    Vec3b f21_ = data.at<Vec3b>(idx_r, idx_c + 1);
    Vec3b f12_ = data.at<Vec3b>(idx_r + 1, idx_c);
    Vec3b f22_ = data.at<Vec3b>(idx_r + 1, idx_c + 1);

    for(int i = 0; i < data.channels(); ++i) {
        f11 =(float)f11_.val[i];
        f21 =(float)f21_.val[i];
        f12 =(float)f12_.val[i];
        f22 =(float)f22_.val[i];
        x1 =(float)idx_c;
        x2 =(float)( idx_c + 1);
        y1 =(float)idx_r;
        y2 =(float)( idx_r + 1);
        x =(float)c;
        y =(float)r;
        float R1 =(( x2 - x) /(x2 - x1)) * f11 +(( x - x1) /(x2 - x1)) * f21;
        float R2 =(( x2 - x) /(x2 - x1)) * f12 +(( x - x1) /(x2 - x1)) * f22;
        pix.val[i] =(int)(((y2 - y) /(y2 - y1)) * R1 +(( y - y1) /(y2 - y1)) * R2);
    }

    return pix;
}

unsigned char MeshFlow::GetGrayValue(Vec3b pix) {
    float pix_gray;
    float wegith_blue  = 0.1140f;
    float weight_green = 0.5870f;
    float weight_red   = 0.2989f;
    pix_gray = wegith_blue * pix.val[0] + weight_green * pix.val[1] + weight_red * pix.val[2];
    return(unsigned char) pix_gray;
}

void MeshFlow::MeshGridRANSACFitting(const vector<Point2f> feat,
                                       const vector<Point2f> feat_track,
                                       vector<unsigned char>& feat_status,
                                       vector<float> feat_error,
                                       vector<Mat>& mesh_homo) {
    for(int i = 0; i < mRansacGridRows * mRansacGridColumns; ++i) {
        vector<Point2f> t_feat;
        vector<Point2f> t_feat_track;
        vector<unsigned char> tfeat_status_;
        vector<unsigned char> t_feat_outlier_status;
        vector<int> t_feat_index;
        float t_mesh_size_r =(float)image_height_ / (float)mRansacGridRows;
        float t_mesh_size_c =(float)image_width_  / (float)mRansacGridColumns;

        for(int j = 0; j <(int) feat.size(); j++) {
            int r, c;
            ComputeMeshIndex(feat.at(j).y, feat.at(j).x, t_mesh_size_r, t_mesh_size_c,(float)mRansacGridRows,(float)mRansacGridColumns, r, c);

            if(r * mRansacGridColumns + c == i) {
                t_feat.push_back(feat.at(j));
                t_feat_track.push_back(feat_track.at(j));
                t_feat_index.push_back(j);
            }
        }

        if(t_feat.size() >= 4) {
            //Semi-global RANSAC
            tfeat_status_.reserve(t_feat.size());
            Mat H = findHomography(t_feat, t_feat_track, CV_RANSAC, 1.5, tfeat_status_);
            mesh_homo.push_back(H);
            vector<Point2f> t_feat_, t_feat_track_;
            vector<int> t_feat_index_;
            vector<unsigned char> tfeat_status__;

            for(int j = 0; j < (int) tfeat_status_.size(); j++) {
                if(tfeat_status_.at(j) && feat_status.at(t_feat_index.at(j)) != 0) {
                    feat_status.at(t_feat_index.at(j)) =(unsigned char) 1;
                }
                else {
                    t_feat_.push_back(t_feat.at(j));
                    t_feat_track_.push_back(t_feat_track.at(j));
                    t_feat_index_.push_back(j);
                }
            }

            //Semi-local RANSAC
            if(t_feat_.size() >= 4) {
                tfeat_status__.reserve(t_feat_.size());
                Mat H1 = findHomography(t_feat_, t_feat_track_, CV_RANSAC, 5.0, tfeat_status__);

                for(int j = 0; j < tfeat_status__.size(); j++) {
                    if(tfeat_status__.at(j) &&
                            feat_status.at(t_feat_index.at(t_feat_index_.at(j))) != 0) {
                        feat_status.at(t_feat_index.at(t_feat_index_.at(j))) =(unsigned char) 2;
                    }
                    else {
                        feat_status.at(t_feat_index.at(t_feat_index_.at(j))) =(unsigned char) 0;
                    }
                }
            }
            else {
                for(int j = 0; j < t_feat_.size(); j++) {
                    feat_status.at(t_feat_index.at(t_feat_index_.at(j))) =(unsigned char) 0;
                }
            }
        }
        else {
            for(int j = 0; j < t_feat.size(); j++) {
                feat_status.at(t_feat_index.at(j)) =(unsigned char) 0;
            }

            Mat H = Mat::eye(3, 3, DataType<double>::type);
            mesh_homo.push_back(H);
        }
    }
}

void MeshFlow::GlobalOutlierRejection(const vector<Point2f> feat,
                                        const vector<Point2f> feat_track,
                                        vector<unsigned char>& feat_status,
                                        Mat global_homography) {

    int num_feat =(int) feat.size();

    if(num_feat > 0) {
        vector<Point2f> feat_reproj;
        cv::perspectiveTransform(feat, feat_reproj, global_homography);

        for(int i = 0; i < num_feat; ++i) {
            float dist =(float)norm(feat_reproj[i] - feat[i]);

            if(dist > 5.0 && feat_status[i] == 1) {
                feat_status[i] = 2;
            }
        }
    }
}

void MeshFlow::ComputeMeshIndex(const float r,
                                  const float c,
                                  const float mesh_size_r,
                                  const float mesh_size_c,
                                  const float max_mesh_r,
                                  const float max_mesh_c,
                                  int& mesh_index_r,
                                  int& mesh_index_c) {

    if(( int) r < image_height_ &&(int) c < image_width_) {
        mesh_index_r =(int) floor(r / mesh_size_r);
        mesh_index_c =(int) floor(c / mesh_size_c);
    }
    else {
        mesh_index_r =(int) max_mesh_r;
        mesh_index_c =(int) max_mesh_c;
    }
}

void MeshFlow::PerspectiveTransform(vector<Point2f> feat1,
                                      vector<Point2f>& feat2,
                                      Mat H,//global homogrphy
                                      vector<Mat> Hs, //local homography if exist
                                      int type) {
    int num_feat =(int) feat1.size();
    feat2 = feat1;
    Mat E = Mat(3, 3, DataType<double>::type);

    if(type == REGIONWISE) {
        for(int i = 0; i < mRansacGridRows * mRansacGridColumns; ++i) {
            int r, c;
            float t_mesh_size_r =(float)image_height_ /(float)mRansacGridRows;
            float t_mesh_size_c =(float)image_width_ /(float)mRansacGridColumns;
            vector<Point2f> t_feat;
            vector<Point2f> t_feat_track;
            vector<int> t_feat_idx;

            for(int j = 0; j < num_feat; j++) {
                ComputeMeshIndex(feat1.at(j).y, feat1.at(j).x, t_mesh_size_r, t_mesh_size_c,(float)mRansacGridRows,(float)mRansacGridColumns, r, c);

                if(r * mRansacGridColumns + c == i) {
                    t_feat.push_back(feat1.at(j));
                    t_feat_idx.push_back(j);
                }
            }

            Mat tH;

            if(!CompareMatEqual(E, Hs[i])) {
                Hs[i].copyTo(tH);
            }
            else {
                H.copyTo(tH);
            }

            if(t_feat.size() > 0 && tH.cols == 3 && tH.rows == 3) {
                if(t_feat.size() > 6) {
                    cv::perspectiveTransform(t_feat, t_feat_track, tH);
                }
                else {
                    cv::perspectiveTransform(t_feat, t_feat_track, H);
                }

                for(size_t j = 0; j < t_feat.size(); j++) {
                    feat2[t_feat_idx[j]] = t_feat_track[j];
                }
            }
        }
    }
    else {
        if(feat1.size() > 0) {
            cv::perspectiveTransform(feat1, feat2, H);
        }
    }
}

bool MeshFlow::CompareMatEqual(const Mat mat1, const Mat mat2) {
    if(mat1.empty() && mat2.empty()) {
        return true;
    }

    if(mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims) {
        return false;
    }

    cv::Mat diff;
    cv::compare(mat1, mat2, diff, cv::CMP_NE);
    int nz = cv::countNonZero(diff);
    return nz == 0;
}

void MeshFlow::InitializeGlobalMeshFlow(const Mat H) {
    vector<Point2f> mesh_vertices;
    vector<Point2f> mesh_vertices_reprojected;

    for(int i = 0; i < mGridRows + 1; ++i) {
        for(int j = 0; j < mGridColumns + 1; j++) {
            Point2f vertex;
            vertex.x = mesh_vertex_x_.at<float>(i, j);
            vertex.y = mesh_vertex_y_.at<float>(i, j);
            mesh_vertices.push_back(vertex);
        }
    }

    cv::perspectiveTransform(mesh_vertices, mesh_vertices_reprojected, H);

    for(int i = 0; i < mGridRows + 1; ++i) {
        for(int j = 0; j < mGridColumns + 1; j++) {
            int vertex_idx = i *(mGridColumns + 1) + j;
            float vextex_x = mesh_vertices_reprojected.at(vertex_idx).x - mesh_vertices.at(vertex_idx).x;
            float vertex_y = mesh_vertices_reprojected.at(vertex_idx).y - mesh_vertices.at(vertex_idx).y;
            mesh_flow_global_x_.at<float>(i, j) = vextex_x;
            mesh_flow_global_y_.at<float>(i, j) = vertex_y;
        }
    }

    medianBlur(mesh_flow_global_x_, mesh_flow_global_x_, mMedianFilterSizeW);
    medianBlur(mesh_flow_global_y_, mesh_flow_global_y_, mMedianFilterSizeH);
}

void MeshFlow::SuppressingMotionNoise(const vector<Point2f> refined_feat_vec,
                                        const vector<Point2f> local_feat_flow) {
    int num_feat =(int) refined_feat_vec.size();
    Mat t_local_mesh_flow_x = Mat::zeros(mGridRows + 1, mGridColumns + 1, CV_32FC(MAX_CHANNEL_SIZE));
    Mat t_local_mesh_flow_y = Mat::zeros(mGridRows + 1, mGridColumns + 1, CV_32FC(MAX_CHANNEL_SIZE));
    Mat t_local_mesh_store_idx_x = Mat::zeros(mGridRows + 1, mGridColumns + 1, DataType<int>::type);
    Mat t_local_mesh_store_idx_y = Mat::zeros(mGridRows + 1, mGridColumns + 1, DataType<int>::type);
    float t_mesh_size_r =(float)image_height_ /(float)mGridRows;
    float t_mesh_size_c =(float)image_width_ /(float)mGridColumns;

    //1. 3x3 mesh motion propagation
    for(int p = 0; p < num_feat; ++p) {
        int idx_r1, idx_r2, idx_c1, idx_c2, idx_r, idx_c;
        Point2f pt = refined_feat_vec.at(p);
        ComputeMeshIndex(pt.y,
                           pt.x,
                           t_mesh_size_r,
                           t_mesh_size_c,
                          (float)mGridRows,
                          (float)mGridColumns,
                           idx_r,
                           idx_c);
        idx_r1 = idx_r - 1;
        idx_r2 = idx_r + 2;
        idx_c1 = idx_c - 1;
        idx_c2 = idx_c + 2;

        if(idx_r1 < 0) {
            idx_r1 = 0;
        }

        if(idx_r2 > mGridRows) {
            idx_r2 = mGridRows;
        }

        if(idx_c1 < 0) {
            idx_c1 = 0;
        }

        if(idx_c2 > mGridColumns) {
            idx_c2 = mGridColumns;
        }

        for(int i = idx_r1; i <= idx_r2; ++i) {
            for(int j = idx_c1; j <= idx_c2; j++) {
                if(!(i == idx_r1 && j == idx_c1) &&
                        !(i == idx_r1 && j == idx_c2) &&
                        !(i == idx_r2 && j == idx_c1) &&
                        !(i == idx_r2 && j == idx_c2)) {
                    //float center_col = 0.0, center_row = 0.0;
                    //center_col =(((float)( idx_c2 - idx_c1)) / 2.0 +(float)idx_c1);
                    //center_row =(((float)( idx_r2 - idx_r1)) / 2.0 +(float)idx_r1);
                    //float w_x = GetGaussianWeight(j, center_col, 2.0);
                    //float w_y = GetGaussianWeight(i, center_row, 2.0);
                    if(t_local_mesh_store_idx_x.at<int>(i, j) < MAX_CHANNEL_SIZE - 1) {
                        //if(local_feat_flow.at(p).x>0.0)
                        {
                            Vec512f tx = t_local_mesh_flow_x.at<Vec512f>(i, j);
                            tx.val[t_local_mesh_store_idx_x.at<int>(i, j)] = local_feat_flow.at(p).x;
                            t_local_mesh_store_idx_x.at<int>(i, j) = t_local_mesh_store_idx_x.at<int>(i, j) + 1;
                            t_local_mesh_flow_x.at<Vec512f>(i, j) = tx;
                        }
                    }

                    if(t_local_mesh_store_idx_y.at<int>(i, j) < MAX_CHANNEL_SIZE - 1) {
                        //if(local_feat_flow.at(p).y > 0.0)
                        {
                            Vec512f ty = t_local_mesh_flow_y.at<Vec512f>(i, j);
                            ty.val[t_local_mesh_store_idx_y.at<int>(i, j)] = local_feat_flow.at(p).y;
                            t_local_mesh_store_idx_y.at<int>(i, j) = t_local_mesh_store_idx_y.at<int>(i, j) + 1;
                            t_local_mesh_flow_y.at<Vec512f>(i, j) = ty;
                        }
                    }
                }
            }
        }
    }

    //2. median filtering of propagated motion volume
    for(int i = 0; i <= mGridRows; ++i) {
        for(int j = 0; j <= mGridColumns; j++) {
            int nx = t_local_mesh_store_idx_x.at<int>(i, j);
            int ny = t_local_mesh_store_idx_y.at<int>(i, j);
            vector<float> t_flow_x, t_flow_y;

            if(nx > 0) {
                Vec512f t_flow = t_local_mesh_flow_x.at<Vec512f>(i, j);

                for(int p = 0; p < nx; ++p) {
                    t_flow_x.push_back(t_flow.val[p]);
                }

                mesh_flow_local_x_.at<float>(i, j) = GetMedian(t_flow_x);
            }
            else {
                mesh_flow_local_x_.at<float>(i, j) = 0.0;
            }

            if(ny > 0) {
                Vec512f t_flow = t_local_mesh_flow_y.at<Vec512f>(i, j);

                for(int p = 0; p < ny; ++p) {
                    t_flow_y.push_back(t_flow.val[p]);
                }

                mesh_flow_local_y_.at<float>(i, j) = GetMedian(t_flow_y);
            }
            else {
                mesh_flow_local_y_.at<float>(i, j) = 0.0;
            }
        }
    }
}

float MeshFlow::GetMedian(vector<float> flow_vec) {
    float median = 0.0;
    int size =(int) flow_vec.size();
    sort(flow_vec.begin(), flow_vec.end());

    if(size > 0) {
        if(size % 2 == 0) {
            median =(flow_vec[size / 2 - 1] + flow_vec[size / 2]) /(float)2.0;
        }
        else {
            median = flow_vec[size / 2];
        }
    }

    return median;
}

float MeshFlow::GetMeshGridBilinearInterpolatedValue(const int r,
                                                       const int c,
                                                       Mat data,
                                                       Mat vertex_y,
                                                       Mat vertex_x) {
    float f11 = 0.0, f12 = 0.0, f21 = 0.0, f22 = 0.0;
    float x = 0.0, x1 = 0.0, x2 = 0.0, y = 0.0, y1 = 0.0, y2 = 0.0;
    int mesh_row = data.rows - 1;
    int mesh_col = data.cols - 1;
    float mesh_size_r =(float)image_height_ /(float)mesh_row;
    float mesh_size_c =(float)image_width_ /(float)mesh_col;
    int mesh_idx_r = 0, mesh_idx_c = 0;
    float result = 0.0;
    x =(float)c;
    y =(float)r;
    ComputeMeshIndex ((float)r,
                      (float)c,
                       mesh_size_r,
                       mesh_size_c,
                      (float)mesh_row,
                      (float)mesh_col,
                       mesh_idx_r,
                       mesh_idx_c);
    f11 = data.at<float>(mesh_idx_r, mesh_idx_c);
    f21 = data.at<float>(mesh_idx_r, mesh_idx_c + 1);
    f12 = data.at<float>(mesh_idx_r + 1, mesh_idx_c);
    f22 = data.at<float>(mesh_idx_r + 1, mesh_idx_c + 1);

    x1 = vertex_x.at<float>(mesh_idx_r, mesh_idx_c);
    x2 = vertex_x.at<float>(mesh_idx_r, mesh_idx_c + 1);
    y1 = vertex_y.at<float>(mesh_idx_r, mesh_idx_c);
    y2 = vertex_y.at<float>(mesh_idx_r + 1, mesh_idx_c);
    float R1 =(( x2 - x) /(x2 - x1)) * f11 +(( x - x1) /(x2 - x1)) * f21;
    float R2 =(( x2 - x) /(x2 - x1)) * f12 +(( x - x1) /(x2 - x1)) * f22;
    result =(( y2 - y) /(y2 - y1)) * R1 +(( y - y1) /(y2 - y1)) * R2;
    return result;
}

inline float MeshFlow::GemanMcLureEstimator(const float error,
                                              const float sigma) {
    return(error * error) /(sigma * sigma + error * error);
}

//Result
Mat MeshFlow::getFeatureMatchingResult () {
    //See all grid-feature together
    Mat img;
    image_cur_.copyTo(img);

    for(size_t p = 0; p < feat_prev_.size(); ++p) {
        /*if(feat_status[p] == 1) //Static background
            arrowedLine(img, feat[p], feat_tracked[p], Scalar(0, 0, 255), 1, 8, 0, 0.4);

        else*/ if (feat_status_[p] == 2)  //Moving objects
            arrowedLine(img, feat_prev_[p], feat_cur_[p], Scalar(0, 255, 0), 1, 8, 0, 0.4);
    }

    return img;
}

Mat MeshFlow::GetMeshFlowResult(Mat vertex_x,
                                  Mat vertex_y,
                                  Mat flow_x,
                                  Mat flow_y) {

    Mat img;
    image_cur_.copyTo(img);
    int mesh_row = vertex_x.rows;
    int mesh_col = vertex_x.cols;
    int vertex_size = mesh_row * mesh_col;

    for(int i = 0; i < mesh_row; ++i) {
        for(int j = 0; j < mesh_col; j++) {
            Point2f t_vertex = Point2f(vertex_x.at<float>(i, j), vertex_y.at<float>(i, j));
            //circle(img, t_vertex, 1, Scalar(255, 0, 0), 1, 8, 0);
            Point2f t_flow_end = t_vertex + Point2f(flow_x.at<float>(i, j), flow_y.at<float>(i, j));
            //line(img, t_vertex, t_flow_end, Scalar(0, 0, 255), 1, 8, 0);
            arrowedLine(img, t_vertex, t_flow_end, Scalar(0, 0, 255), 1, 8, 0, 0.2);
        }
    }

    return img;
}

Mat MeshFlow::GetMeshFlowResult(Mat img,
                                  Mat vertex_x,
                                  Mat vertex_y,
                                  Mat flow_x,
                                  Mat flow_y) {
    img = Mat::zeros(image_height_, image_width_, CV_8UC3);
    //image_prev_.copyTo(img);
    int mesh_row = vertex_x.rows;
    int mesh_col = vertex_x.cols;

    for(int i = 0; i < mesh_row; ++i) {
        for(int j = 0; j < mesh_col; j++) {
            if(flow_x.at<float>(i, j) > 0 && flow_y.at<float>(i, j) > 0) {
                Point2f pt;
                pt.x = flow_x.at<float>(i, j);
                pt.y = flow_y.at<float>(i, j);
                Point2f t_vertex = Point2f(vertex_x.at<float>(i, j), vertex_y.at<float>(i, j));
                //circle(img, t_vertex, 1, Scalar(255, 0, 0), 1, 8, 0);
                Point2f t_flow_end = t_vertex +  10 * Point2f(flow_x.at<float>(i, j), flow_y.at<float>(i, j));
                //line(img, t_vertex, t_flow_end, Scalar(0, 0, 255), 1, 8, 0);
                arrowedLine(img, t_vertex, t_flow_end, Scalar(0, 0, 255), 1, 8, 0, 0.2);
            }
        }
    }

    return img;
}

Mat MeshFlow::CombineImages(Mat img1, Mat img2) {
    Mat combine = Mat::zeros(Size(img1.cols + img2.cols, img1.rows), CV_8UC3);
    Mat region_img1 = combine(Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(region_img1);
    Mat region_img2 = combine(Rect(img1.cols, 0, img2.cols, img2.rows));
    img2.copyTo(region_img2);
    return combine;
}

