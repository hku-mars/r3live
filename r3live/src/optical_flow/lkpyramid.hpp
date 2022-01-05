// This file is modified from lkpyramid.hpp of openCV
#pragma once
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#define CV_CPU_HAS_SUPPORT_SSE2 1
#define USING_OPENCV_TBB 1
#include "opencv2/core/hal/intrin.hpp"
#include "tools_logger.hpp"
#include "tools_timer.hpp"
#include <numeric>
#include <future>
#include "tools_thread_pool.hpp"

enum
{
    cv_OPTFLOW_USE_INITIAL_FLOW = 4,
    cv_OPTFLOW_LK_GET_MIN_EIGENVALS = 8,
    cv_OPTFLOW_FARNEBACK_GAUSSIAN = 256
};

typedef short deriv_type;

inline int opencv_buildOpticalFlowPyramid(cv::InputArray img, cv::OutputArrayOfArrays pyramid,
                                   cv::Size winSize, int maxLevel, bool withDerivatives = true,
                                   int pyrBorder = cv::BORDER_REFLECT_101,
                                   int derivBorder = cv::BORDER_CONSTANT,
                                   bool tryReuseInputImage = true);

inline void calc_sharr_deriv(const cv::Mat &src, cv::Mat &dst);

void calculate_optical_flow(cv::InputArray prevImg, cv::InputArray nextImg,
                               cv::InputArray prevPts, cv::InputOutputArray nextPts,
                               cv::OutputArray status, cv::OutputArray err,
                               cv::Size winSize = cv::Size(21, 21), int maxLevel = 3,
                               cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                               int flags = 0, double minEigThreshold = 1e-4);

inline int calculate_LK_optical_flow(const cv::Range &range, const cv::Mat &_prevImg, const cv::Mat &_prevDeriv, const cv::Mat &_nextImg,
                            const cv::Point2f *_prevPts, cv::Point2f *_nextPts,
                            uchar *_status, float *_err,
                            cv::Size _winSize, cv::TermCriteria _criteria,
                            int _level, int _maxLevel, int _flags, float _minEigThreshold);

struct opencv_LKTrackerInvoker : cv::ParallelLoopBody
{
    opencv_LKTrackerInvoker(const cv::Mat *_prevImg, const cv::Mat *_prevDeriv, const cv::Mat *_nextImg,
                            const cv::Point2f *_prevPts, cv::Point2f *_nextPts,
                            uchar *_status, float *_err,
                            cv::Size _winSize, cv::TermCriteria _criteria,
                            int _level, int _maxLevel, int _flags, float _minEigThreshold);
    void operator()(const cv::Range &range) const;
    bool calculate( cv::Range range) const;
    const cv::Mat *prevImg;
    const cv::Mat *nextImg;
    const cv::Mat *prevDeriv;
    const cv::Point2f *prevPts;
    cv::Point2f *nextPts;
    uchar *status;
    float *err;
    cv::Size winSize;
    cv::TermCriteria criteria;
    int level;
    int maxLevel;
    int flags;
    float minEigThreshold;
};

class LK_optical_flow_kernel
{
public:
    cv::Size get_win_size() const { return m_lk_win_size; }
    void set_win_size(cv::Size winSize_) { m_lk_win_size = winSize_; }

    int get_max_level() const { return m_maxLevel; }
    void set_max_level(int maxLevel_) { m_maxLevel = maxLevel_; }

    cv::TermCriteria get_term_criteria() const { return m_terminate_criteria; }
    void set_term_criteria(cv::TermCriteria &crit_) { m_terminate_criteria = crit_; }

    int get_flags() const { return flags; }
    void set_flags(int flags_) { flags = flags_; }

    double get_min_eig_threshold() const { return minEigThreshold; }
    void set_min_eig_threshold(double minEigThreshold_) { minEigThreshold = minEigThreshold_; }

    LK_optical_flow_kernel(cv::Size winSize_ = cv::Size(21, 21),
                               int maxLevel_ = 3,
                               cv::TermCriteria criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                               int flags_ = 0,
                               double minEigThreshold_ = 1e-4
                               ) : m_lk_win_size(winSize_), m_maxLevel(maxLevel_), m_terminate_criteria(criteria_), flags(flags_), minEigThreshold(minEigThreshold_)
    {
        set_termination_criteria(m_terminate_criteria);
    }

    /**
    @brief Calculates a sparse optical flow.
    @param prevImg First input image.
    @param nextImg Second input image of the same cv::Size and the same type as prevImg.
    @param prevPts Vector of 2D points for which the flow needs to be found.
    @param nextPts Output vector of 2D points containing the calculated new positions of input features in the second image.
    @param status Output status vector. Each element of the vector is set to 1 if the
                  flow for the corresponding features has been found. Otherwise, it is set to 0.
    @param err Optional output vector that contains error response for each point (inverse confidence).
    **/
    void calc(cv::InputArray prevImg, cv::InputArray nextImg,
                    cv::InputArray prevPts, cv::InputOutputArray nextPts,
                    cv::OutputArray status,
                    cv::OutputArray err = cv::noArray());                        

    void allocate_img_deriv_memory( std::vector<cv::Mat> &img_pyr,
                                    std::vector<cv::Mat> &img_pyr_deriv_I,
                                    std::vector<cv::Mat> &img_pyr_deriv_I_buff);
    void calc_image_deriv_Sharr(std::vector<cv::Mat> &img_pyr,
                                std::vector<cv::Mat> &img_pyr_deriv_I,
                                std::vector<cv::Mat> &img_pyr_deriv_I_buff);
    void set_termination_criteria(cv::TermCriteria &crit);

public:
    std::vector<cv::Mat> m_prev_img_pyr, m_curr_img_pyr;
    std::vector<cv::Mat> m_prev_img_deriv_I, m_prev_img_deriv_I_buff;
    std::vector<cv::Mat> m_curr_img_deriv_I, m_curr_img_deriv_I_buff;
    cv::Size m_lk_win_size;
    int m_maxLevel;
    cv::TermCriteria m_terminate_criteria;
    int flags;
    double minEigThreshold;

    void swap_image_buffer();

    int track_image(const cv::Mat & curr_img, const std::vector<cv::Point2f> & last_tracked_pts, std::vector<cv::Point2f> & curr_tracked_pts, 
                    std::vector<uchar> & status, int opm_method = 3 ); // opm_method: [0] openCV parallel_body [1] openCV parallel for [2] Thread pool  
};