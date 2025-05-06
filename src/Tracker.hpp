#pragma once
// -----------------------------------------------------------------------------
//  tracker.hpp  –  Push-2 visual-tracker (thread-friendly, clean shutdown)
// -----------------------------------------------------------------------------
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>  // Linux / other Unix systems
#endif

#include <mutex>
#include <atomic>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <array>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/opengl.hpp>
#include <librealsense2/rs.hpp>

#include <raylib.h>          // only for Vector3
#include <eigen3/Eigen/Dense>   // always works


// -----------------------------------------------------------------------------
//  Helper structs
// -----------------------------------------------------------------------------
struct DraggableCorner {
    cv::Point2f position;
    bool        dragging = false;
    int         label    = -1;
};

struct AnchorPoint {
    cv::Point2f  original;
    cv::Mat      descriptor;
};

struct FrameBundle {
    cv::Mat                                   frame;         // RGBA 1280×720
    std::array<Vector3,4>                     corners;       // world-space
    std::optional<std::array<cv::Point2f,4>>  imageCorners;  // 2-D projection
    cv::Mat                                   rvec;          // CV camera-coords
    cv::Mat                                   tvec;
    cv::Mat                                   handRGBA;
    uint64_t                                  timestamp = 0; // cv::getTickCount()
};

struct DebugCircle {
    cv::Point2f center;
    cv::Scalar color;
    int radius;
    int thickness;
};

struct DebugLine {
    cv::Point2f start;
    cv::Point2f end;
    cv::Scalar color;
    int thickness;
};


// -----------------------------------------------------------------------------
//  Tracker – owns the RealSense pipeline & feature tracker
// -----------------------------------------------------------------------------
class Tracker
{
public:
    Tracker();
    ~Tracker();                       // guarantees pipeline is stopped

    // long-running loop – call from std::thread
    void run();

    // graceful shutdown: sets running=false and exits run() quickly
    void stop() { running = false; }

    // data access
    std::optional<std::array<Vector3,4>>        getPush2WorldCorners();
    std::optional<std::array<cv::Point2f,4>>    getPush2WarpedCorners();
    const cv::Mat&                              getCameraMatrix() const {
        return cameraMatrix;
    }

    // most recent RGB/depth/pose for the render thread
    std::mutex   frameSwapMutex;
    FrameBundle  nextFrame;

    // lens parameters (k1…k3,p1,p2)
    cv::Mat      distCoeffs;

private:
    // ---- RealSense ----
    rs2::pipeline   pipe;
    bool            pipeStarted = false;

    // ---- main loop control ----
    std::atomic<bool> running{true};

    // ---- calibration ----
    cv::Mat cameraMatrix;            // 3×3 intrinsics

    // ---- feature-tracking state ----
    std::mutex                     poseMutex;
    std::array<Vector3,4>          currentWorldCorners{{}};
    bool                           cornersValid = false;

    std::vector<DraggableCorner>   corners;
    std::vector<cv::Point2f>       originalFiducials;
    std::vector<AnchorPoint>       anchorPoints;

    cv::Mat handMask;            // 8-bit mask (same size as colour frame)
    bool    planeInit = false;   // did we already estimate the controller plane?
    Eigen::Vector4f planeEq;     // ax+by+cz+d = 0 in CAMERA space

    cv::Mat                        prevGray;
    bool                           tracking      = false;
    int                            draggingCorner= -1;

    cv::Ptr<cv::Feature2D>         detector;
    cv::Ptr<cv::Feature2D>         descriptor;

    std::vector<DebugCircle> circlesToDraw;
    std::vector<DebugLine> linesToDraw;

    cv::ogl::Texture2D gpuFrameTex;
    GLuint sharedTextureID = 0; // for Raylib to draw

    // ---- helpers ----
    void startTracking(const cv::Mat& gray);
    void updateTracking(const cv::Mat& gray,
                        cv::Mat& frameRGBA,
                        cv::Mat& depth16);
    void redetectFeatures(const cv::Mat& gray);
    void drawCorners(cv::Mat& frameBGR);
    float sampleDepth(const cv::Mat& depth,
                      const cv::Point2f& pt);
    int  findNearestCorner(int x,int y);

    static void mouseCallback(int event,int x,int y,
                              int flags,void* userdata);
};
