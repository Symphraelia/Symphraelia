#include "Tracker.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/opengl.hpp>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <thread>


Tracker::Tracker() {
    detector = cv::GFTTDetector::create(1000, 0.05, 1, 7, false, 0.1);
    descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();

    // Configure pipeline and options
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);

    // Temporarily start pipeline to apply frame queue option
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::device dev = profile.get_device();
    // In your Tracker constructor, after pipe.start(cfg):
    auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intr = color_profile.get_intrinsics();

    // Build your cv::Mat cameraMatrix from the actual values:
    cameraMatrix = (cv::Mat_<double>(3,3) << 
        intr.fx, 0,       /*(intr.width - 1) -*/ intr.ppx,
        0,       intr.fy, /*(intr.height -1) -*/ intr.ppy,
        0,       0,       1
    );
    // distortion coefficients
    distCoeffs = cv::Mat::zeros(1, 5, CV_64F); // initialize
    if (intr.model == RS2_DISTORTION_BROWN_CONRADY ||
        intr.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY) {
        distCoeffs.at<double>(0) = intr.coeffs[0]; // k1
        distCoeffs.at<double>(1) = intr.coeffs[1]; // k2
        distCoeffs.at<double>(2) = intr.coeffs[2]; // p1
        distCoeffs.at<double>(3) = intr.coeffs[3]; // p2
        distCoeffs.at<double>(4) = intr.coeffs[4]; // k3
    }

    // Warn if USB speed is low
    if (dev.query_sensors().size() > 0) {
        auto usb_desc = dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
        std::cout << "USB Type: " << usb_desc << "\n";
        if (std::string(usb_desc).find("2.") != std::string::npos)
            std::cerr << "[WARNING] USB 2.x connection detected. Expect frame drops.\n";
    }

    // Depth sensor config
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    /*if (depth_sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) {
        depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
    }//*/

    if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
        depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_DEFAULT);
        std::cout << "Depth preset set.\n";
    } else {
        std::cout << "Visual preset not supported.\n";
    }
}

Tracker::~Tracker()
{
    // tell the run-loop (if still active) to exit
    running = false;

    // stop the RS pipeline only if it was started
    if (pipeStarted)
        pipe.stop();
}


std::optional<std::array<Vector3,4>> Tracker::getPush2WorldCorners() {
    std::lock_guard<std::mutex> lock(poseMutex);
    if (cornersValid) {
        return currentWorldCorners;
    } else {
        return std::nullopt;
    }
}
std::optional<std::array<cv::Point2f, 4>> Tracker::getPush2WarpedCorners() {
    std::lock_guard<std::mutex> lock(poseMutex);
    if (!cornersValid) return std::nullopt;

    std::array<cv::Point2f, 4> warped;
    for (int i = 0; i < 4; ++i)
        warped[i] = corners[i].position;
    return warped;
}



void Tracker::startTracking(const cv::Mat& gray) {
    anchorPoints.clear();
    originalFiducials.clear();

    // 1) grab the 4 clicked corners
    for (auto& c : corners)
        originalFiducials.push_back(c.position);

    // 2) reorder into TL → TR → BR → BL exactly once
    {
        // temp copy
        std::vector<cv::Point2f> pts = originalFiducials;

        // sum/diff trick
        cv::Point2f TL, TR, BR, BL;
        float minSum  =  1e9f, maxSum  = -1e9f;
        float minDiff =  1e9f, maxDiff = -1e9f;
        for (auto &p : pts) {
            float s = p.x + p.y, d = p.y - p.x;
            if (s < minSum ) { minSum  = s;  TL = p; }
            if (s > maxSum ) { maxSum  = s;  BR = p; }
            if (d < minDiff) { minDiff = d;  TR = p; }
            if (d > maxDiff) { maxDiff = d;  BL = p; }
        }
        originalFiducials = { TL, TR, BR, BL };
    }

    // 3) then your existing mask + feature extraction…
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    std::vector<cv::Point> region;
    for (auto& c : corners) region.push_back(c.position);
    cv::fillConvexPoly(mask, region, 255);

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints, mask);
    cv::Mat descriptors;
    descriptor->compute(gray, keypoints, descriptors);
    for (int i = 0; i < (int)keypoints.size(); ++i)
        anchorPoints.push_back({keypoints[i].pt, descriptors.row(i).clone()});

    prevGray = gray.clone();
    std::cout << "Tracking initialized with " << anchorPoints.size() << " points.\n";
}


void Tracker::updateTracking(const cv::Mat& gray,
    cv::Mat& frame,
    cv::Mat& flipped_depth) {
        circlesToDraw.clear();
        linesToDraw.clear();
        if (corners.size() != 4) return;

// === 1–5. Your existing masking, blur, detect, describe, match ===
cv::Mat controllerMask = cv::Mat::zeros(gray.size(), CV_8UC1);
std::vector<cv::Point> region;
for (auto& c : corners) region.push_back(c.position);
cv::fillConvexPoly(controllerMask, region, 255);
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
cv::dilate(controllerMask, controllerMask, kernel);

cv::Mat blurredGray;
cv::GaussianBlur(gray, blurredGray, cv::Size(5,5), 5);
cv::Mat grayBlurred = blurredGray.clone();
gray.copyTo(grayBlurred, controllerMask);

std::vector<cv::KeyPoint> sceneKps;
detector->detect(grayBlurred, sceneKps);
cv::Mat sceneDesc;
descriptor->compute(grayBlurred, sceneKps, sceneDesc);

std::vector<cv::Point2f> srcPts, dstPts;
cv::BFMatcher matcher(cv::NORM_HAMMING);
for (auto& anchor : anchorPoints) {
std::vector<cv::DMatch> matches;
matcher.match(anchor.descriptor, sceneDesc, matches);
if (!matches.empty() && matches[0].distance < 50) {
srcPts.push_back(anchor.original);
dstPts.push_back(sceneKps[matches[0].trainIdx].pt);
circlesToDraw.push_back({
    sceneKps[matches[0].trainIdx].pt,
    cv::Scalar(0,255,255), // Yellow
    2,                     // Radius
    -1                     // Filled circle
});
}
}
if (srcPts.size() < 10) { redetectFeatures(gray); prevGray=gray; return; }

    // ——— homography warp of those 4 fiducials (TL→TR→BR→BL) ———
    cv::Mat H = cv::findHomography(srcPts, dstPts, cv::RANSAC);
    std::vector<cv::Point2f> warped(4);
    cv::perspectiveTransform(originalFiducials, warped, H);

    double err = 0;
    for (int i = 0; i < 4; ++i) err += cv::norm(corners[i].position - warped[i]);
    if (err >= 200.0) { std::cout<<"Rejected warp (err="<<err<<")\n"; prevGray=gray; return; }

    // draw 2D outline
    for (int i = 0; i < 4; ++i) {
        linesToDraw.push_back({
            warped[i],
            warped[(i+1)%4],
            cv::Scalar(0,255,255), // Yellow
            1                     // Thickness
        });
        corners[i].position = warped[i];
    }

// === 7. Unproject the four warped corners with median‐depth & real intrinsics ===

// 7.1 Grab your real intrinsics (you should have populated cameraMatrix from Realsense)
float fx = (float)cameraMatrix.at<double>(0,0);
float fy = (float)cameraMatrix.at<double>(1,1);
float cx = (float)cameraMatrix.at<double>(0,2);
float cy = (float)cameraMatrix.at<double>(1,2);

// 7.2 Build a list of depths under each warped corner
std::array<cv::Point2f,4> warped2d = {
    warped[0], warped[1], warped[2], warped[3]
};
std::vector<float> depths;
depths.reserve(4);
for (auto &uv : warped2d) {
    float d = sampleDepth(flipped_depth, uv);
    if (d > 0) depths.push_back(d);
}
if (depths.empty()) {
    std::cout<<"No valid depths under any corner\n";
    prevGray = gray;
    return;
}

// 7.3 Median‐filter to get a robust plane depth
std::sort(depths.begin(), depths.end());
float Z = depths[depths.size()/2] * 0.001f;  // m

// 7.4 Unproject each corner at that Z
std::array<Vector3,4> worldCorners;
for (int i = 0; i < 4; ++i) {
    auto uv = warped2d[i];

    // If your image is flipped 180°, unflip the pixel here:
    // float u = (W-1) - uv.x, v = (H-1) - uv.y;
    // But if you baked the flip into cameraMatrix already, skip this.

    float X = (uv.x - cx) * Z / fx;
    float Y = (uv.y - cy) * Z / fy;

    worldCorners[i] = { X, Y, Z };
}

// 7.5 Publish for drawing
{
    std::lock_guard<std::mutex> lock(poseMutex);
    currentWorldCorners = worldCorners;
    cornersValid = true;
}

prevGray = gray;
}

void Tracker::redetectFeatures(const cv::Mat& gray) {
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints);

    cv::Mat descriptors;
    descriptor->compute(gray, keypoints, descriptors);

    anchorPoints.clear();
    for (size_t i = 0; i < keypoints.size(); ++i)
        anchorPoints.push_back({keypoints[i].pt, descriptors.row((int)i).clone()});
}

void Tracker::drawCorners(cv::Mat& frame) {
    static const char* labels[] = {"TL", "TR", "BR", "BL"};
    for (size_t i = 0; i < corners.size(); ++i) {
        auto& corner = corners[i];
        cv::circle(frame, corner.position, 5, cv::Scalar(255, 255, 255), 1);
        cv::putText(frame, labels[i], corner.position + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

float Tracker::sampleDepth(const cv::Mat& flipped_depth, const cv::Point2f& pt) {
    int x = (int)pt.x;
    int y = (int)pt.y;
    if (x < 0 || y < 0 || x >= flipped_depth.cols || y >= flipped_depth.rows)
        return -1.0f;
    
    uint16_t depth_val = flipped_depth.at<uint16_t>(y, x);
    if (depth_val == 0) return -1.0f; // 0 = no depth data
    return static_cast<float>(depth_val); // in millimeters
}


int Tracker::findNearestCorner(int x, int y) {
    for (size_t i = 0; i < corners.size(); ++i)
        if (cv::norm(corners[i].position - cv::Point2f(x, y)) < 15)
            return (int)i;
    return -1;
}

void Tracker::mouseCallback(int event, int x, int y, int flags, void* userdata) {
    auto* self = reinterpret_cast<Tracker*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        int hit = self->findNearestCorner(x, y);
        if (hit >= 0) {
            self->draggingCorner = hit;
            self->corners[hit].dragging = true;
        } else if (self->corners.size() < 4) {
            self->corners.push_back({cv::Point2f(x, y), false, (int)self->corners.size()});
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        if (self->draggingCorner != -1)
            self->corners[self->draggingCorner].dragging = false;
        self->draggingCorner = -1;
    } else if (event == cv::EVENT_MOUSEMOVE && self->draggingCorner != -1) {
        self->corners[self->draggingCorner].position = cv::Point2f(x, y);
    }
}

void Tracker::run()
{
    cv::namedWindow("Push View");
    cv::setMouseCallback("Push View", mouseCallback, this);
    rs2::align alignToColor{ RS2_STREAM_COLOR };

    cv::Mat handMask, handRGBA;
    cv::Mat bgr, flippedDepth;

    while (true)
    {
        /* ------------ grab aligned frames ----------------------- */
        rs2::frameset fs;
        if (!pipe.poll_for_frames(&fs)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        fs = alignToColor.process(fs);
        rs2::video_frame color = fs.get_color_frame();
        rs2::depth_frame depth = fs.get_depth_frame();

        /* ------------ build flipped RGB + depth ----------------- */
        
        cv::flip(cv::Mat(color.get_height(), color.get_width(),
                         CV_8UC3, (void*)color.get_data()), bgr, -1);
        cv::flip(cv::Mat(depth.get_height(), depth.get_width(),
                         CV_16U, (void*)depth.get_data()), flippedDepth, -1);
        cv::Mat gray;  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        
        /* ------------ pose tracking ----------------------------- */
        cv::Mat rvec, tvec;
        std::optional<std::array<cv::Point2f,4>> warped;
            
        if (corners.size() == 4)
        {
            if (!tracking) { startTracking(gray); tracking = true; }
            else           { updateTracking(gray, bgr, flippedDepth); }
        
            warped = getPush2WarpedCorners();
            if (warped)
            {
                /* ---------- Pose (solvePnP) ------------------------------ */
                static const std::vector<cv::Point3f> mdl = {
                    {0,0,0}, {0.376f,0,0}, {0.376f,0.303f,0}, {0,0.303f,0}};
                std::vector<cv::Point2f> img(warped->begin(), warped->end());
                
                cv::solvePnP(mdl, img, cameraMatrix, distCoeffs,
                             rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

                /* ---------- fit plane ---------------------------------------- */
                cv::Mat R;  cv::Rodrigues(rvec, R);
                Eigen::Matrix3f Rc;
                for (int r=0;r<3;++r) for (int c=0;c<3;++c)
                    Rc(r,c) = (float)R.at<double>(r,c);
                Eigen::Vector3f tc((float)tvec.at<double>(0),
                                   (float)tvec.at<double>(1),
                                   (float)tvec.at<double>(2));

                Eigen::Vector3f P[4];
                for (int i=0;i<4;++i)
                    P[i] = Rc * Eigen::Vector3f(mdl[i].x, mdl[i].y, mdl[i].z) + tc;

                Eigen::Vector3f centre = (P[0]+P[1]+P[2]+P[3])*0.25f;
                Eigen::Vector3f normal = ((P[1]-P[0]).cross(P[3]-P[0])).normalized();

                /* make normal point toward camera (+Z camera axis) */
                if (normal.z() > 0) normal = -normal;

                planeEq = { normal.x(), normal.y(), normal.z(), -normal.dot(centre) };
            }
        }
        
        /* ---------- depth → mask ------------------------------------- */
        if (handMask.empty())
            handMask.create(flippedDepth.size(), CV_8UC1);
        handMask.setTo(0);

        const float MARGIN = 0.005f;                // 5 mm above deck
        const float a = planeEq[0], b = planeEq[1],
                c = planeEq[2], d = planeEq[3];

        for (int y = 0; y < flippedDepth.rows; ++y)
        {
        const uint16_t* dz = flippedDepth.ptr<uint16_t>(y);
        uint8_t*        m  = handMask.ptr<uint8_t>(y);
        
        for (int x = 0; x < flippedDepth.cols; ++x)
        {
            float Z = dz[x]*0.001f; if (!Z) continue;
        
            /* use the pixel as-is – NO extra flip ------------------- */
            float X = (x - cameraMatrix.at<double>(0,2)) * Z /
                       cameraMatrix.at<double>(0,0);
            float Y = (y - cameraMatrix.at<double>(1,2)) * Z /
                       cameraMatrix.at<double>(1,1);
        
            float dist = a*X + b*Y + c*Z + d;      // signed distance
            if (dist >  MARGIN)                    // strictly above plane
                m[x] = 255;
        }
        }

        /* clean + invert ---------------------------------------------- */
        cv::erode (handMask, handMask, cv::Mat(), {-1,-1}, 1);
        cv::dilate(handMask, handMask, cv::Mat(), {-1,-1}, 1);
        //cv::bitwise_not(handMask, handMask);          // 255 = hand, 0 = deck/scene
        
        /* ------------ mask → RGBA cut-out ----------------------- */
        if (!handMask.empty())
        {
            if (handRGBA.empty()) handRGBA.create(bgr.size(), CV_8UC4);
            const uchar* rgb=bgr.data,*msk=handMask.data; uchar* dst=handRGBA.data;
            int N=bgr.total();
            for(int i=0;i<N;++i){
                dst[4*i+0]=rgb[3*i+2]; dst[4*i+1]=rgb[3*i+1];
                dst[4*i+2]=rgb[3*i+0]; dst[4*i+3]=msk[i]?180:0;}
        } else handRGBA.release();

        /* ------------ publish to render thread ------------------ */
        cv::Mat rgba;  cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);
        {
            std::lock_guard<std::mutex> lk(frameSwapMutex);
            nextFrame.frame   = rgba.clone();
            nextFrame.rvec    = rvec;          // now non-empty
            nextFrame.tvec    = tvec;
            nextFrame.handRGBA= handRGBA.clone();
        }

        for (const auto& c : circlesToDraw) {
            cv::circle(bgr, c.center, c.radius, c.color, c.thickness);
        }
        
        for (const auto& l : linesToDraw) {
            cv::line(bgr, l.start, l.end, l.color, l.thickness);
        }
        drawCorners(bgr);
        cv::imshow("Push View", bgr);
        int k=cv::waitKey(1);
        if(k==27)break; if(k=='r'){tracking=false;corners.clear();}
    }
}
