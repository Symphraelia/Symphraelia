#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "Tracker.hpp"
#include <opencv2/core/ocl.hpp> 

#include <thread>
#include <opencv2/opencv.hpp>
#include <iostream>
//#include <iostream>


bool flip_rx = false;
bool flip_ry = false;
bool flip_rz = false;
bool flip_tx = false;
bool flip_ty = false;
bool flip_tz = false;


// -----------------------------------------------------------------------------
// Controller-top outer size  (same numbers you used for the yellow box)
// -----------------------------------------------------------------------------
constexpr float CTRL_W = 0.376f;   // 376 mm  (left ↔ right)
constexpr float CTRL_H = 0.304f;   // 304 mm  (top  ↔ bottom)

// measured clearances
constexpr float PADS_FROM_LEFT   = 0.070f;   //  70 mm
constexpr float PADS_FROM_RIGHT  = 0.080f;   //  80 mm
constexpr float PADS_FROM_TOP    = 0.1234f;   // 120 mm
constexpr float PADS_FROM_BOTTOM = 0.010f;   //  11 mm

// layout
constexpr int   COLS = 8;
constexpr int   ROWS = 8;
constexpr float GAP  = 0.003f;     //   3 mm  between pad edges

// -----------------------------------------------------------------------------
// Derived pad size (same width & height for every pad)
// -----------------------------------------------------------------------------
constexpr float USABLE_W = CTRL_W - PADS_FROM_LEFT - PADS_FROM_RIGHT;    // inner box
constexpr float USABLE_H = CTRL_H - PADS_FROM_TOP  - PADS_FROM_BOTTOM;

constexpr float PAD_W = (USABLE_W - (COLS-1) * GAP) / COLS;              
constexpr float PAD_H = (USABLE_H - (ROWS-1) * GAP) / ROWS;              

static_assert(PAD_W > 0 && PAD_H > 0, "pad size must be positive");

// -----------------------------------------------------------------------------
// Pad array
// -----------------------------------------------------------------------------

struct Pad {
    Vector3 centre;   // model space
    int     row, col;
};

constexpr Pad makePad(int r,int c)
{
    // centre = edgeOffset + halfPad + index * (pad + gap)
    return Pad {
        {  PADS_FROM_LEFT + PAD_W*0.5f + c * (PAD_W + GAP),
           PADS_FROM_TOP  + PAD_H*0.5f + r * (PAD_H + GAP),
           0.0f },
        r, c
    };
}

constexpr std::array<Pad,ROWS*COLS> pads = []{
    std::array<Pad,ROWS*COLS> p {};
    for (int r = 0; r < ROWS; ++r)
        for (int c = 0; c < COLS; ++c)
            p[r*COLS + c] = makePad(r,c);
    return p;
}();


void DrawPadGrid(Color boxClr, Color dotClr)
{
    constexpr int ROWS = 8, COLS = 8;

    // pad half-sizes (for one-line maths)
    const float hx = PAD_W * 0.5f;
    const float hy = PAD_H * 0.5f;

    for (const Pad& p : pads)
    {
        // model-space corners of this pad
        Vector3 tl = { p.centre.x - hx, p.centre.y - hy, 0.0f };
        Vector3 tr = { p.centre.x + hx, p.centre.y - hy, 0.0f };
        Vector3 br = { p.centre.x + hx, p.centre.y + hy, 0.0f };
        Vector3 bl = { p.centre.x - hx, p.centre.y + hy, 0.0f };

        // outline
        DrawLine3D( tl, tr, boxClr );
        DrawLine3D( tr, br, boxClr );
        DrawLine3D( br, bl, boxClr );
        DrawLine3D( bl, tl, boxClr );

        // centre dot (small vertical segment so it’s visible from any angle)
        Vector3 top =  { p.centre.x, p.centre.y,  0.003f };
        Vector3 base = { p.centre.x, p.centre.y,  0.000f };
        DrawSphereEx(p.centre, 0.002f, 8, 8, dotClr);
    }
}


// ---------------------------------------------------------------------------
//  DrawPush2BoxFromPose
//  rvec, tvec : OpenCV camera coords (+X right, +Y down, +Z forward)
// ---------------------------------------------------------------------------
void DrawPush2BoxFromPose(const cv::Mat& rvec, const cv::Mat& tvec,
    Color boxColor)
{
if (rvec.empty() || tvec.empty()) return;

cv::Mat Rcv;
cv::Rodrigues(rvec, Rcv);

Matrix M = { 0 };

// column 0  (X axis – keep signs)
M.m0 =  (float)Rcv.at<double>(0,0);
M.m1 = -(float)Rcv.at<double>(1,0);   // –Y
M.m2 = -(float)Rcv.at<double>(2,0);   // –Z

// column 1  (Y axis)
M.m4 =  (float)Rcv.at<double>(0,1);
M.m5 = -(float)Rcv.at<double>(1,1);
M.m6 = -(float)Rcv.at<double>(2,1);

// column 2  (Z axis)
M.m8  =  (float)Rcv.at<double>(0,2);
M.m9  = -(float)Rcv.at<double>(1,2);
M.m10 = -(float)Rcv.at<double>(2,2);

// translation
M.m12 =  (float)tvec.at<double>(0);   //  X
M.m13 = -(float)tvec.at<double>(1);   // –Y
M.m14 = -(float)tvec.at<double>(2);   // –Z
M.m15 =  1.0f;

rlPushMatrix();
rlMultMatrixf(MatrixToFloat(M));
/* … draw unit wire-box (unchanged) … */
static const Vector3 v[8] = { {0,0,0},{.376,0,0},{.376,.304,0},{0,.304,0},
                {0,0,.03},{.376,0,.03},{.376,.304,.03},{0,.304,.03} };
static const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},
              {6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
for (int i=0;i<12;++i) DrawLine3D(v[e[i][0]], v[e[i][1]], boxColor);
DrawPadGrid(GREEN, RED);
rlPopMatrix();
}


Matrix CreateProjectionFromOpenCV(const cv::Mat& K,
    int width, int height,
    float nearZ, float farZ)
{
double fx = K.at<double>(0,0);
double fy = K.at<double>(1,1);
double cx = K.at<double>(0,2);
double cy = K.at<double>(1,2);

float L = static_cast<float>(-cx           * nearZ / fx);
float R = static_cast<float>((width - cx)  * nearZ / fx);
float B = static_cast<float>( cy           * nearZ / fy);
float T = static_cast<float>((cy - height) * nearZ / fy);

Matrix M = { 0 };

M.m0  =  2.0f * nearZ / (R - L);
M.m5  =  -2.0f * nearZ / (T - B);      // stays positive
M.m8  =  (R + L) / (R - L);
M.m9  =  (T + B) / (T - B);

M.m10 = -(farZ + nearZ) / (farZ - nearZ);
M.m11 = -1.0f;                        // perspective term
M.m14 = -(2.0f * farZ * nearZ) / (farZ - nearZ);

M.m15 =  0.0f;                        // **must be 0 in a perspective matrix**

return M;
}


Texture2D handMaskTex = { 0 };

int main()
{
    std::cout << "Have OpenCL: " << cv::ocl::haveOpenCL() << "\n";
    std::cout << "Use OpenCL: " << cv::ocl::useOpenCL() << "\n";
    std::cout << "OpenCL Device: " << cv::ocl::Device::getDefault().name() << "\n";
    /* ---------- window & camera ---------------------------------- */
    const int camW = 1280, camH = 720;
    InitWindow(camW, camH, "Push-2 AR Visualizer");
    SetTargetFPS(165);

    /* ---------- start tracker thread ----------------------------- */
    Tracker tracker;
    std::thread worker([&]{ tracker.run(); });

    /* ---------- projection from intrinsics ----------------------- */
    CameraReal camReal = {
        .position   ={0,0,0},
        .target     ={0,0,-1},
        .up         ={0,1,0},
        .fovy       =45,
        .projection =CAMERA_PERSPECTIVE,
        .opencvProjection =
            CreateProjectionFromOpenCV(tracker.getCameraMatrix(),
                                       camW, camH, 0.01f, 100.f),
        .useCustomProjection = true
    };

    /* ---------- textures ----------------------------------------- */
    Image blank = GenImageColor(camW, camH, BLANK);
    Texture2D videoTex = LoadTextureFromImage(blank);
    UnloadImage(blank);

    Texture2D handTex = {0};                       // RGBA cut-out

    /* ---------- render loop -------------------------------------- */
    while (!WindowShouldClose())
    {
        /* sync with tracker -------------------------------------- */
        FrameBundle f;
        { std::lock_guard<std::mutex> lk(tracker.frameSwapMutex);
          f = tracker.nextFrame; }

        /* upload video frame ------------------------------------- */
        if (!f.frame.empty() &&
            f.frame.size() == cv::Size(camW, camH))
            UpdateTexture(videoTex, f.frame.ptr());

        /* upload hand RGBA --------------------------------------- */
        if (!f.handRGBA.empty())
        {
            int w = f.handRGBA.cols, h = f.handRGBA.rows;

            if (!handTex.id || handTex.width != w || handTex.height != h)
            {
                if (handTex.id) UnloadTexture(handTex);
                Image img = { f.handRGBA.ptr(), w, h,
                              1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
                handTex = LoadTextureFromImage(img);
            }
            UpdateTexture(handTex, f.handRGBA.ptr());
        } //*/

        /* draw ---------------------------------------------------- */
        BeginDrawing();
            ClearBackground(BLANK);
            DrawTexture(videoTex, 0, 0, WHITE);         // live camera

            BeginModeReal3D(camReal);                   // 3-D overlay
                if (!f.rvec.empty())
                    DrawPush2BoxFromPose(f.rvec, f.tvec, YELLOW);
                /* …other virtual objects… */
            EndMode3D();
            /**/
            if (handTex.id)                             // coloured hands
            {
                BeginBlendMode(BLEND_ALPHA);            // use per-pixel A
                Rectangle src = {0,0,
                                 (float)handTex.width,
                                (float)handTex.height}; // flip Y
                Rectangle dst = {0,0,
                                 (float)GetScreenWidth(),
                                 (float)GetScreenHeight()};
                DrawTexturePro(handTex, src, dst, {0,0}, 0, WHITE);
                EndBlendMode();
            }//*/

            
            DrawFPS(camW-100, 10);
        EndDrawing();
    }

    /* ---------- shutdown ---------------------------------------- */
    CloseWindow();
    worker.join();
    return 0;
}
