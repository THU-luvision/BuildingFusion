#ifndef MOBILEGUI_HPP
#define MOBILEGUI_HPP

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <GL/glut.h>
#include <GL/glew.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
class MobileGUI
{
  public:
    MobileGUI(int showCaseMode, int im_width = 640, int im_height = 480)
    {

        showcaseMode =  showCaseMode;
        width = 1280;
        height = 980;
        imwidth = im_width;
        imheight = im_height;
        panel = 205;

        width += panel;

        RGB = "RGB";
        DepthNorm = "DepthNorm";
        //ModelImg = "ModelImg";
        //ModelNorm = "ModelNorm";
        Patches = "Patches";
        Cells = "Cells";
        int imageWidth = 640;
        int imageHeight = 480;


        pangolin::Params windowParams;
        windowParams.Set("SAMPLE_BUFFERS", 0);
        windowParams.Set("SAMPLES", 0);

        pangolin::CreateWindowAndBind("CollaborativeFusion", width, height, windowParams);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);


        pangolin::SetFullscreen(0);

//        gpuMem = new pangolin::Var<int>("ui.GPU memory free", 0);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);

        s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(imwidth, imheight, imheight , imheight, im_width / 2, imheight / 2, 0.1, 1000),
                                            pangolin::ModelViewLookAt(0, 0, -1, 0, 0, 1, pangolin::AxisNegY));
        pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, (-imwidth + 0.0f) / imheight)
                                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::Display(RGB).SetAspect((imwidth + 0.0f) / imheight);
        pangolin::Display(DepthNorm).SetAspect((imwidth + 0.0f)  / imheight);
        pangolin::Display(Patches).SetAspect((imwidth + 0.0f) /imheight);
        pangolin::Display(Cells).SetAspect((imwidth + 0.0f) /imheight);
        //pangolin::Display(ModelImg).SetAspect(imwidth / imheight);
        //pangolin::Display(ModelNorm).SetAspect(imwidth / imheight);

        pangolin::CreatePanel("ui").SetBounds(1.0, 0.5, 0.0, pangolin::Attach::Pix(panel));
        pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, showcaseMode ? 0 : pangolin::Attach::Pix(180), 1.0)
                                  .SetLayout(pangolin::LayoutEqualHorizontal)
                                  .AddDisplay(pangolin::Display(RGB))
                                  .AddDisplay(pangolin::Display(DepthNorm))
                                  .AddDisplay(pangolin::Display(Patches))
                                  .AddDisplay(pangolin::Display(Cells));
                                  //.AddDisplay(pangolin::Display(ModelImg))
                                  //.AddDisplay(pangolin::Display(ModelNorm));

        pause = new pangolin::Var<bool>("ui.Pause", false, true);
        step = new pangolin::Var<bool>("ui.Step", false, false);
//        save = new pangolin::Var<bool>("ui.Save", false, false);
        reset = new pangolin::Var<bool>("ui.Reset", false, false);
//        flipColors = new pangolin::Var<bool>("ui.Flip RGB", false, true);

//        pyramid = new pangolin::Var<bool>("ui.Pyramid", true, true);
//        so3 = new pangolin::Var<bool>("ui.SO(3)", true, true);
//        frameToFrameRGB = new pangolin::Var<bool>("ui.Frame to frame RGB", false, true);
//        fastOdom = new pangolin::Var<bool>("ui.Fast Odometry", false, true);
//        rgbOnly = new pangolin::Var<bool>("ui.RGB only tracking", false, true);
//        confidenceThreshold = new pangolin::Var<float>("ui.Confidence threshold", 10.0, 0.0, 24.0);
//        depthCutoff = new pangolin::Var<float>("ui.Depth cutoff", 10.0, 0.0, 12.0);
//        icpWeight = new pangolin::Var<float>("ui.ICP weight", 10.0, 0.0, 100.0);

        followPose = new pangolin::Var<bool>("ui.Follow pose", true, true);
//        drawRawCloud = new pangolin::Var<bool>("ui.Draw raw", false, true);
//        drawFilteredCloud = new pangolin::Var<bool>("ui.Draw filtered", false, true);
        drawGlobalModel = new pangolin::Var<bool>("ui.Draw global model", true, true);
//        drawUnstable = new pangolin::Var<bool>("ui.Draw unstable points", false, true);
//        drawPoints = new pangolin::Var<bool>("ui.Draw points", false, true);
        drawColors = new pangolin::Var<bool>("ui.Draw colors", false, true);
        drawSemanticLabel = new pangolin::Var<bool>("ui.Draw semantic label", false, true);
        drawInstanceLabel = new pangolin::Var<bool>("ui.Draw instance label", false, true);
//        drawFxaa = new pangolin::Var<bool>("ui.Draw FXAA", showcaseMode, true);
//        drawWindow = new pangolin::Var<bool>("ui.Draw time window", false, true);
        drawNormals = new pangolin::Var<bool>("ui.Draw normals", false, true);
//        drawTimes = new pangolin::Var<bool>("ui.Draw times", false, true);
    }
    virtual ~MobileGUI()
    {
 //       delete gpuMem;
        delete renderBuffer;
        delete colorFrameBuffer;
    }
    void drawPyramidWireframe(float r, float g, float b) 
    {
        float pyrH = 0.5f;
        float pyrW = 0.25f;
        glLineWidth(3.0f);
        glColor4f(r,g,b,0.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glEnd();
    }
    void drawPyramidWireframe(const std::string &color) {
        float pyrH = 0.5f;
        float pyrW = 0.25f;
        glLineWidth(3.0f);
        if(color =="black")
        glColor4f(1.0f, 1.0f, 0.0f, 0.0f);
        else if(color=="red")
        glColor4f(1.0f,0.0f,0.0f,0.0f);
        else if(color=="green")
        glColor4f(0.0f,1.0f,0.0f,0.0f);
        else if(color=="blue")
        glColor4f(0.0f,0.0f,1.0f,0.0f);
        else if(color == "white")
        glColor4f(1.0f,1.0f,1.0f,0.0f);
        else 
        glColor4f(1.0f,0.0f,1.0f,0.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glEnd();
    }
    void drawCamera(Sophus::SE3d currPose, float r, float g, float b)
    {
        Eigen::Matrix3f R = currPose.rotationMatrix().cast<float>();
        Eigen::Vector3f t = currPose.translation().cast<float>();
        Eigen::Isometry3f cpose = Eigen::Isometry3f::Identity();
        cpose.prerotate(R);
        cpose.pretranslate(t);
         
        glPushMatrix();
        //glColor4f(1.0, 0.0f, 0.0f, 1.0f);
        glMultMatrixf(cpose.data());
        drawPyramidWireframe(r,g,b);
        glPopMatrix();
        glColor4f(1.0f,1.0f,1.0f,0.0f);        
    }
    void drawCamera(Sophus::SE3d currPose,const std::string &color= "black")
    {
        //PoseSE3d currPose = gcSLAM.globalFrameList.back().pose_sophus[0];
        Eigen::Matrix3f R = currPose.rotationMatrix().cast<float>();
        Eigen::Vector3f t = currPose.translation().cast<float>();
        Eigen::Isometry3f cpose = Eigen::Isometry3f::Identity();
        cpose.prerotate(R);
        cpose.pretranslate(t);
         
      glPushMatrix();
      //glColor4f(1.0, 0.0f, 0.0f, 1.0f);
      glMultMatrixf(cpose.data());
      drawPyramidWireframe(color);
      glPopMatrix();
      glColor4f(1.0f,1.0f,1.0f,0.0f);
    }
    void PreCall()
    {
        glClearColor(1.0f,1.0f, 1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        width = pangolin::DisplayBase().v.w;
        height = pangolin::DisplayBase().v.h;

        pangolin::Display("cam").Activate(s_cam);
    }
    void PostCall()
    {
//        GLint cur_avail_mem_kb = 0;
//        glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);

//        int memFree = cur_avail_mem_kb / 1024;

//        gpuMem->operator=(memFree);

        pangolin::FinishFrame();

        glFinish();
    }

    void loadImageToTexture(pangolin::GlTexture *texture, unsigned char*imageArray)
    {
        texture->Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);
    }

    void DisplayImg(const std::string & id, pangolin::GlTexture *img)
    {

        glDisable(GL_DEPTH_TEST);

        pangolin::Display(id).Activate();
        img->RenderToViewport(true);
        glEnable(GL_DEPTH_TEST);
    }

    void setModelView(Eigen::Matrix4f &currPose, int iclnuim)
    {

        pangolin::OpenGlMatrix mv;
        Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

        Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
        Eigen::Quaternionf currQuat(currRot);
        Eigen::Vector3f forwardVector(0, 0, 1);
        Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

        Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
        Eigen::Vector3f up = (currQuat * upVector).normalized();

        Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

        eye -= forward;

        Eigen::Vector3f at = eye + forward;

        Eigen::Vector3f z = (eye - at).normalized();  // Forward
        Eigen::Vector3f x = up.cross(z).normalized(); // Right
        Eigen::Vector3f y = z.cross(x);

        Eigen::Matrix4d m;
        m << x(0),  x(1),  x(2),  -(x.dot(eye)),
             y(0),  y(1),  y(2),  -(y.dot(eye)),
             z(0),  z(1),  z(2),  -(z.dot(eye)),
                0,     0,     0,              1;

        memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

        s_cam.SetModelViewMatrix(mv);
    }

    bool showcaseMode;
    int width;
    int height;
    int imwidth;
    int imheight;
    pangolin::GlRenderBuffer * renderBuffer;
    pangolin::GlFramebuffer * colorFrameBuffer;

    pangolin::OpenGlRenderState s_cam;
    pangolin::Var<int> * gpuMem;

    std::string RGB;
    std::string DepthNorm;
    //std::string ModelImg;
    //std::string ModelNorm;
    std::string Patches;
    std::string Cells;
    int panel;


    pangolin::Var<bool> * pause,
                        * step,
                        //* save,
                        * reset,
                        //* flipColors,
                        //* rgbOnly,
                        //* pyramid,
                        //* so3,
                        //* frameToFrameRGB,
                        //* fastOdom,
                        * followPose,
                        //* drawRawCloud,
                        //* drawFilteredCloud,
                        * drawNormals,
                        //* autoSettings,
                        //* drawDefGraph,
                        * drawColors,
                        //* drawFxaa,
                        * drawGlobalModel,
                        * drawSemanticLabel,
                        * drawInstanceLabel;
                        //* drawUnstable,
                        //* drawPoints,
                        //* drawTimes,
                        //* drawFerns,
                        //* drawDeforms;
                        //* drawWindow;

    pangolin::Var<std::string> * totalPoints,
                               * totalNodes,
                               * totalFerns,
                               * totalDefs,
                               * totalFernDefs,
                               * trackInliers,
                               * trackRes,
                               * logProgress;
    pangolin::Var<float> * confidenceThreshold,
                         * depthCutoff,
                         * icpWeight;
};


#endif // MOBILEGUI_HPP
