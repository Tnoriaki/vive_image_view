#ifndef _VIVE_IMAGE_VIEW_H_
#define _VIVE_IMAGE_VIEW_H_

#include <ros/package.h>
#include <image_view/ImageViewConfig.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#define X 0
#define Y 1
#define XY 2
#define L 0
#define R 1
#define LR 2
#define HMD 0
#define LC 1
#define RC 2
#define HMD_LC_RC 3
//========= Copyright Valve Corporation ============//

#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <GL/glu.h>
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <openvr.h>
#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#include <sstream>


bool VRCompositor_Ready = false;
boost::mutex mtx;
std::string txt_ros;
ros::WallTime t_gl_old,t_gl_now, t_ros_old,t_ros_now, t_cb_old,t_cb_now, t_cb_l_old,t_cb_l_now, t_cb_r_old,t_cb_r_now, t_th_old,t_th_now, t_cb_cp_old,t_cb_cp_now;
bool ros_image_isNew_mono = false, ros_image_isNew[LR] = {false,false};
double cam_f[LR][XY] = {{600,600},{600,600}};
const double hmd_fov = 110*M_PI/180;//field of view
const int hmd_panel_size[XY] = {1080,1200};//pixel
cv::Mat ros_image_stereo_resized[LR];
cv::Mat hmd_panel_img[LR] = {
    cv::Mat(cv::Size(hmd_panel_size[X], hmd_panel_size[Y]), CV_8UC3, CV_RGB(0,0,0)),
    cv::Mat(cv::Size(hmd_panel_size[X], hmd_panel_size[Y]), CV_8UC3, CV_RGB(0,0,0)),
};

enum IMAGE_VIEW_MODE{
  NORMAL,
  STEREO
} view_mode;


//TODO: proper linux compatibility
#ifdef __linux__
#include <shared/linuxcompathack.h>
#endif

#if defined(POSIX)
#include "unistd.h"
#endif

void ThreadSleep( unsigned long nMilliseconds ){
#if defined(_WIN32)
	::Sleep( nMilliseconds );
#elif defined(POSIX)
	usleep( nMilliseconds * 1000 );
#endif
}

class CGLRenderModel{
public:
	CGLRenderModel( const std::string & sRenderModelName );
	~CGLRenderModel();
	bool BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture );
	void Cleanup();
	void Draw();
	const std::string & GetName() const { return m_sModelName; }
private:
	GLuint m_glVertBuffer;
	GLuint m_glIndexBuffer;
	GLuint m_glVertArray;
	GLuint m_glTexture;
	GLsizei m_unVertexCount;
	std::string m_sModelName;
};

static bool g_bPrintf = true;

class CMainApplication{
public:
	CMainApplication( int argc, char *argv[] );
	virtual ~CMainApplication();
	bool BInit();
	bool BInitGL();
	bool BInitCompositor();
	void SetupRenderModels();
	void Shutdown();
	void RunMainLoop();
	bool HandleInput();
	void ProcessVREvent( const vr::VREvent_t & event );
	void RenderFrame();
	bool SetupTexturemaps();
	void SetupScene();
//	void AddCubeToScene( Matrix4 mat, std::vector<float> &vertdata );
	void AddCubeVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata );
	void DrawControllers();
	bool SetupStereoRenderTargets();
	void SetupDistortion();
	void SetupCameras();
	void RenderStereoTargets();
	void RenderDistortion();
	void RenderScene( vr::Hmd_Eye nEye );
	Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
	Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
	Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
	void UpdateHMDMatrixPose();
	Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose );
	GLuint CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader );
	bool CreateAllShaders();
	void SetupRenderModelForTrackedDevice( vr::TrackedDeviceIndex_t unTrackedDeviceIndex );
	CGLRenderModel *FindOrLoadRenderModel( const char *pchRenderModelName );
	bool UpdateTexturemaps();
	cv::Mat ros_image_monoeye,ros_image_stereo[LR];
	geometry_msgs::PoseStamped ros_pose[HMD_LC_RC];
private: 
	bool m_bDebugOpenGL;
	bool m_bVerbose;
	bool m_bPerf;
	bool m_bVblank;
	bool m_bGlFinishHack;
	vr::IVRSystem *m_pHMD;
	vr::IVRRenderModels *m_pRenderModels;
	std::string m_strDriver;
	std::string m_strDisplay;
	vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
	Matrix4 m_rmat4DevicePose[ vr::k_unMaxTrackedDeviceCount ];
	bool m_rbShowTrackedDevice[ vr::k_unMaxTrackedDeviceCount ];
private: // SDL bookkeeping
	SDL_Window *m_pWindow;
	uint32_t m_nWindowWidth;
	uint32_t m_nWindowHeight;
	SDL_GLContext m_pContext;
private: // OpenGL bookkeeping
	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	bool m_bShowCubes;
	std::string m_strPoseClasses;                            // what classes we saw poses for this frame
	char m_rDevClassChar[ vr::k_unMaxTrackedDeviceCount ];   // for each device, a character representing its class
	int m_iSceneVolumeWidth;
	int m_iSceneVolumeHeight;
	int m_iSceneVolumeDepth;
	float m_fScaleSpacing;
	float m_fScale;
	int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20
	float m_fNearClip;
	float m_fFarClip;
	GLuint m_iTexture;
	GLuint m_EyeTexture[LR];
	unsigned int m_uiVertcount;
	GLuint m_glSceneVertBuffer;
	GLuint m_unSceneVAO;
	GLuint m_unLensVAO;
	GLuint m_glIDVertBuffer;
	GLuint m_glIDIndexBuffer;
	unsigned int m_uiIndexSize;
	GLuint m_glControllerVertBuffer;
	GLuint m_unControllerVAO;
	unsigned int m_uiControllerVertcount;
	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;
	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;
	struct VertexDataScene	{
		Vector3 position;
		Vector2 texCoord;
	};
	struct VertexDataLens	{
		Vector2 position;
		Vector2 texCoordRed;
		Vector2 texCoordGreen;
		Vector2 texCoordBlue;
	};
	GLuint m_unSceneProgramID;
	GLuint m_unLensProgramID;
	GLuint m_unControllerTransformProgramID;
	GLuint m_unRenderModelProgramID;
	GLint m_nSceneMatrixLocation;
	GLint m_nControllerMatrixLocation;
	GLint m_nRenderModelMatrixLocation;
	struct FramebufferDesc	{
		GLuint m_nDepthBufferId;
		GLuint m_nRenderTextureId;
		GLuint m_nRenderFramebufferId;
		GLuint m_nResolveTextureId;
		GLuint m_nResolveFramebufferId;
	};
	FramebufferDesc leftEyeDesc;
	FramebufferDesc rightEyeDesc;
	bool CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc );
	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;
	std::vector< CGLRenderModel * > m_vecRenderModels;
	CGLRenderModel *m_rTrackedDeviceToRenderModel[ vr::k_unMaxTrackedDeviceCount ];
};

void dprintf( const char *fmt, ... ){
	va_list args;
	char buffer[ 2048 ];
	va_start( args, fmt );
	vsprintf_s( buffer, fmt, args );
	va_end( args );
	if ( g_bPrintf )
		printf( "%s", buffer );
	OutputDebugStringA( buffer );
}

CMainApplication *pMainApplication;

#endif
