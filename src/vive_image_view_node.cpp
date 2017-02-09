#include "vive_image_view.h"

CMainApplication::CMainApplication( int argc, char *argv[] )
	: m_pWindow(NULL)
	, m_pContext(NULL)
	, m_nWindowWidth( 1280 )
	, m_nWindowHeight( 720 )
	, m_unSceneProgramID( 0 )
	, m_unLensProgramID( 0 )
	, m_unControllerTransformProgramID( 0 )
	, m_unRenderModelProgramID( 0 )
	, m_pHMD( NULL )
	, m_pRenderModels( NULL )
	, m_bDebugOpenGL( false )
	, m_bVerbose( false )
	, m_bPerf( false )
	, m_bVblank( false )
	, m_bGlFinishHack( true )
	, m_glControllerVertBuffer( 0 )
	, m_unControllerVAO( 0 )
	, m_unLensVAO( 0 )
	, m_unSceneVAO( 0 )
	, m_nSceneMatrixLocation( -1 )
	, m_nControllerMatrixLocation( -1 )
	, m_nRenderModelMatrixLocation( -1 )
	, m_iTrackedControllerCount( 0 )
	, m_iTrackedControllerCount_Last( -1 )
	, m_iValidPoseCount( 0 )
	, m_iValidPoseCount_Last( -1 )
	, m_iSceneVolumeInit( 20 )
	, m_strPoseClasses("")
	, m_bShowCubes( false )
{

	for( int i = 1; i < argc; i++ )	{
		if( !stricmp( argv[i], "-gldebug" ) )		{
			m_bDebugOpenGL = true;
		}
		else if( !stricmp( argv[i], "-verbose" ) )		{
			m_bVerbose = true;
		}
		else if( !stricmp( argv[i], "-novblank" ) )		{
			m_bVblank = false;
		}
		else if( !stricmp( argv[i], "-noglfinishhack" ) )		{
			m_bGlFinishHack = false;
		}
		else if( !stricmp( argv[i], "-noprintf" ) )		{
			g_bPrintf = false;
		}
		else if ( !stricmp( argv[i], "-cubevolume" ) && ( argc > i + 1 ) && ( *argv[ i + 1 ] != '-' ) )		{
			m_iSceneVolumeInit = atoi( argv[ i + 1 ] );
			i++;
		}
	}
	// other initialization tasks are done in BInit
	memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));
};

CMainApplication::~CMainApplication(){
	// work is done in Shutdown
	dprintf( "Shutdown" );
}

std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL ){
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
	if( unRequiredBufferLen == 0 )return "";
	char *pchBuffer = new char[ unRequiredBufferLen ];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
	std::string sResult = pchBuffer;
	delete [] pchBuffer;
	return sResult;
}

bool CMainApplication::BInit(){
	if ( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_TIMER ) < 0 )	{
		printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}
	// Loading the SteamVR Runtime
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
	if ( eError != vr::VRInitError_None )	{
		m_pHMD = NULL;
		char buf[1024];
		sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
		SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
		return false;
	}
	m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface( vr::IVRRenderModels_Version, &eError );
	if( !m_pRenderModels )	{
		m_pHMD = NULL;
		vr::VR_Shutdown();
		char buf[1024];
		sprintf_s( buf, sizeof( buf ), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
		SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
		return false;
	}
	int nWindowPosX = 700;
	int nWindowPosY = 100;
	m_nWindowWidth = 1280;
	m_nWindowHeight = 720;
	Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 4 );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );
	//SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );
	SDL_GL_SetAttribute( SDL_GL_MULTISAMPLEBUFFERS, 0 );
	SDL_GL_SetAttribute( SDL_GL_MULTISAMPLESAMPLES, 0 );
	if( m_bDebugOpenGL )
		SDL_GL_SetAttribute( SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG );
	  m_pWindow = SDL_CreateWindow( "hellovr_sdl", nWindowPosX, nWindowPosY, m_nWindowWidth, m_nWindowHeight, unWindowFlags );
	if (m_pWindow == NULL)	{
		printf( "%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
		return false;
	}
	m_pContext = SDL_GL_CreateContext(m_pWindow);
	if (m_pContext == NULL)	{
		printf( "%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
		return false;
	}
	glewExperimental = GL_TRUE;
	GLenum nGlewError = glewInit();
	if (nGlewError != GLEW_OK)	{
		printf( "%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString( nGlewError ) );
		return false;
	}
	glGetError(); // to clear the error caused deep in GLEW
	if ( SDL_GL_SetSwapInterval( m_bVblank ? 1 : 0 ) < 0 )	{
		printf( "%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError() );
		return false;
	}
	m_strDriver = "No Driver";
	m_strDisplay = "No Display";
	m_strDriver = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
	m_strDisplay = GetTrackedDeviceString( m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );
	std::string strWindowTitle = "hellovr_sdl - " + m_strDriver + " " + m_strDisplay;
	SDL_SetWindowTitle( m_pWindow, strWindowTitle.c_str() );
		// cube array
 	m_iSceneVolumeWidth = m_iSceneVolumeInit;
 	m_iSceneVolumeHeight = m_iSceneVolumeInit;
 	m_iSceneVolumeDepth = m_iSceneVolumeInit;
 	m_fScale = 0.3f;
 	m_fScaleSpacing = 4.0f;
 	m_fNearClip = 0.1f;
 	m_fFarClip = 30.0f;
 	m_iTexture = 0;
	m_EyeTexture[L] = m_EyeTexture[R] = 0;
 	m_uiVertcount = 0;
	if (!BInitGL())	{
		printf("%s - Unable to initialize OpenGL!\n", __FUNCTION__);
		return false;
	}
	if (!BInitCompositor())	{
		printf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
		return false;
	}
	return true;
}


void APIENTRY DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, const void* userParam){
	dprintf( "GL Error: %s\n", message );
}

bool CMainApplication::BInitGL(){
	if( m_bDebugOpenGL )	{
		glDebugMessageCallback( (GLDEBUGPROC)DebugCallback, nullptr);
		glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE );
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	}
	if( !CreateAllShaders() )
		return false;
	SetupTexturemaps();
	SetupScene();
	SetupCameras();
	SetupStereoRenderTargets();
	SetupDistortion();
	SetupRenderModels();
	return true;
}

bool CMainApplication::BInitCompositor(){
	vr::EVRInitError peError = vr::VRInitError_None;
	if ( !vr::VRCompositor() )	{
		printf( "Compositor initialization failed. See log file for details\n" );
		return false;
	}
	return true;
}

void CMainApplication::Shutdown(){
	if( m_pHMD )	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
	}
	for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )	{
		delete (*i);
	}
	m_vecRenderModels.clear();
	if( m_pContext )	{
		glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE );
		glDebugMessageCallback(nullptr, nullptr);
		glDeleteBuffers(1, &m_glSceneVertBuffer);
		glDeleteBuffers(1, &m_glIDVertBuffer);
		glDeleteBuffers(1, &m_glIDIndexBuffer);

		if ( m_unSceneProgramID )		{
			glDeleteProgram( m_unSceneProgramID );
		}
		if ( m_unControllerTransformProgramID )		{
			glDeleteProgram( m_unControllerTransformProgramID );
		}
		if ( m_unRenderModelProgramID )		{
			glDeleteProgram( m_unRenderModelProgramID );
		}
		if ( m_unLensProgramID )		{
			glDeleteProgram( m_unLensProgramID );
		}
		glDeleteRenderbuffers( 1, &leftEyeDesc.m_nDepthBufferId );
		glDeleteTextures( 1, &leftEyeDesc.m_nRenderTextureId );
		glDeleteFramebuffers( 1, &leftEyeDesc.m_nRenderFramebufferId );
		glDeleteTextures( 1, &leftEyeDesc.m_nResolveTextureId );
		glDeleteFramebuffers( 1, &leftEyeDesc.m_nResolveFramebufferId );
		glDeleteRenderbuffers( 1, &rightEyeDesc.m_nDepthBufferId );
		glDeleteTextures( 1, &rightEyeDesc.m_nRenderTextureId );
		glDeleteFramebuffers( 1, &rightEyeDesc.m_nRenderFramebufferId );
		glDeleteTextures( 1, &rightEyeDesc.m_nResolveTextureId );
		glDeleteFramebuffers( 1, &rightEyeDesc.m_nResolveFramebufferId );
		if( m_unLensVAO != 0 )		{
			glDeleteVertexArrays( 1, &m_unLensVAO );
		}
		if( m_unSceneVAO != 0 )		{
			glDeleteVertexArrays( 1, &m_unSceneVAO );
		}
		if( m_unControllerVAO != 0 )		{
			glDeleteVertexArrays( 1, &m_unControllerVAO );
		}
	}

	if( m_pWindow )	{
		SDL_DestroyWindow(m_pWindow);
		m_pWindow = NULL;
	}

	SDL_Quit();
}

bool CMainApplication::HandleInput(){
	SDL_Event sdlEvent;
	bool bRet = false;
	while ( SDL_PollEvent( &sdlEvent ) != 0 )	{
		if ( sdlEvent.type == SDL_QUIT )		{
			bRet = true;
		}
		else if ( sdlEvent.type == SDL_KEYDOWN )		{
			if ( sdlEvent.key.keysym.sym == SDLK_ESCAPE  || sdlEvent.key.keysym.sym == SDLK_q )	{
				bRet = true;
			}
			if( sdlEvent.key.keysym.sym == SDLK_c )	{
				m_bShowCubes = !m_bShowCubes;
			}
		}
	}
	// Process SteamVR events
	vr::VREvent_t event;
	while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) )	{
		ProcessVREvent( event );
	}
	// Process SteamVR controller state
	for( vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++ )	{
		vr::VRControllerState_t state;
		if( m_pHMD->GetControllerState( unDevice, &state ) )		{
			m_rbShowTrackedDevice[ unDevice ] = state.ulButtonPressed == 0;
		}
	}
	return bRet;
}

void CMainApplication::RunMainLoop(){
	bool bQuit = false;
	SDL_StartTextInput();
	SDL_ShowCursor( SDL_DISABLE );
	while ( !bQuit )	{
		bQuit = HandleInput();
		RenderFrame();
	}
	SDL_StopTextInput();
}


//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------
void CMainApplication::ProcessVREvent( const vr::VREvent_t & event ){
	switch( event.eventType )	{
	case vr::VREvent_TrackedDeviceActivated:
	  {
			SetupRenderModelForTrackedDevice( event.trackedDeviceIndex );
			dprintf( "Device %u attached. Setting up render model.\n", event.trackedDeviceIndex );
		}
		break;
	case vr::VREvent_TrackedDeviceDeactivated:
		{
			dprintf( "Device %u detached.\n", event.trackedDeviceIndex );
		}
		break;
	case vr::VREvent_TrackedDeviceUpdated:
		{
			dprintf( "Device %u updated.\n", event.trackedDeviceIndex );
		}
		break;
	}
}

void CMainApplication::RenderFrame(){
	// for now as fast as possible
	if ( m_pHMD )	{
		DrawControllers();
		RenderStereoTargets();
		RenderDistortion();
		vr::Texture_t leftEyeTexture,rightEyeTexture;
		if(view_mode==IMAGE_VIEW_MODE::STEREO){
			leftEyeTexture = {(void*)m_EyeTexture[L], vr::API_OpenGL, vr::ColorSpace_Gamma };
			rightEyeTexture = {(void*)m_EyeTexture[R], vr::API_OpenGL, vr::ColorSpace_Gamma };
		}else{
			leftEyeTexture = {(void*)leftEyeDesc.m_nResolveTextureId, vr::API_OpenGL, vr::ColorSpace_Gamma };
			rightEyeTexture = {(void*)rightEyeDesc.m_nResolveTextureId, vr::API_OpenGL, vr::ColorSpace_Gamma };
		}
	  boost::mutex::scoped_lock lk(mtx);
		vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
    lk.unlock();
	}
	if ( m_bVblank && m_bGlFinishHack )	{
		//$ HACKHACK. From gpuview profiling, it looks like there is a bug where two renders and a present
		// happen right before and after the vsync causing all kinds of jittering issues. This glFinish()
		// appears to clear that up. Temporary fix while I try to get nvidia to investigate this problem.
		// 1/29/2014 mikesart
		glFinish();
	}
	// SwapWindow
  SDL_GL_SwapWindow( m_pWindow );
	// Clear
  // We want to make sure the glFinish waits for the entire present to complete, not just the submission
  // of the command. So, we do a clear here right here so the glFinish will wait fully for the swap.
  glClearColor( 0, 0, 0, 1 );
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	// Flush and wait for swap.
	if ( m_bVblank )	{
		glFlush();
		glFinish();
	}
	// Spew out the controller and pose count whenever they change.
	if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last )	{
		m_iValidPoseCount_Last = m_iValidPoseCount;
		m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
		dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
	}
//	UpdateHMDMatrixPose();//move to thread
}


//-----------------------------------------------------------------------------
// Purpose: Compiles a GL shader program and returns the handle. Returns 0 if
//			the shader couldn't be compiled for some reason.
//-----------------------------------------------------------------------------
GLuint CMainApplication::CompileGLShader( const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader ){
	GLuint unProgramID = glCreateProgram();
	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource( nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader( nSceneVertexShader );
	GLint vShaderCompiled = GL_FALSE;
	glGetShaderiv( nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
	if ( vShaderCompiled != GL_TRUE)	{
		dprintf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		glDeleteProgram( unProgramID );
		glDeleteShader( nSceneVertexShader );
		return 0;
	}
	glAttachShader( unProgramID, nSceneVertexShader);
	glDeleteShader( nSceneVertexShader ); // the program hangs onto this once it's attached
	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource( nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader( nSceneFragmentShader );
	GLint fShaderCompiled = GL_FALSE;
	glGetShaderiv( nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
	if (fShaderCompiled != GL_TRUE)	{
		dprintf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader );
		glDeleteProgram( unProgramID );
		glDeleteShader( nSceneFragmentShader );
		return 0;	
	}
	glAttachShader( unProgramID, nSceneFragmentShader );
	glDeleteShader( nSceneFragmentShader ); // the program hangs onto this once it's attached
	glLinkProgram( unProgramID );
	GLint programSuccess = GL_TRUE;
	glGetProgramiv( unProgramID, GL_LINK_STATUS, &programSuccess);
	if ( programSuccess != GL_TRUE )	{
		dprintf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram( unProgramID );
		return 0;
	}
	glUseProgram( unProgramID );
	glUseProgram( 0 );
	return unProgramID;
}


//-----------------------------------------------------------------------------
// Purpose: Creates all the shaders used by HelloVR SDL
//-----------------------------------------------------------------------------
bool CMainApplication::CreateAllShaders(){
	m_unSceneProgramID = CompileGLShader( 
		"Scene",
		// Vertex Shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVcoordsIn;\n"
		"layout(location = 2) in vec3 v3NormalIn;\n"
		"out vec2 v2UVcoords;\n"
		"void main()\n"
		"{\n"
		"	v2UVcoords = v2UVcoordsIn;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",
		// Fragment Shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"in vec2 v2UVcoords;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture(mytexture, v2UVcoords);\n"
		"}\n"
		);
	m_nSceneMatrixLocation = glGetUniformLocation( m_unSceneProgramID, "matrix" );
	if( m_nSceneMatrixLocation == -1 )	{
		dprintf( "Unable to find matrix uniform in scene shader\n" );
		return false;
	}
	m_unControllerTransformProgramID = CompileGLShader(
		"Controller",
		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3ColorIn;\n"
		"out vec4 v4Color;\n"
		"void main()\n"
		"{\n"
		"	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",
		// fragment shader
		"#version 410\n"
		"in vec4 v4Color;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = v4Color;\n"
		"}\n"
		);
	m_nControllerMatrixLocation = glGetUniformLocation( m_unControllerTransformProgramID, "matrix" );
	if( m_nControllerMatrixLocation == -1 )	{
		dprintf( "Unable to find matrix uniform in controller shader\n" );
		return false;
	}
	m_unRenderModelProgramID = CompileGLShader( 
		"render model",
		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3NormalIn;\n"
		"layout(location = 2) in vec2 v2TexCoordsIn;\n"
		"out vec2 v2TexCoord;\n"
		"void main()\n"
		"{\n"
		"	v2TexCoord = v2TexCoordsIn;\n"
		"	gl_Position = matrix * vec4(position.xyz, 1);\n"
		"}\n",
		//fragment shader
		"#version 410 core\n"
		"uniform sampler2D diffuse;\n"
		"in vec2 v2TexCoord;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture( diffuse, v2TexCoord);\n"
		"}\n"
		);
	m_nRenderModelMatrixLocation = glGetUniformLocation( m_unRenderModelProgramID, "matrix" );
	if( m_nRenderModelMatrixLocation == -1 )	{
		dprintf( "Unable to find matrix uniform in render model shader\n" );
		return false;
	}
	m_unLensProgramID = CompileGLShader(
		"Distortion",
		// vertex shader
		"#version 410 core\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVredIn;\n"
		"layout(location = 2) in vec2 v2UVGreenIn;\n"
		"layout(location = 3) in vec2 v2UVblueIn;\n"
		"noperspective  out vec2 v2UVred;\n"
		"noperspective  out vec2 v2UVgreen;\n"
		"noperspective  out vec2 v2UVblue;\n"
		"void main()\n"
		"{\n"
		"	v2UVred = v2UVredIn;\n"
		"	v2UVgreen = v2UVGreenIn;\n"
		"	v2UVblue = v2UVblueIn;\n"
		"	gl_Position = position;\n"
		"}\n",
		// fragment shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"noperspective  in vec2 v2UVred;\n"
		"noperspective  in vec2 v2UVgreen;\n"
		"noperspective  in vec2 v2UVblue;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"	float fBoundsCheck = ( (dot( vec2( lessThan( v2UVgreen.xy, vec2(0.05, 0.05)) ), vec2(1.0, 1.0))+dot( vec2( greaterThan( v2UVgreen.xy, vec2( 0.95, 0.95)) ), vec2(1.0, 1.0))) );\n"
		"	if( fBoundsCheck > 1.0 )\n"
		"	{ outputColor = vec4( 0, 0, 0, 1.0 ); }\n"
		"	else\n"
		"	{\n"
		"		float red = texture(mytexture, v2UVred).x;\n"
		"		float green = texture(mytexture, v2UVgreen).y;\n"
		"		float blue = texture(mytexture, v2UVblue).z;\n"
		"		outputColor = vec4( red, green, blue, 1.0  ); }\n"
		"}\n"
		);
	return m_unSceneProgramID != 0 
		&& m_unControllerTransformProgramID != 0
		&& m_unRenderModelProgramID != 0
		&& m_unLensProgramID != 0;
}

bool CMainApplication::SetupTexturemaps(){
	std::string strFullPath = ros::package::getPath("vive_image_view") + "/texture.png";

	std::vector<unsigned char> imageRGBA;
	unsigned nImageWidth, nImageHeight;
  unsigned nError = lodepng::decode( imageRGBA, nImageWidth, nImageHeight, strFullPath.c_str() );
	if ( nError != 0 ) return false;

	glGenTextures(1, &m_iTexture );
	glBindTexture( GL_TEXTURE_2D, m_iTexture );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, nImageWidth, nImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, &imageRGBA[0] );
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);
	glBindTexture( GL_TEXTURE_2D, 0 );
//	return ( m_iTexture != 0 );

	GLubyte texture[1080][1200][3] = {255};
	for(int i=0;i<1080;i++){
    for(int j=0;j<1080;j++){
      for(int k=0;k<3;k++){
        texture[i][j][k] = 100;
      }
	  }
	}
	for(int i=L;i<LR;i++){
	  glGenTextures(1, &m_EyeTexture[i] );
	  glBindTexture( GL_TEXTURE_2D, m_EyeTexture[i] );
	  glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, 1080, 1200, 0, GL_RGB, GL_UNSIGNED_BYTE, &texture[0] );
	  glGenerateMipmap(GL_TEXTURE_2D);
	  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	  GLfloat fLargest_tmp;
	  glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest_tmp);
	  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest_tmp);
	  glBindTexture( GL_TEXTURE_2D, 0 );
	}

	return ( m_iTexture != 0 );
}


void CMainApplication::SetupScene(){
	if ( !m_pHMD )return;

	std::vector<float> vertdataarray;

	Matrix4 mat;
	mat.scale(10,10,1);
	mat.translate(-5,-5,0);
//  AddCubeToScene( mat, vertdataarray );
  Vector4 A = mat * Vector4( 0, 0, 0, 1 );
  Vector4 B = mat * Vector4( 1, 0, 0, 1 );
  Vector4 C = mat * Vector4( 1, 1, 0, 1 );
  Vector4 D = mat * Vector4( 0, 1, 0, 1 );
  AddCubeVertex( B.x, B.y, B.z, 0, 1, vertdataarray ); //Back
  AddCubeVertex( A.x, A.y, A.z, 1, 1, vertdataarray );
  AddCubeVertex( D.x, D.y, D.z, 1, 0, vertdataarray );
  AddCubeVertex( D.x, D.y, D.z, 1, 0, vertdataarray );
  AddCubeVertex( C.x, C.y, C.z, 0, 0, vertdataarray );
  AddCubeVertex( B.x, B.y, B.z, 0, 1, vertdataarray );


	m_uiVertcount = vertdataarray.size()/5;
	
	glGenVertexArrays( 1, &m_unSceneVAO );
	glBindVertexArray( m_unSceneVAO );

	glGenBuffers( 1, &m_glSceneVertBuffer );
	glBindBuffer( GL_ARRAY_BUFFER, m_glSceneVertBuffer );
	glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STATIC_DRAW);

	glBindBuffer( GL_ARRAY_BUFFER, m_glSceneVertBuffer );

	GLsizei stride = sizeof(VertexDataScene);
	uintptr_t offset = 0;

	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride , (const void *)offset);

	offset += sizeof(Vector3);
	glEnableVertexAttribArray( 1 );
	glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

	glBindVertexArray( 0 );
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

}

void CMainApplication::AddCubeVertex( float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata ){
	vertdata.push_back( fl0 );
	vertdata.push_back( fl1 );
	vertdata.push_back( fl2 );
	vertdata.push_back( fl3 );
	vertdata.push_back( fl4 );
}



//-----------------------------------------------------------------------------
// Purpose: Draw all of the controllers as X/Y/Z lines
//-----------------------------------------------------------------------------
void CMainApplication::DrawControllers(){
	// don't draw controllers if somebody else has input focus
	if( m_pHMD->IsInputFocusCapturedByAnotherProcess() )return;

	std::vector<float> vertdataarray;

	m_uiControllerVertcount = 0;
	m_iTrackedControllerCount = 0;

	for ( vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice )
	{
		if ( !m_pHMD->IsTrackedDeviceConnected( unTrackedDevice ) )	continue;
		if( m_pHMD->GetTrackedDeviceClass( unTrackedDevice ) != vr::TrackedDeviceClass_Controller )	continue;
		m_iTrackedControllerCount += 1;
		if( !m_rTrackedDevicePose[ unTrackedDevice ].bPoseIsValid )	continue;
		const Matrix4 & mat = m_rmat4DevicePose[unTrackedDevice];
		Vector4 center = mat * Vector4( 0, 0, 0, 1 );
		for ( int i = 0; i < 3; ++i )	{
			Vector3 color( 0, 0, 0 );
			Vector4 point( 0, 0, 0, 1 );
			point[i] += 0.05f;  // offset in X, Y, Z
			color[i] = 1.0;  // R, G, B
			point = mat * point;
			vertdataarray.push_back( center.x );
			vertdataarray.push_back( center.y );
			vertdataarray.push_back( center.z );

			vertdataarray.push_back( color.x );
			vertdataarray.push_back( color.y );
			vertdataarray.push_back( color.z );
		
			vertdataarray.push_back( point.x );
			vertdataarray.push_back( point.y );
			vertdataarray.push_back( point.z );
		
			vertdataarray.push_back( color.x );
			vertdataarray.push_back( color.y );
			vertdataarray.push_back( color.z );
		
			m_uiControllerVertcount += 2;
		}
		Vector4 start = mat * Vector4( 0, 0, -0.02f, 1 );
		Vector4 end = mat * Vector4( 0, 0, -39.f, 1 );
		Vector3 color( .92f, .92f, .71f );
		vertdataarray.push_back( start.x );vertdataarray.push_back( start.y );vertdataarray.push_back( start.z );
		vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );
		vertdataarray.push_back( end.x );vertdataarray.push_back( end.y );vertdataarray.push_back( end.z );
		vertdataarray.push_back( color.x );vertdataarray.push_back( color.y );vertdataarray.push_back( color.z );
		m_uiControllerVertcount += 2;
	}
	// Setup the VAO the first time through.
	if ( m_unControllerVAO == 0 )	{
		glGenVertexArrays( 1, &m_unControllerVAO );
		glBindVertexArray( m_unControllerVAO );
		glGenBuffers( 1, &m_glControllerVertBuffer );
		glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );
		GLuint stride = 2 * 3 * sizeof( float );
		GLuint offset = 0;
		glEnableVertexAttribArray( 0 );
		glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
		offset += sizeof( Vector3 );
		glEnableVertexAttribArray( 1 );
		glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
		glBindVertexArray( 0 );
	}
	glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );
	// set vertex data if we have some
	if( vertdataarray.size() > 0 ){
		//$ TODO: Use glBufferSubData for this...
		glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW );
	}
}

void CMainApplication::SetupCameras(){
	m_mat4ProjectionLeft = GetHMDMatrixProjectionEye( vr::Eye_Left );
	m_mat4ProjectionRight = GetHMDMatrixProjectionEye( vr::Eye_Right );
	m_mat4eyePosLeft = GetHMDMatrixPoseEye( vr::Eye_Left );
	m_mat4eyePosRight = GetHMDMatrixPoseEye( vr::Eye_Right );
}

bool CMainApplication::CreateFrameBuffer( int nWidth, int nHeight, FramebufferDesc &framebufferDesc ){
	glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId );
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);

	glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight );
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,	framebufferDesc.m_nDepthBufferId );

	glGenTextures(1, &framebufferDesc.m_nRenderTextureId );
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId );
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);

	glGenFramebuffers(1, &framebufferDesc.m_nResolveFramebufferId );
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nResolveFramebufferId);

	glGenTextures(1, &framebufferDesc.m_nResolveTextureId );
	glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);

	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE){
		return false;
	}
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
	return true;
}

bool CMainApplication::SetupStereoRenderTargets(){
	if ( !m_pHMD )return false;
	m_pHMD->GetRecommendedRenderTargetSize( &m_nRenderWidth, &m_nRenderHeight );
	CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, leftEyeDesc );
	CreateFrameBuffer( m_nRenderWidth, m_nRenderHeight, rightEyeDesc );
		return true;
}

void CMainApplication::SetupDistortion(){
	if ( !m_pHMD )return;
	GLushort m_iLensGridSegmentCountH = 43;
	GLushort m_iLensGridSegmentCountV = 43;
	float w = (float)( 1.0/float(m_iLensGridSegmentCountH-1));
	float h = (float)( 1.0/float(m_iLensGridSegmentCountV-1));
	float u, v = 0;
	std::vector<VertexDataLens> vVerts(0);
	VertexDataLens vert;
	//left eye distortion verts
	float Xoffset = -1;
	for( int y=0; y<m_iLensGridSegmentCountV; y++ )	{
		for( int x=0; x<m_iLensGridSegmentCountH; x++ )		{
			u = x*w; v = 1-y*h;
			vert.position = Vector2( Xoffset+u, -1+2*y*h );
			vr::DistortionCoordinates_t dc0 = m_pHMD->ComputeDistortion(vr::Eye_Left, u, v);
			vert.texCoordRed = Vector2(dc0.rfRed[0], 1 - dc0.rfRed[1]);
			vert.texCoordGreen =  Vector2(dc0.rfGreen[0], 1 - dc0.rfGreen[1]);
			vert.texCoordBlue = Vector2(dc0.rfBlue[0], 1 - dc0.rfBlue[1]);
			vVerts.push_back( vert );
		}
	}
	//right eye distortion verts
	Xoffset = 0;
	for( int y=0; y<m_iLensGridSegmentCountV; y++ )	{
		for( int x=0; x<m_iLensGridSegmentCountH; x++ )		{
			u = x*w; v = 1-y*h;
			vert.position = Vector2( Xoffset+u, -1+2*y*h );
			vr::DistortionCoordinates_t dc0 = m_pHMD->ComputeDistortion( vr::Eye_Right, u, v );
			vert.texCoordRed = Vector2(dc0.rfRed[0], 1 - dc0.rfRed[1]);
			vert.texCoordGreen = Vector2(dc0.rfGreen[0], 1 - dc0.rfGreen[1]);
			vert.texCoordBlue = Vector2(dc0.rfBlue[0], 1 - dc0.rfBlue[1]);
			vVerts.push_back( vert );
		}
	}
	std::vector<GLushort> vIndices;
	GLushort a,b,c,d;
	GLushort offset = 0;
	for( GLushort y=0; y<m_iLensGridSegmentCountV-1; y++ )	{
		for( GLushort x=0; x<m_iLensGridSegmentCountH-1; x++ )		{
			a = m_iLensGridSegmentCountH*y+x +offset;
			b = m_iLensGridSegmentCountH*y+x+1 +offset;
			c = (y+1)*m_iLensGridSegmentCountH+x+1 +offset;
			d = (y+1)*m_iLensGridSegmentCountH+x +offset;
			vIndices.push_back( a );
			vIndices.push_back( b );
			vIndices.push_back( c );
			vIndices.push_back( a );
			vIndices.push_back( c );
			vIndices.push_back( d );
		}
	}
	offset = (m_iLensGridSegmentCountH)*(m_iLensGridSegmentCountV);
	for( GLushort y=0; y<m_iLensGridSegmentCountV-1; y++ )	{
		for( GLushort x=0; x<m_iLensGridSegmentCountH-1; x++ )		{
			a = m_iLensGridSegmentCountH*y+x +offset;
			b = m_iLensGridSegmentCountH*y+x+1 +offset;
			c = (y+1)*m_iLensGridSegmentCountH+x+1 +offset;
			d = (y+1)*m_iLensGridSegmentCountH+x +offset;
			vIndices.push_back( a );
			vIndices.push_back( b );
			vIndices.push_back( c );
			vIndices.push_back( a );
			vIndices.push_back( c );
			vIndices.push_back( d );
		}
	}
	m_uiIndexSize = vIndices.size();
	glGenVertexArrays( 1, &m_unLensVAO );
	glBindVertexArray( m_unLensVAO );
	glGenBuffers( 1, &m_glIDVertBuffer );
	glBindBuffer( GL_ARRAY_BUFFER, m_glIDVertBuffer );
	glBufferData( GL_ARRAY_BUFFER, vVerts.size()*sizeof(VertexDataLens), &vVerts[0], GL_STATIC_DRAW );
	glGenBuffers( 1, &m_glIDIndexBuffer );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_glIDIndexBuffer );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, vIndices.size()*sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW );
	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataLens), (void *)offsetof( VertexDataLens, position ) );
	glEnableVertexAttribArray( 1 );
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataLens), (void *)offsetof( VertexDataLens, texCoordRed ) );
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataLens), (void *)offsetof( VertexDataLens, texCoordGreen ) );
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataLens), (void *)offsetof( VertexDataLens, texCoordBlue ) );
	glBindVertexArray( 0 );
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void CMainApplication::RenderStereoTargets(){
	glClearColor( 0.15f, 0.15f, 0.18f, 1.0f ); // nice background color, but not black
	glEnable( GL_MULTISAMPLE );
	// Left Eye
  glBindFramebuffer( GL_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId );
  glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
  RenderScene( vr::Eye_Left );
  glBindFramebuffer( GL_FRAMEBUFFER, 0 );
  glDisable( GL_MULTISAMPLE );
  glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.m_nResolveFramebufferId );
  glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, GL_COLOR_BUFFER_BIT,	GL_LINEAR );
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );
  glEnable( GL_MULTISAMPLE );
	// Right Eye
  glBindFramebuffer( GL_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
  glViewport(0, 0, m_nRenderWidth, m_nRenderHeight );
  RenderScene( vr::Eye_Right );
  glBindFramebuffer( GL_FRAMEBUFFER, 0 );
  glDisable( GL_MULTISAMPLE );
  glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId );
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.m_nResolveFramebufferId );
  glBlitFramebuffer( 0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight, GL_COLOR_BUFFER_BIT,	GL_LINEAR  );
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0 );
}

void CMainApplication::RenderScene( vr::Hmd_Eye nEye ){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	if( m_bShowCubes )	{
		glUseProgram( m_unSceneProgramID );
		glUniformMatrix4fv( m_nSceneMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix( nEye ).get() );
		glBindVertexArray( m_unSceneVAO );
		glBindTexture( GL_TEXTURE_2D, m_iTexture );
		glDrawArrays( GL_TRIANGLES, 0, m_uiVertcount );
		glBindVertexArray( 0 );
	}
	bool bIsInputCapturedByAnotherProcess = m_pHMD->IsInputFocusCapturedByAnotherProcess();
	if( !bIsInputCapturedByAnotherProcess )	{
		// draw the controller axis lines
		glUseProgram( m_unControllerTransformProgramID );
		glUniformMatrix4fv( m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix( nEye ).get() );
		glBindVertexArray( m_unControllerVAO );
		glDrawArrays( GL_LINES, 0, m_uiControllerVertcount );
		glBindVertexArray( 0 );
	}
	// ----- Render Model rendering -----
	glUseProgram( m_unRenderModelProgramID );
	for( uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++ )	{
		if( !m_rTrackedDeviceToRenderModel[ unTrackedDevice ] || !m_rbShowTrackedDevice[ unTrackedDevice ] )	continue;
		const vr::TrackedDevicePose_t & pose = m_rTrackedDevicePose[ unTrackedDevice ];
		if( !pose.bPoseIsValid )	continue;
		if( bIsInputCapturedByAnotherProcess && m_pHMD->GetTrackedDeviceClass( unTrackedDevice ) == vr::TrackedDeviceClass_Controller )	continue;
		const Matrix4 & matDeviceToTracking = m_rmat4DevicePose[ unTrackedDevice ];
		Matrix4 matMVP = GetCurrentViewProjectionMatrix( nEye ) * matDeviceToTracking;
		glUniformMatrix4fv( m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get() );
		m_rTrackedDeviceToRenderModel[ unTrackedDevice ]->Draw();
	}
	glUseProgram( 0 );
}

void CMainApplication::RenderDistortion(){
	glDisable(GL_DEPTH_TEST);
	glViewport( 0, 0, m_nWindowWidth, m_nWindowHeight );
	glBindVertexArray( m_unLensVAO );
	glUseProgram( m_unLensProgramID );
	//render left lens (first half of index array )
	if(view_mode==IMAGE_VIEW_MODE::STEREO){
		glBindTexture(GL_TEXTURE_2D, m_EyeTexture[L] );
	}else{
		glBindTexture(GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId );
	}
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	glDrawElements( GL_TRIANGLES, m_uiIndexSize/2, GL_UNSIGNED_SHORT, 0 );
	//render right lens (second half of index array )
	if(view_mode==IMAGE_VIEW_MODE::STEREO){
		glBindTexture(GL_TEXTURE_2D, m_EyeTexture[R] );
	}else{
		glBindTexture(GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId  );
	}
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	glDrawElements( GL_TRIANGLES, m_uiIndexSize/2, GL_UNSIGNED_SHORT, (const void *)(m_uiIndexSize) );
	glBindVertexArray( 0 );
	glUseProgram( 0 );
}

Matrix4 CMainApplication::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye ){
	if ( !m_pHMD )return Matrix4();
	vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip, vr::API_OpenGL);
	return Matrix4(
		mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
		mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1], 
		mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2], 
		mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
	);
}

Matrix4 CMainApplication::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye ){
	if ( !m_pHMD )return Matrix4();
	vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
	Matrix4 matrixObj(
		matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0, 
		matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
		matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
		matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
		);
	return matrixObj.invert();
}

Matrix4 CMainApplication::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye ){
	Matrix4 matMVP;
	if( nEye == vr::Eye_Left )	{
		matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
	}
	else if( nEye == vr::Eye_Right )	{
		matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
	}
	return matMVP;
}

void CMainApplication::UpdateHMDMatrixPose(){
	if ( !m_pHMD )return;

  boost::mutex::scoped_lock lk(mtx);
	vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
  lk.unlock();

	m_iValidPoseCount = 0;
	m_strPoseClasses = "";
	for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )	{
		if ( m_rTrackedDevicePose[nDevice].bPoseIsValid )		{
			m_iValidPoseCount++;
			m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
			if (m_rDevClassChar[nDevice]==0)			{
				switch (m_pHMD->GetTrackedDeviceClass(nDevice))	{
          case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
          case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
          case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
          case vr::TrackedDeviceClass_Other:             m_rDevClassChar[nDevice] = 'O'; break;
          case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
          default:                                       m_rDevClassChar[nDevice] = '?'; break;
				}
			}
			m_strPoseClasses += m_rDevClassChar[nDevice];
		}
	}

  for( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice ) {
    if ( m_rTrackedDevicePose[nDevice].bPoseIsValid ){
      int dev_type = -1;
      if( m_pHMD->GetTrackedDeviceClass(nDevice) == vr::TrackedDeviceClass_HMD){
        dev_type = HMD;
        m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd].invert();
      }
      else if( m_pHMD->GetTrackedDeviceClass(nDevice) == vr::TrackedDeviceClass_Controller){
        if ( m_pHMD->GetControllerRoleForTrackedDeviceIndex(nDevice) == vr::TrackedControllerRole_LeftHand){ dev_type = LC; }
        else if( m_pHMD->GetControllerRoleForTrackedDeviceIndex(nDevice) == vr::TrackedControllerRole_RightHand){ dev_type = RC; }
      }
      if(dev_type != -1){
        tf::Matrix3x3 rot_mat;
        tf::Vector3 pos_vec;
        tf::Quaternion quat_vr_coord,quat_ros_coord;
        for (int i=0; i<3; i++){
          for (int o=0; o<3; o++){//0->3で左端列を下に進むらしい
            rot_mat[i][o] = static_cast<double>(m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking.m[i][o]);
          }
        }
        for (int i=0; i<3; i++){
          pos_vec[i] = static_cast<double>(m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking.m[i][3]);
        }
        tf::Matrix3x3 vr2tf;//(-1,0,0, 0,1,0, 0,0,-1);//初期に向いている方向のオフセット("b"のレーザー照射器に対して後ろ向きがデフォになっている)
        vr2tf.setRPY(0,M_PI,0);
        rot_mat = vr2tf.inverse() * rot_mat;
        pos_vec = vr2tf.inverse() * pos_vec;
        rot_mat.getRotation(quat_vr_coord);
        quat_ros_coord.setX(-quat_vr_coord.z());//VR空間座標とROS空間座標の変換
        quat_ros_coord.setY(-quat_vr_coord.x());
        quat_ros_coord.setZ( quat_vr_coord.y());
        quat_ros_coord.setW( quat_vr_coord.w());
        tf::Vector3 pos_tmp = pos_vec;
        pos_vec.setX(-pos_tmp.z());
        pos_vec.setY(-pos_tmp.x());
        pos_vec.setZ(pos_tmp.y());

        ros_pose[dev_type].pose.orientation.x = quat_ros_coord.x();
        ros_pose[dev_type].pose.orientation.y = quat_ros_coord.y();
        ros_pose[dev_type].pose.orientation.z = quat_ros_coord.z();
        ros_pose[dev_type].pose.orientation.w = quat_ros_coord.w();
        ros_pose[dev_type].pose.position.x  = pos_vec.x();
        ros_pose[dev_type].pose.position.y  = pos_vec.y();
        ros_pose[dev_type].pose.position.z  = pos_vec.z();
      }
    }
  }

}


//-----------------------------------------------------------------------------
// Purpose: Finds a render model we've already loaded or loads a new one
//-----------------------------------------------------------------------------
CGLRenderModel *CMainApplication::FindOrLoadRenderModel( const char *pchRenderModelName ){
	CGLRenderModel *pRenderModel = NULL;
	for( std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++ )	{
		if( !stricmp( (*i)->GetName().c_str(), pchRenderModelName ) )		{
			pRenderModel = *i;
			break;
		}
	}
	// load the model if we didn't find one
	if( !pRenderModel )	{
		vr::RenderModel_t *pModel;
		vr::EVRRenderModelError error;
		while ( 1 )		{
			error = vr::VRRenderModels()->LoadRenderModel_Async( pchRenderModelName, &pModel );
			if ( error != vr::VRRenderModelError_Loading )
				break;
			ThreadSleep( 1 );
		}
		if ( error != vr::VRRenderModelError_None )		{
			dprintf( "Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum( error ) );
			return NULL; // move on to the next tracked device
		}
		vr::RenderModel_TextureMap_t *pTexture;
		while ( 1 )		{
			error = vr::VRRenderModels()->LoadTexture_Async( pModel->diffuseTextureId, &pTexture );
			if ( error != vr::VRRenderModelError_Loading )	break;
			ThreadSleep( 1 );
		}
		if ( error != vr::VRRenderModelError_None )		{
			dprintf( "Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName );
			vr::VRRenderModels()->FreeRenderModel( pModel );
			return NULL; // move on to the next tracked device
		}
		pRenderModel = new CGLRenderModel( pchRenderModelName );
		if ( !pRenderModel->BInit( *pModel, *pTexture ) )		{
			dprintf( "Unable to create GL model from render model %s\n", pchRenderModelName );
			delete pRenderModel;
			pRenderModel = NULL;
		}
		else{
			m_vecRenderModels.push_back( pRenderModel );
		}
		vr::VRRenderModels()->FreeRenderModel( pModel );
		vr::VRRenderModels()->FreeTexture( pTexture );
	}
	return pRenderModel;
}

//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL a Render Model for a single tracked device
//-----------------------------------------------------------------------------
void CMainApplication::SetupRenderModelForTrackedDevice( vr::TrackedDeviceIndex_t unTrackedDeviceIndex ){
	if( unTrackedDeviceIndex >= vr::k_unMaxTrackedDeviceCount )	return;
	// try to find a model we've already set up
	std::string sRenderModelName = GetTrackedDeviceString( m_pHMD, unTrackedDeviceIndex, vr::Prop_RenderModelName_String );
	CGLRenderModel *pRenderModel = FindOrLoadRenderModel( sRenderModelName.c_str() );
	if( !pRenderModel )	{
		std::string sTrackingSystemName = GetTrackedDeviceString( m_pHMD, unTrackedDeviceIndex, vr::Prop_TrackingSystemName_String );
		dprintf( "Unable to load render model for tracked device %d (%s.%s)", unTrackedDeviceIndex, sTrackingSystemName.c_str(), sRenderModelName.c_str() );
	}
	else{
		m_rTrackedDeviceToRenderModel[ unTrackedDeviceIndex ] = pRenderModel;
		m_rbShowTrackedDevice[ unTrackedDeviceIndex ] = true;
	}
}

//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
void CMainApplication::SetupRenderModels(){
	memset( m_rTrackedDeviceToRenderModel, 0, sizeof( m_rTrackedDeviceToRenderModel ) );
	if( !m_pHMD )	return;
	for( uint32_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++ ){
		if( !m_pHMD->IsTrackedDeviceConnected( unTrackedDevice ) )continue;
		SetupRenderModelForTrackedDevice( unTrackedDevice );
	}
}

//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose ){
	Matrix4 matrixObj(
		matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
		matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
		matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
		matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
		);
	return matrixObj;
}

//-----------------------------------------------------------------------------
// Purpose: Create/destroy GL Render Models
//-----------------------------------------------------------------------------
CGLRenderModel::CGLRenderModel( const std::string & sRenderModelName ) : m_sModelName( sRenderModelName ){
	m_glIndexBuffer = 0;
	m_glVertArray = 0;
	m_glVertBuffer = 0;
	m_glTexture = 0;
}

CGLRenderModel::~CGLRenderModel(){
	Cleanup();
}

//-----------------------------------------------------------------------------
// Purpose: Allocates and populates the GL resources for a render model
//-----------------------------------------------------------------------------
bool CGLRenderModel::BInit( const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture ){
	// create and bind a VAO to hold state for this model
	glGenVertexArrays( 1, &m_glVertArray );
	glBindVertexArray( m_glVertArray );
	// Populate a vertex buffer
	glGenBuffers( 1, &m_glVertBuffer );
	glBindBuffer( GL_ARRAY_BUFFER, m_glVertBuffer );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t ) * vrModel.unVertexCount, vrModel.rVertexData, GL_STATIC_DRAW );
	// Identify the components in the vertex buffer
	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vPosition ) );
	glEnableVertexAttribArray( 1 );
	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vNormal ) );
	glEnableVertexAttribArray( 2 );
	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, rfTextureCoord ) );
	// Create and populate the index buffer
	glGenBuffers( 1, &m_glIndexBuffer );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_glIndexBuffer );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( uint16_t ) * vrModel.unTriangleCount * 3, vrModel.rIndexData, GL_STATIC_DRAW );
	glBindVertexArray( 0 );
	// create and populate the texture
	glGenTextures(1, &m_glTexture );
	glBindTexture( GL_TEXTURE_2D, m_glTexture );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth, vrDiffuseTexture.unHeight,	0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData );
	// If this renders black ask McJohn what's wrong.
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
	GLfloat fLargest;
	glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest );
	glBindTexture( GL_TEXTURE_2D, 0 );
	m_unVertexCount = vrModel.unTriangleCount * 3;
	return true;
}

//-----------------------------------------------------------------------------
// Purpose: Frees the GL resources for a render model
//-----------------------------------------------------------------------------
void CGLRenderModel::Cleanup(){
	if( m_glVertBuffer )	{
		glDeleteBuffers(1, &m_glIndexBuffer);
		glDeleteVertexArrays( 1, &m_glVertArray );
		glDeleteBuffers(1, &m_glVertBuffer);
		m_glIndexBuffer = 0;
		m_glVertArray = 0;
		m_glVertBuffer = 0;
	}
}

//-----------------------------------------------------------------------------
// Purpose: Draws the render model
//-----------------------------------------------------------------------------
void CGLRenderModel::Draw(){
	glBindVertexArray( m_glVertArray );
	glActiveTexture( GL_TEXTURE0 );
	glBindTexture( GL_TEXTURE_2D, m_glTexture );
	glDrawElements( GL_TRIANGLES, m_unVertexCount, GL_UNSIGNED_SHORT, 0 );
	glBindVertexArray( 0 );
}

std::string txt_ros;

bool CMainApplication::UpdateTexturemaps(){
  if(view_mode==IMAGE_VIEW_MODE::STEREO){
    //calc eye to HMD panel distance
  //    const double hmd_eye2panel_z[XY] = { (double)hmd_panel_img[i].cols/2/tan(hmd_fov/2), (double)hmd_panel_img[i].rows/2/tan(hmd_fov/2) };
    const double hmd_eye2panel_z[XY] = { (double)hmd_panel_img[L].rows/2/tan(hmd_fov/2), (double)hmd_panel_img[L].rows/2/tan(hmd_fov/2) };//[pixel]パネル距離水平垂直で違うのはおかしいので垂直画角を信じる
    const double cam_pic_size[LR][XY] = { { (double)ros_image_stereo[L].cols, (double)ros_image_stereo[L].rows }, { (double)ros_image_stereo[R].cols, (double)ros_image_stereo[R].rows } };
    double cam_fov[LR][XY];
    int cam_pic_size_on_hmd[LR][XY];
    cv::Mat hmd_panel_roi[LR];
    const cv::Point parallax_adjust[LR] = {cv::Point(-50,0),cv::Point(+50,0)};//視差調整用
  //    const cv::Point parallax_adjust[LR] = {cv::Point(+50,0),cv::Point(-50,0)};//視差調整用
    for(int i=L;i<LR;i++){
      if(ros_image_isNew[i]){
        for(int j=X;j<XY;j++){
          cam_fov[i][j] = 2*atan( cam_pic_size[i][j]/2 / cam_f[i][j] );
          cam_pic_size_on_hmd[i][j] = (int)( hmd_eye2panel_z[X] * 2*tan(cam_fov[i][j]/2) );
        }
        cv::resize(ros_image_stereo[i], ros_image_stereo_resized[i], cv::Size(cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y]));
        cv::putText(ros_image_stereo_resized[i], txt_ros, cv::Point(0,500), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,250,250), 2, CV_AA);
        cv::flip(ros_image_stereo_resized[i],ros_image_stereo_resized[i],0);
        double angle = cp_ros[1] * 300;
        float scale = 1.0;
        // 画像の中心を求める
        cv::Point2f center(ros_image_stereo_resized[i].cols / 2.0, ros_image_stereo_resized[i].rows / 2.0 - 500);
        // 回転
        cv::Mat matrix = cv::getRotationMatrix2D( center, angle, scale );
        //画像を回転させる
        cv::warpAffine(ros_image_stereo_resized[i], ros_image_stereo_resized[i], matrix, ros_image_stereo_resized[i].size());

        cv::Rect hmd_panel_area_rect( ros_image_stereo_resized[i].cols/2-hmd_panel_img[i].cols/2, ros_image_stereo_resized[i].rows/2-hmd_panel_img[i].rows/2, hmd_panel_img[i].cols, hmd_panel_img[i].rows);
        hmd_panel_area_rect += parallax_adjust[i];
        cv::Rect ros_image_stereo_resized_rect( 0, 0, ros_image_stereo_resized[i].cols, ros_image_stereo_resized[i].rows);
        cv::Point ros_image_stereo_resized_center(ros_image_stereo_resized[i].cols/2, ros_image_stereo_resized[i].rows/2);
        cv::Rect cropped_rect;
        if( !hmd_panel_area_rect.contains( cv::Point(ros_image_stereo_resized_rect.x, ros_image_stereo_resized_rect.y) )
            || !hmd_panel_area_rect.contains( cv::Point(ros_image_stereo_resized_rect.x+ros_image_stereo_resized_rect.width,ros_image_stereo_resized_rect.y+ros_image_stereo_resized_rect.height) ) ){
          ROS_WARN_THROTTLE(3.0,"Resized ROS image[%d] (%dx%d (%+d,%+d)) exceed HMD eye texture (%dx%d) -> Cropping",i,cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y],parallax_adjust[i].x,parallax_adjust[i].y,hmd_panel_size[X],hmd_panel_size[Y]);
          cropped_rect = ros_image_stereo_resized_rect & hmd_panel_area_rect;
          ros_image_stereo_resized[i] = ros_image_stereo_resized[i](cropped_rect);
        }
        cv::Rect hmd_panel_draw_rect( cropped_rect.x-hmd_panel_area_rect.x, cropped_rect.y-hmd_panel_area_rect.y, ros_image_stereo_resized[i].cols, ros_image_stereo_resized[i].rows);
        ros_image_stereo_resized[i].copyTo(hmd_panel_img[i](hmd_panel_draw_rect));

  //        std::string strFullPath = ros::package::getPath("vive_image_view") + "/texture.png";
  //
  //        std::vector<unsigned char> imageRGBA;
  //        unsigned nImageWidth, nImageHeight;
  //        unsigned nError = lodepng::decode( imageRGBA, nImageWidth, nImageHeight, strFullPath.c_str() );
  //        if ( nError != 0 ) return false;

  //        cv::putText(hmd_panel_img[i], txt_ros, cv::Point(0,500), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,0,200), 2, CV_AA);

        int cur_tex_w,cur_tex_h;
        glBindTexture( GL_TEXTURE_2D, m_EyeTexture[i] );
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_WIDTH , &cur_tex_w );
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_HEIGHT , &cur_tex_h );
        glTexSubImage2D( GL_TEXTURE_2D, 0, cur_tex_w/2 - hmd_panel_img[i].cols/2, cur_tex_h/2 - hmd_panel_img[i].rows/2, hmd_panel_img[i].cols, hmd_panel_img[i].rows,GL_RGB, GL_UNSIGNED_BYTE, hmd_panel_img[i].data );
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
      }
    }
  }else{
    glBindTexture( GL_TEXTURE_2D, m_iTexture );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, ros_image_monoeye.cols, ros_image_monoeye.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, ros_image_monoeye.data );
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
    GLfloat fLargest;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);
    glBindTexture( GL_TEXTURE_2D, 0 );
  }
	return ( m_iTexture != 0 );
}

CMainApplication *pMainApplication;

void convertImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& out){
  try {
    out = cv_bridge::toCvCopy(msg,"rgb8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
  }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){
  convertImage(msg, pMainApplication->ros_image_monoeye);
  t_cb_now = ros::WallTime::now();
  ROS_INFO_STREAM_THROTTLE(3.0,"imageCb() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_now - t_cb_old).toSec()<<" fps");
  t_cb_old = t_cb_now;
  ros_image_isNew_mono = true;
}

void imageCb_L(const sensor_msgs::ImageConstPtr& msg){
  convertImage(msg, pMainApplication->ros_image_stereo[L]);
  t_cb_l_now = ros::WallTime::now();
  ROS_INFO_STREAM_THROTTLE(3.0,"imageCb_L() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_l_now - t_cb_l_old).toSec()<<" fps");
  t_cb_l_old = t_cb_l_now;
  ros_image_isNew[L] = true;
}

void imageCb_R(const sensor_msgs::ImageConstPtr& msg){
  convertImage(msg, pMainApplication->ros_image_stereo[R]);
  t_cb_r_now = ros::WallTime::now();
  ROS_INFO_STREAM_THROTTLE(3.0,"imageCb_R() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_r_now - t_cb_r_old).toSec()<<" fps");
  t_cb_r_old = t_cb_r_now;
  ros_image_isNew[R] = true;
}

void infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg){ cam_f[L][0] = msg->K[0]; cam_f[L][1] = msg->K[4];}
void infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg){ cam_f[R][0] = msg->K[0]; cam_f[R][1] = msg->K[4];}

#include <sstream>
void cpCb(const geometry_msgs::PointStampedConstPtr& msg){
  cp_ros[0] = msg->point.x;
  cp_ros[1] = msg->point.y;
  cp_ros[2] = msg->point.z;
  t_cb_cp_now = ros::WallTime::now();

  std::ostringstream ostr;
  ostr<<"cpCb() subscribed "<<msg->point.x<<" "<<msg->point.y<<" "<<msg->point.z<<" "<<" @ "<<1.0/(t_cb_cp_now - t_cb_cp_old).toSec()<<" fps";
  txt_ros = ostr.str();
  ROS_INFO_STREAM_THROTTLE(3.0,"cpCb() subscribed "<<msg->point.x<<" "<<msg->point.y<<" "<<msg->point.z<<" "<<" @ "<<1.0/(t_cb_cp_now - t_cb_cp_old).toSec()<<" fps");
  t_cb_cp_old = t_cb_cp_now;
}

void publishDevicePosesThread(int* publish_rate){
  ros::NodeHandle node;
  ros::Publisher ros_pose_pub[HMD_LC_RC];
  ros_pose_pub[HMD] = node.advertise<geometry_msgs::PoseStamped>("/vive_head_pose", 1);
  ros_pose_pub[LC] = node.advertise<geometry_msgs::PoseStamped>("/vive_lcon_pose", 1);
  ros_pose_pub[RC] = node.advertise<geometry_msgs::PoseStamped>("/vive_rcon_pose", 1);
  ros::WallRate loop_rate(*publish_rate);
  while (ros::ok()){
    if(VRCompositor_Ready){ ROS_INFO("vr::VRCompositor() ready. Starting publishDevicePosesThread()"); break; }
    else{ ROS_INFO_THROTTLE(1.0,"publishDevicePosesThread() is waiting for vr::VRCompositor() initialization"); }
  }
  while (ros::ok()){
    pMainApplication->UpdateHMDMatrixPose();
    for(int i=0;i<HMD_LC_RC;i++){
      pMainApplication->ros_pose[i].header.frame_id = "world_vive";
      pMainApplication->ros_pose[i].header.stamp = ros::Time::now();
      ros_pose_pub[i].publish(pMainApplication->ros_pose[i]);
    }
    loop_rate.sleep();
//    ros::spinOnce();
    t_th_now = ros::WallTime::now();
    ROS_INFO_STREAM_THROTTLE(3.0,"UpdateHMDMatrixPose() @ "<<1.0/(t_th_now - t_th_old).toSec()<<" fps");
    t_th_old = t_th_now;
  }
}



int main(int argc, char *argv[]){
  ros::init(argc, argv, "vive_image_view", ros::init_options::AnonymousName);
  if (ros::names::remap("image") == "image") {
    ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun image_view image_view image:=<image topic> [transport]");
  }
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  std::string topic_L = nh.resolveName("image_left");
  std::string topic_R = nh.resolveName("image_right");
  std::string topic_i_L = nh.resolveName("camera_info_left");
  std::string topic_i_R = nh.resolveName("camera_info_right");
  local_nh.param("mode", mode, std::string("normal"));
  std::cout<<"topic = "<<topic<<std::endl;
  std::cout<<"topic_L = "<<topic_L<<std::endl;
  std::cout<<"topic_R = "<<topic_R<<std::endl;
  std::cout<<"topic_i_L = "<<topic_i_L<<std::endl;
  std::cout<<"topic_i_R = "<<topic_i_R<<std::endl;
  std::cout<<"mode = "<<mode<<std::endl;

  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  ROS_INFO_STREAM("Using transport \"" << transport << "\"");
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
  image_transport::Subscriber sub,sub_L,sub_R;
  ros::Subscriber sub_i_L,sub_i_R,sub_cp;
  if(mode=="stereo"){
    view_mode = IMAGE_VIEW_MODE::STEREO;
  }else{
    view_mode = IMAGE_VIEW_MODE::NORMAL;
  }
  if(view_mode==IMAGE_VIEW_MODE::STEREO){
    sub_L = it.subscribe(topic_L, 1, imageCb_L);
    sub_R = it.subscribe(topic_R, 1, imageCb_R);
    sub_i_L = local_nh.subscribe(topic_i_L, 1, infoCb_L);
    sub_i_R = local_nh.subscribe(topic_i_R, 1, infoCb_R);
  }else{
    sub = it.subscribe(topic, 1, imageCb, hints);
  }
  sub_cp = local_nh.subscribe("/act_capture_point", 1, cpCb);
  pMainApplication = new CMainApplication( argc, argv );
  int rate_b = 90;
  boost::thread thread_rosposepub(publishDevicePosesThread, &rate_b);

	if (!pMainApplication->BInit())	{
		pMainApplication->Shutdown();
		return 1;
	}
	VRCompositor_Ready = true;
	//RunMainLoop
	bool bQuit = false;
	SDL_StartTextInput();
	SDL_ShowCursor( SDL_DISABLE );
	t_gl_old = t_ros_old = t_cb_old = t_cb_l_old = t_cb_r_old = t_th_old = ros::WallTime::now();
	ros::WallRate loop_rate(90);//HTC_Vive's MAX fps
	while ( !bQuit && ros::ok())	{

		if(ros_image_isNew_mono || ros_image_isNew[L] || ros_image_isNew[R]){
			pMainApplication->UpdateTexturemaps();
			t_ros_now = ros::WallTime::now();
			ROS_INFO_STREAM_THROTTLE(3.0,"UpdateTexturemaps() @ "<<1.0/(t_ros_now - t_ros_old).toSec()<<" fps");
			t_ros_old = t_ros_now;
      if(ros_image_isNew_mono)ros_image_isNew_mono = false;
      if(ros_image_isNew[L])ros_image_isNew[L] = false;
      if(ros_image_isNew[R])ros_image_isNew[R] = false;
		}
		bQuit = pMainApplication->HandleInput();
		pMainApplication->RenderFrame();

		t_gl_now = ros::WallTime::now();
		ROS_INFO_STREAM_THROTTLE(3.0,"RenderFrame() @ "<<1.0/(t_gl_now - t_gl_old).toSec()<<" fps");
		t_gl_old = t_gl_now;

		ros::spinOnce();
		loop_rate.sleep();
	}
	SDL_StopTextInput();
	thread_rosposepub.join();
	pMainApplication->Shutdown();
	return 0;
}
