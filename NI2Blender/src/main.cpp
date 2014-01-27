// Headers for OpenNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
// Header for NITE
#include <XnVNite.h>

#include <iostream>
#include <GL/glut.h> // For GUI
// Local header
#include "MyMethods.h"
#include "PracticalSocket.h"  // For Socket and SocketException
#include "MyTimer.h"
//-----------------------------------------------------------------------------
// Error Handling
//-----------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
		if (rc != XN_STATUS_OK)											\
		{																\
			printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
			return rc;													\
		}

//-----------------------------------------------------------------------------
// Namespaces
//-----------------------------------------------------------------------------
using namespace xn;
using namespace std;

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------
// OpenNI objects
Context g_Context;
DepthGenerator g_DepthGenerator;
GestureGenerator g_GestureGenerator;
HandsGenerator g_HandsGenerator;
UserGenerator g_UserGenerator;
XnMapOutputMode _outputModeDepth;

// Auxiliary vars for GUI
ImageGenerator g_ImageGenerator;
XnMapOutputMode _outputModeImage;
ImageMetaData g_imageMD;
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480
#define POSITION_X 1250 // or 1600 if you're using a single monitor
#define POSITION_Y 410
#define USE_GUI_GLUT 1

//NITE objects
XnVSessionManager* _sessionManager;
XnVBroadcaster* _broadcaster;
XnVWaveDetector* _waveDetector;
XnVPushDetector* _pushDetector;
XnVCircleDetector* _circleDetector;
XnVSwipeDetector* _swipeDetector;
XnVSteadyDetector* _steadyDetector;

// For debugging purposes
XnBool _printGesture = false;
XnBool _printCircle = false;
XnBool _printSessionStatus = false;
XnBool _printUserTracking = true;
XnBool _printHandsTracking = false;

// Toggle on/off features
XnBool _featureGesture = true;
XnBool _featureCircle = true;
const XnBool _featureUserTracking = true; // If it's not true, nothing works
XnBool _featureHandsTracking = true;

// Toggle extra features
XnBool _mirror = true;
XnBool _useSockets = true;

// Session control
XnBool _sessionInitialized = false;
XnBool _inSession = false;
XnBool _sessionRegistered = false;

// User tracking vars
XnBool _needPose = false;
XnChar _strPose[20] = "";

// Set the frames per second
//XnFPSData _xnFPS;

// Setup the status
XnStatus nRetVal = XN_STATUS_OK;

// XnVHandles
XnVHandle hSessionManager;

// Socket object
TCPSocket tcp_sock;

// Resolution of output map
const int res_x = XN_VGA_X_RES;
const int res_y = XN_VGA_Y_RES;
const int res_z = 4000; // == 4m

// User id control
int user_id = -1;

// Id of data sent
int data_id = 1;

// Stores the last gesture recognized
string last_gesture;

//Time to control steady hand
Timer l_timer = Timer();
Timer r_timer = Timer();

// Variable to prevent repeated data (provided from hands) be sent
XnPoint3D l_last_point3d;
XnPoint3D r_last_point3d;
bool l_hand_out_fov = false;
bool r_hand_out_fov = false;

// Percent to filter data (coordinates from hands)
const float percent = 1.5;
float valid_step_x = (res_x * percent) / 100;
float valid_step_y = (res_y * percent) / 100;
float valid_step_z = (res_z * percent) / 100;

// Definitions
#define GESTURE_INIT_SESSION "Wave"
#define KINECT_SMOOTHING_HANDS 0.8
#define KINECT_SMOOTHING_SKELETON 0.8
#define SECONDS_STEADY_HAND 1.5

//-----------------------------------------------------------------------------
// CALLBACKS
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Session Event Handlers
//-----------------------------------------------------------------------------

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& pFocus, void* UserCxt) {
	if (_printSessionStatus)
		printf("\nSession Started");
	if (!_inSession) {
		if (_useSockets) {
			char header[16] = "session_started";
			float coordinates[3] = { 0.0, 0.0, 0.0 };
			char gesture[5] = "none";
			formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
					0.0, 0.0, 0.0);
		}
		addListeners();
	}
	_inSession = true;
}

void XN_CALLBACK_TYPE SessionEnd(void* UserCxt) {
	if (_printSessionStatus)
		printf("\nSession Ended");
	if (_inSession) {
		if (_useSockets) {
			char header[14] = "session_ended";
			float coordinates[3] = { 0.0, 0.0, 0.0 };
			char gesture[5] = "none";
			formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
					0.0, 0.0, 0.0);
		}
		removeListeners();
	}
	_inSession = false;
}

//-----------------------------------------------------------------------------
// Circle Events
//-----------------------------------------------------------------------------

void XN_CALLBACK_TYPE CircleCB(XnFloat fTimes, XnBool bConfident,
		const XnVCircle* pCircle, void* UserCxt) {
	if (_printCircle) {
		float fAngle = fmod((double) fTimes, 1.0) * 2 * XnVMathCommon::PI;
		printf("\nCircle - Angle:%.2f, Confident:%i, Radius:%.2f", fAngle,
				(int) bConfident, pCircle->fRadius);
	}
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[7] = "circle";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				pCircle->fRadius, (float) bConfident, 0.0);
	}
	_circleDetector->Reset(); // Used to force recognize only one circle gesture
}

void XN_CALLBACK_TYPE NoCircleCB(XnFloat fLastValue,
		XnVCircleDetector::XnVNoCircleReason reason, void* UserCxt) {
	if (_printCircle)
		printf("\nNo Circle - Last Value:%.2f, No Circle Reason:%d",
				fLastValue, reason);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[10] = "no_circle";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fLastValue, (int) reason, 0.0);
	}
}

//-----------------------------------------------------------------------------
// Gesture Events
//-----------------------------------------------------------------------------

void XN_CALLBACK_TYPE SwipeUp(XnFloat fVelocity, XnFloat fAngle, void* UserCxt) {
	if (_printGesture)
		printf("\nSwipe Up - Velocity:%.2f, Angle:%.2f", fVelocity, fAngle);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[9] = "swipe_up";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, fAngle, 0.0);
	}
}

void XN_CALLBACK_TYPE SwipeDown(XnFloat fVelocity, XnFloat fAngle,
		void* UserCxt) {
	if (_printGesture)
		printf("\nSwipe Down - Velocity:%.2f, Angle:%.2f", fVelocity, fAngle);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[11] = "swipe_down";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, fAngle, 0.0);
	}
}

void XN_CALLBACK_TYPE SwipeLeft(XnFloat fVelocity, XnFloat fAngle,
		void* UserCxt) {
	if (_printGesture)
		printf("\nSwipe Left - Velocity:%.2f, Angle:%.2f", fVelocity, fAngle);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[11] = "swipe_left";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, fAngle, 0.0);
	}
}

void XN_CALLBACK_TYPE SwipeRight(XnFloat fVelocity, XnFloat fAngle,
		void* UserCxt) {
	if (_printGesture)
		printf("\nSwipe Right - Velocity:%.2f, Angle:%.2f", fVelocity, fAngle);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[12] = "swipe_right";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, fAngle, 0.0);
	}
}

void XN_CALLBACK_TYPE OnWave(void* UserCxt) {
	if (_printGesture)
		printf("\nWave Occured");
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[8] = "on_wave";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture, 0.0,
				0.0, 0.0);
	}
	//_sessionManager->EndSession();// TODO: Check if turn off the session here is a good idea
}

void XN_CALLBACK_TYPE OnPush(XnFloat fVelocity, XnFloat fAngle, void* UserCxt) {
	if (_printGesture)
		printf("\nPush Occured - Velocity:%.2f, Angle:%.2f", fVelocity, fAngle);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[8] = "on_push";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, fAngle, 0.0);
	}
}

void XN_CALLBACK_TYPE StabilizedPush(XnFloat fVelocity, void* UserCxt) {
	if (_printGesture)
		printf("\nPush Stabilized - Velocity:%.2f", fVelocity);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[16] = "stabilized_push";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
				fVelocity, 0.0, 0.0);
	}
}

void XN_CALLBACK_TYPE OnSteady(XnUInt32 nId, XnFloat fVelocity, void* UserCxt) {
	if (_printGesture)
		printf("\nSteady Occured - Id:%d, Velocity:%.2f", nId, fVelocity);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[10] = "on_steady";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture, 0.0,
				0.0, 0.0);
	}
}

void XN_CALLBACK_TYPE NotSteady(XnUInt32 nId, XnFloat fVelocity, void* UserCxt) {
	if (_printGesture)
		printf("\nNot Steady Occured - Id:%d, Velocity:%.2f", nId, fVelocity);
	if (_useSockets) {
		char header[8] = "gesture";
		float coordinates[3] = { 0.0, 0.0, 0.0 };
		char gesture[11] = "not_steady";
		formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture, 0.0,
				0.0, 0.0);
	}
}

//-----------------------------------------------------------------------------
// User Tracking/Skeleton Events
//-----------------------------------------------------------------------------

void XN_CALLBACK_TYPE NewUser(UserGenerator& generator, XnUserID nId,
		void* pCookie) {
	if (_printUserTracking)
		printf("\nNew User: %d", nId);
	if (user_id == -1) {
		if (_needPose) {
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(_strPose,
					nId);
		} else {
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
		}
	}
}

void XN_CALLBACK_TYPE LostUser(UserGenerator& generator, XnUserID nId,
		void* pCookie) {
	if (_printUserTracking)
		printf("\nLost user: %d", nId);
	if ((int) nId == user_id) {
		if (_printUserTracking)
			printf("\nThe calibrated user (%d) was lost", nId);
		if (_useSockets) {
			char header[21] = "calibrated_user_lost";
			float coordinates[3] = { 0.0, 0.0, 0.0 };
			char gesture[5] = "none";
			formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
					0.0, 0.0, 0.0);
		}
		user_id = -1;
		if (_inSession)// Ends the session is still open
			_sessionManager->EndSession();
		if (_sessionRegistered)// Unregister session manager if registered
			unregisterSessionManager(hSessionManager);
	}
}

void XN_CALLBACK_TYPE UserExit(UserGenerator& generator, XnUserID nId,
		void* pCookie) {
	if (_printUserTracking)
		printf("\nThe user %d exited from field of view", nId);
	if ((int) nId == user_id) {
		if (_printUserTracking)
			printf("\nThe calibrated user (%d) exited from field of view", nId);
		if (_useSockets) {
			char header[21] = "calibrated_user_exit";
			float coordinates[3] = { 0.0, 0.0, 0.0 };
			char gesture[5] = "none";
			formatData(header, user_id, -1, -1, -1, coordinates, 0.0, gesture,
					0.0, 0.0, 0.0);
		}
		user_id = -1;
		if (_inSession)// Ends the session is still open
			_sessionManager->EndSession();
		if (_sessionRegistered)// Unregister session manager if registered
			unregisterSessionManager(hSessionManager);
	}
}

void XN_CALLBACK_TYPE UserReEnter(UserGenerator& generator, XnUserID nId,
		void* pCookie) {
	if (_printUserTracking)
		printf("\nUser %d re-entered the scene after exiting", nId);
	if (user_id == -1) {
		if (_needPose) {
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(_strPose,
					nId);
		} else {
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
		}
	}
}

void XN_CALLBACK_TYPE UserPoseDetected(PoseDetectionCapability& capability,
		const XnChar* strPose, XnUserID nId, void* pCookie) {
	if (_printUserTracking)
		printf("\nPose %s detected for user: %d", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
}

void XN_CALLBACK_TYPE UserCalibrationStart(SkeletonCapability& capability,
		XnUserID nId, void* pCookie) {
	if (_printUserTracking)
		printf("\nCalibration started for user: %d", nId);
}
// Save user id when it is -1, create and initialize session manager
void XN_CALLBACK_TYPE UserCalibrationComplete(SkeletonCapability& capability,
		XnUserID nId, XnCalibrationStatus eStatus, void* pCookie) {
	if (eStatus == XN_CALIBRATION_STATUS_OK) {
		if (_printUserTracking)
			printf("\nCalibration complete, start tracking user: %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		if (user_id == -1) {
			user_id = nId;
			if (_useSockets) {
				char header[20] = "new_user_calibrated";
				float coordinates[3] = { 0.0, 0.0, 0.0 };
				char gesture[5] = "none";
				formatData(header, user_id, -1, -1, -1, coordinates, 0.0,
						gesture, 0.0, 0.0, 0.0);
			}
		}
		if (!_sessionInitialized)// Initializes session manager if was not initialized
			initSessionManager();
		if (!_sessionRegistered)// Add callbacks for session manager
			hSessionManager = registerSessionManager();
	} else {
		if (_printUserTracking)
			printf("\nCalibration failed for user: %d", nId);
		if (eStatus == XN_CALIBRATION_STATUS_MANUAL_ABORT) {
			printf("\nManual abort occurred, stop attempting to calibrate!");
			return;
		}
		if (_needPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(_strPose,
					nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
	}
}

//-----------------------------------------------------------------------------
// Methods
//-----------------------------------------------------------------------------

// Client socket initialization and configuration
void initSocket(string servAddress, unsigned short servPort) {
	tcp_sock.connect(servAddress, servPort);
}

void addListeners() {
	if (_featureGesture) {
		_broadcaster->AddListener(_waveDetector);
		_broadcaster->AddListener(_pushDetector);
		_broadcaster->AddListener(_swipeDetector);
		_broadcaster->AddListener(_steadyDetector);
	}

	if (_featureCircle)
		_broadcaster->AddListener(_circleDetector);

	_sessionManager->AddListener(_broadcaster);
}

void removeListeners() {
	if (_featureGesture) {
		_broadcaster->RemoveListener(_waveDetector);
		_broadcaster->RemoveListener(_pushDetector);
		_broadcaster->RemoveListener(_swipeDetector);
		_broadcaster->RemoveListener(_steadyDetector);
	}

	if (_featureCircle)
		_broadcaster->RemoveListener(_circleDetector);

	_sessionManager->RemoveListener(_broadcaster);
}

// Create and initialize session manager
void initSessionManager() {
	// Create and initialize point tracker
	_sessionManager = new XnVSessionManager();
	nRetVal = _sessionManager->Initialize(&g_Context, GESTURE_INIT_SESSION,
			"RaiseHand", &g_HandsGenerator, &g_GestureGenerator,
			&g_GestureGenerator); // I change here...putting generators on initialize session
	if (nRetVal != XN_STATUS_OK) {
		printf("Couldn't initialize the Session Manager: %s\n",
				xnGetStatusString(nRetVal));
		cleanUpExit();
	}
	_sessionManager->SetQuickRefocusTimeout(15000);// Time to finish the session when any movement is detected (ms)
	_sessionManager->SetPrimaryStaticTimeout(2.0);// Time to recognize another gesture (s)
	_sessionInitialized = true;
}

// Registers session manager and return Handle, allowing unregistering.
XnVHandle registerSessionManager() {
	XnVHandle handleSessionManager = _sessionManager->RegisterSession(NULL,
			&SessionStart, &SessionEnd);
	_sessionRegistered = true;
	return handleSessionManager;
}

// Unregisters session manager
void unregisterSessionManager(XnVHandle h) {
	_sessionManager->UnregisterSession(h);
	_sessionRegistered = false;
}

// Extracts and sends hand position data and more
void handleHandPosition(bool fix_coordinates = true) {
	XnSkeletonJointPosition skeleton_hands[2]; // 0=left; 1=right
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user_id,
			XN_SKEL_LEFT_HAND, skeleton_hands[0]);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user_id,
			XN_SKEL_RIGHT_HAND, skeleton_hands[1]);
	if (skeleton_hands[0].fConfidence > 0.5) {// Left hand
		//printf("\nLeft Hand from Skeleton - (%3.3f, %3.3f, %4.3f), Confidence:%2.2f", skeleton_hands[0].position.X,	skeleton_hands[0].position.Y, skeleton_hands[0].position.Z,	skeleton_hands[0].fConfidence);
		if(l_hand_out_fov)l_hand_out_fov = false;
		g_DepthGenerator.ConvertRealWorldToProjective(1,
				&skeleton_hands[0].position, &skeleton_hands[0].position);
		if (fix_coordinates)
			fixCoordinates(&skeleton_hands[0].position);
		if (checkCoordinates(&l_last_point3d, skeleton_hands[0].position)) {// Hand in movement
			if (_printHandsTracking)
				printf(
						"\nLeft Hand from Skeleton - (%3.3f, %3.3f, %4.3f), Confidence:%2.2f",
						skeleton_hands[0].position.X,
						skeleton_hands[0].position.Y,
						skeleton_hands[0].position.Z,
						skeleton_hands[0].fConfidence);
			if (_useSockets)
				sendHandCoordinates(skeleton_hands[0].position, 1, 0);
			if (l_timer.isRunning())
				l_timer.reset();
		} else {// Hand isn't in movement
			l_timer.start();
			if (!l_timer.isOver(SECONDS_STEADY_HAND)) {
				XnSkeletonJointPosition skeleton_head;
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(
						user_id, XN_SKEL_HEAD, skeleton_head);
				g_DepthGenerator.ConvertRealWorldToProjective(1,
						&skeleton_head.position, &skeleton_head.position);
				if (_useSockets)
					//sendHeadCoordinates(skeleton_head.position, 1, 0); //TODO: Disable for tests
				l_timer.reset();
			}
		}
	}
	if (skeleton_hands[1].fConfidence > 0.5) {// Right hand
		//printf("\nRight Hand from Skeleton - (%3.3f, %3.3f, %4.3f), Confidence:%2.2f", skeleton_hands[1].position.X, skeleton_hands[1].position.Y, skeleton_hands[1].position.Z, skeleton_hands[1].fConfidence);
		if(r_hand_out_fov)r_hand_out_fov = false;
		g_DepthGenerator.ConvertRealWorldToProjective(1,
				&skeleton_hands[1].position, &skeleton_hands[1].position);
		if (fix_coordinates)
			fixCoordinates(&skeleton_hands[1].position);
		if (checkCoordinates(&r_last_point3d, skeleton_hands[1].position)) {// Hand in movement
			if (_printHandsTracking)
				printf(
						"\nRight Hand from Skeleton - (%3.3f, %3.3f, %4.3f), Confidence:%2.2f",
						skeleton_hands[1].position.X,
						skeleton_hands[1].position.Y,
						skeleton_hands[1].position.Z,
						skeleton_hands[1].fConfidence);
			if (_useSockets)
				sendHandCoordinates(skeleton_hands[1].position, 0, 1);
			if (r_timer.isRunning())
				r_timer.reset();
		} else {// Hand isn't in movement
			r_timer.start();
			if (!r_timer.isOver(SECONDS_STEADY_HAND)) {
				XnSkeletonJointPosition skeleton_head;
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(
						user_id, XN_SKEL_HEAD, skeleton_head);
				g_DepthGenerator.ConvertRealWorldToProjective(1,
						&skeleton_head.position, &skeleton_head.position);
				if(_useSockets)
					//sendHeadCoordinates(skeleton_head.position, 0, 1); //TODO: Disable for tests
				r_timer.reset();
			}
		}
	}
}

// Sends hand coordinates data to socket connection
void sendHandCoordinates(XnPoint3D h_coordinates, int is_l_hand, int is_r_hand) {
	char *header = "hand_coordinates";
	float coordinates[3] = { h_coordinates.X, h_coordinates.Y, h_coordinates.Z };
	char *gesture = "none";
	formatData(header, user_id, 0, is_l_hand, is_r_hand, coordinates, 0.0,
			gesture, 0.0, 0.0, 0.0);
}

// Sends head coordinates data to socket connection
// TODO: Verificar a necessidade de filtrar estes dados aqui, no cliente.
void sendHeadCoordinates(XnPoint3D h_coordinates, int is_l_hand, int is_r_hand) {
	char *header = "head_coordinates";
	float coordinates[3] = { h_coordinates.X, h_coordinates.Y, h_coordinates.Z };
	char *gesture = "none";
	formatData(header, user_id, 0, is_l_hand, is_r_hand, coordinates, 0.0,
			gesture, 0.0, 0.0, 0.0);
}

// Fix coordinates
void fixCoordinates(XnPoint3D *c) {
	if (c->X > res_x)
		c->X = res_x;
	else if (c->X < 0)
		c->X = 0;
	if (c->Y > res_y)
		c->Y = res_y;
	else if (c->Y < 0)
		c->Y = 0;
}

// Checks if the new point3d is valid, and stores it if so
bool checkCoordinates(XnPoint3D *last_point3d, XnPoint3D new_point3d) {
	if (last_point3d->X == 0 && last_point3d->Y == 0) {
		storeCoordinates(last_point3d, new_point3d);
		return true;
	} else {
		if (last_point3d->X + valid_step_x < new_point3d.X || last_point3d->X
				- valid_step_x > new_point3d.X) {// Test X
			storeCoordinates(last_point3d, new_point3d);
			return true;
		} else if (last_point3d->Y + valid_step_y < new_point3d.Y
				|| last_point3d->Y - valid_step_y > new_point3d.Y) {// Test Y
			storeCoordinates(last_point3d, new_point3d);
			return true;
		} else if (last_point3d->Z + valid_step_z < new_point3d.Z
				|| last_point3d->Z - valid_step_z > new_point3d.Z) {// Test Z
			storeCoordinates(last_point3d, new_point3d);
			return true;
		} else
			return false;
	}
}

// Stores last coordinates
void storeCoordinates(XnPoint3D *last_point3d, XnPoint3D new_point3d) {
	if (last_point3d->X != new_point3d.X || last_point3d->Y != new_point3d.Y
			|| last_point3d->Z != new_point3d.Z) {
		last_point3d->X = new_point3d.X;
		last_point3d->Y = new_point3d.Y;
		last_point3d->Z = new_point3d.Z;
	}
}

// Initializes variables to control repeated data of hands
void initLastPoint3d() {
	l_last_point3d.X = 0;
	l_last_point3d.Y = 0;
	l_last_point3d.Z = 0;
	r_last_point3d.X = 0;
	r_last_point3d.Y = 0;
	r_last_point3d.Z = 0;
}

/*Format of data:
 * #header|data_id|player_id|hand_id,left,right|x,y,z,c_p1|gesture,g_p1,g_p2,g_p3#
 * Formats the data and send it
 */
void formatData(char header[], int player_id, int hand_id, int l_hand,
		int r_hand, float coordinates[3], float c_p1, char gesture[],
		float g_p1, float g_p2, float g_p3) {
	printf("\n#%s|%i|%i|%i,%i,%i|%.3f,%.3f,%.3f,%.3f|%s,%.2f,%.2f,%.2f#\n",
			header, data_id, player_id, hand_id, l_hand, r_hand,
			coordinates[0], coordinates[1], coordinates[2], c_p1, gesture,
			g_p1, g_p2, g_p3);
	if (_useSockets) {
		char sensor_data[100];
		int n = sprintf(sensor_data,
				"#%s|%i|%i|%i,%i,%i|%.3f,%.3f,%.3f,%.3f|%s,%.2f,%.2f,%.2f#",
				header, data_id, player_id, hand_id, l_hand, r_hand,
				coordinates[0], coordinates[1], coordinates[2], c_p1, gesture,
				g_p1, g_p2, g_p3);
		try {
			tcp_sock.send(sensor_data, n);
		} catch (SocketException &e) {
			cerr << e.what() << endl;
			exit(1);
		}
	}
	data_id++;
	last_gesture = gesture;
}

// Clean up
void cleanUpExit() {
	if (_inSession) {
		removeListeners();
	}
	if (NULL != _sessionManager) {
		if (_sessionRegistered)
			unregisterSessionManager(hSessionManager);
		delete _sessionManager;
		_sessionManager = NULL;
	}
	delete _broadcaster;
	delete _waveDetector;
	delete _pushDetector;
	delete _swipeDetector;
	delete _steadyDetector;
	delete _circleDetector;
	delete g_pTexMap;

	g_ImageGenerator.Release();
	g_DepthGenerator.Release();
	g_HandsGenerator.Release();
	g_GestureGenerator.Release();
	g_UserGenerator.Release();
	g_Context.Release();
	if (_useSockets) {
		char exit_flag[2] = "0";
		try {
			tcp_sock.send(exit_flag, 2);
		} catch (SocketException &e) {
			cerr << e.what() << endl;
			exit(1);
		}
		tcp_sock.cleanUp();
		tcp_sock.~Socket();
	}
	printf("\nFinished!\n");
	exit(1);
}
//-----------------------------------------------------------------------------
// GUI
//-----------------------------------------------------------------------------

void glutIdle(void) {
	// Display the frame
	glutPostRedisplay();
}

void glutDisplay(void) {
	nRetVal = g_Context.WaitAnyUpdateAll();
	if (nRetVal != XN_STATUS_OK) {
		printf("Read failed: %s\n", xnGetStatusString(nRetVal));
		return;
	}
	if (_sessionInitialized) {
		_sessionManager->Update(&g_Context);
	}
	// Extract hand position of tracked user
	if (_featureHandsTracking && _inSession) {
		if (g_UserGenerator.GetSkeletonCap().IsTracking(user_id)) {
			handleHandPosition();
		}
	}
	g_ImageGenerator.GetMetaData(g_imageMD);

	// Clear the OpenGL buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

	xnOSMemSet(g_pTexMap, 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));

	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y) {
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex) {
			*pTex = *pImage;
		}

		pImageRow += g_imageMD.XRes();
		pTexRow += g_nTexMapX;
	}

	// Create the OpenGL texture map
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB,
			GL_UNSIGNED_BYTE, g_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1, 1, 1, 1);

	glBegin(GL_QUADS);

	int nXRes = g_imageMD.FullXRes();
	int nYRes = g_imageMD.FullYRes();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float) nXRes / (float) g_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float) nXRes / (float) g_nTexMapX, (float) nYRes
			/ (float) g_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float) nYRes / (float) g_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();

	// Swap the OpenGL display buffers
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key){
		case 27:
			cleanUpExit();
			exit (1);
	}
}

void glInit (int * pargc, char ** argv){
	g_ImageGenerator.GetMetaData(g_imageMD);

	// Texture map init
	g_nTexMapX = (((unsigned short) (g_imageMD.FullXRes() - 1) / 512) + 1)* 512;
	g_nTexMapY = (((unsigned short) (g_imageMD.FullYRes() - 1) / 512) + 1)* 512;
	g_pTexMap = (XnRGB24Pixel*) malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutInitWindowPosition(POSITION_X, POSITION_Y);
	glutCreateWindow ("Natural Interface to Blender 3D");
	//glutFullScreen();
	//glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

}

//-----------------------------------------------------------------------------
// Init Method
//-----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
	//moveKinectMotor(10); // Needs to run with root privileges

	if (_useSockets)
		initSocket("localhost", 2001);

	// Context Init
	nRetVal = g_Context.Init();
	CHECK_RC(nRetVal, "Initialize context");
	g_Context.SetGlobalMirror(_mirror);

	// Create the depth generator
	nRetVal = g_DepthGenerator.Create(g_Context);
	CHECK_RC(nRetVal, "Create depth generator");

	// Set it to VGA maps at 30 FPS (Depth image)
	_outputModeDepth.nXRes = res_x;
	_outputModeDepth.nYRes = res_y;
	_outputModeDepth.nFPS = 30;
	nRetVal = g_DepthGenerator.SetMapOutputMode(_outputModeDepth);
	CHECK_RC(nRetVal, "Set map output mode for depth generator");

	// Create the image generator and set output map (Image only)
	nRetVal = g_ImageGenerator.Create(g_Context);
	CHECK_RC(nRetVal, "Create image generator");
	_outputModeImage.nXRes = res_x;
	_outputModeImage.nYRes = res_y;
	_outputModeImage.nFPS = 30;
	nRetVal = g_ImageGenerator.SetMapOutputMode(_outputModeImage);
	CHECK_RC(nRetVal, "Set map output mode for image generator");

	// Create the hands generator
	nRetVal = g_HandsGenerator.Create(g_Context);
	CHECK_RC(nRetVal, "Create hands generator");
	// Smoothing hand movements
	g_HandsGenerator.SetSmoothing(KINECT_SMOOTHING_HANDS);
	initLastPoint3d();

	// Create the gesture generator
	nRetVal = g_GestureGenerator.Create(g_Context);
	CHECK_RC(nRetVal, "Create gesture generator");

	// Create user generator.
	nRetVal = g_UserGenerator.Create(g_Context);
	CHECK_RC(nRetVal, "Create user generator");

	//---------------------------------------------------------------//
	//------------------------- SETUP FEATURES ---------------------//
	//--------------------------------------------------------------//

	// Feature Gesture.
	if (_featureGesture) {
		// Wave detector.
		_waveDetector = new XnVWaveDetector();//TODO: Check both ways this gesture
		//_waveDetector->SetMinLength(25); // The length (in mm) of the motion before a direction change (a flip)
		/*printf(
		 "\nWave Detector Config - Flip Count:%d, Min Length:%d, Max Deviation:%d",
		 _waveDetector->GetFlipCount(), _waveDetector->GetMinLength(),
		 _waveDetector->GetMaxDeviation());*/
		_waveDetector->RegisterWave(NULL, &OnWave);

		// Push detector.
		_pushDetector = new XnVPushDetector();
		_pushDetector->SetPushImmediateDuration(400); // Change the time used to detect push (ms)
		_pushDetector->SetPushImmediateMinimumVelocity(0.35f); // To test
		_pushDetector->RegisterPush(NULL, &OnPush);
		//_pushDetector->RegisterStabilized(NULL, &StabilizedPush);

		// Swipe detector.
		_swipeDetector = new XnVSwipeDetector();
		_swipeDetector->SetMotionTime(700); // Minimal duration to recognize as swipe (ms)
		_swipeDetector->SetMotionSpeedThreshold(0.55f); // Minimal speed to recognize as a swipe (m/s)
		_swipeDetector->RegisterSwipeUp(NULL, &SwipeUp);
		_swipeDetector->RegisterSwipeDown(NULL, &SwipeDown);
		_swipeDetector->RegisterSwipeLeft(NULL, &SwipeLeft);
		_swipeDetector->RegisterSwipeRight(NULL, &SwipeRight);

		// Steady detector.
		_steadyDetector = new XnVSteadyDetector();
		_steadyDetector->SetDetectionDuration(800); // The time it takes to detect steady state (ms) - The previus value was 1000up
		_steadyDetector->RegisterSteady(NULL, &OnSteady);
		//_steadyDetector->RegisterNotSteady(NULL, &NotSteady);
	}

	// Feature Circle.
	if (_featureCircle) {
		// Circle detector.
		_circleDetector = new XnVCircleDetector();
		_circleDetector->SetMinimumPoints(16); // Minimum number of points to consider a circle
		_circleDetector->RegisterCircle(NULL, &CircleCB);
		_circleDetector->RegisterNoCircle(NULL, &NoCircleCB);
	}

	// Feature User Tracking.
	if (_featureUserTracking) {
		// Setup user generator callbacks.
		XnCallbackHandle hUserCallbacks, hCalibrationStart,
				hCalibrationComplete, hPoseDetected;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
			printf("\nSupplied user generator doesn't support skeleton");
			return 1;
		}
		nRetVal = g_UserGenerator.RegisterUserCallbacks(NewUser, LostUser,
				NULL, hUserCallbacks);
		CHECK_RC(nRetVal, "Register to user callbacks");
		nRetVal = g_UserGenerator.RegisterToUserExit(UserExit, NULL,
				hUserCallbacks);
		CHECK_RC(nRetVal, "Register to user exit callback");
		nRetVal = g_UserGenerator.RegisterToUserReEnter(UserReEnter, NULL,
				hUserCallbacks);
		CHECK_RC(nRetVal, "Register to user re-enters callback");

		// Setup Skeleton detection.
		nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(
				UserCalibrationStart, NULL, hCalibrationStart);
		CHECK_RC(nRetVal, "Register to calibration start");
		nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(
						UserCalibrationComplete, NULL, hCalibrationComplete);
		CHECK_RC(nRetVal, "Register to calibration complete");

		if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
			printf("\nPose required!");
			_needPose = true;
			if (!g_UserGenerator.IsCapabilitySupported(
					XN_CAPABILITY_POSE_DETECTION)) {
				printf("\nPose required, but not supported.");
				return 1;
			}
			g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(
					UserPoseDetected, NULL, hPoseDetected);
			g_UserGenerator.GetSkeletonCap().GetCalibrationPose(_strPose);
		}
		g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(
				XN_SKEL_PROFILE_HEAD_HANDS);
		g_UserGenerator.GetSkeletonCap().SetSmoothing(KINECT_SMOOTHING_SKELETON);
	}

	// Create the broadcaster manager.
	_broadcaster = new XnVBroadcaster();

	// Start generating all
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "Start Generating All");

	// Set the frame rate.
	//nRetVal = xnFPSInit(&_xnFPS, 180);// TODO: Verify this functionality and turn on then
	//CHECK_RC(nRetVal, "FPS Init");

	printf("\nStarting!");

	//-------------------------------------------------------------//
	//------------------------- MAIN LOOP ------------------------//
	//-----------------------------------------------------------//
#if (USE_GUI_GLUT == 1)
	glInit(&argc, argv);
	// Per frame code is in glutDisplay
	glutMainLoop();
#else
	while (!xnOSWasKeyboardHit()) {
		// Update to next frame
		nRetVal = g_Context.WaitAnyUpdateAll();
		CHECK_RC(nRetVal, "Update data");
		if (_sessionInitialized) {
			_sessionManager->Update(&g_Context);
		}
		// Extract hand position of tracked user
		if (_featureHandsTracking && _inSession) {
			if (g_UserGenerator.GetSkeletonCap().IsTracking(user_id)) {
				handleHandPosition();
			}
		}
	}

	cleanUpExit();
#endif
	return 0;
}
