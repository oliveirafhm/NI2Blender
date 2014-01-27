/*
 * methods.h
 *
 *  Created on: Jan 9, 2012
 *      Author: fabio
 */

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace std;

#ifndef METHODS_H_
#define METHODS_H_

// Returns array length
template<typename T, int size>
int getArrayLength(T(&)[size]) {
	return size;
}

// Client socket initialization and configuration
void initSocket(string servAddress, unsigned short servPort);

void addListeners();

void removeListeners();

// Create and initialize session manager
void initSessionManager();

// Registers session manager
XnVHandle registerSessionManager();

// Unregisters session manager
void unregisterSessionManager(XnVHandle h);

// Extracts and sends hand position data
void handleHandPosition(bool fix_coordinates);

// Sends hand coordinates data to socket connection
void sendHandCoordinates(XnPoint3D h_coordinates, int is_l_hand, int is_r_hand);

void sendHeadCoordinates(XnPoint3D h_coordinates, int is_l_hand, int is_r_hand);

// Fix coordinates
void fixCoordinates(XnPoint3D *c);

// Checks if the new point3d is valid, and stores it if so
bool checkCoordinates(XnPoint3D *last_point3d, XnPoint3D new_point3d);

// Checks if the new point3d is valid, and stores it if so
void storeCoordinates(XnPoint3D *last_point3d, XnPoint3D new_point3d);

// Initializes variables to control repeated data
void initLastPoint3d();

/*Format of data: #header|data_id|player_id|hand_id,left,right|x,y,z,ftime|gesture,p1,p2,p3#
 Formats the data and send it*/
void formatData(char header[], int player_id, int hand_id, int l_hand,
		int r_hand, float coordinates[3], float ftime, char gesture[],
		float p1, float p2, float p3);

// Clean up
void cleanUpExit();

//-----------------------------------------------------------------------------
// MyMethods.cpp
//-----------------------------------------------------------------------------
void errorCheck(XnStatus rc);

void moveKinectMotor(int angle);

// Invert float array
void invertFloatArray(float array[], int length);

//-----------------------------------------------------------------------------
// GUI
//-----------------------------------------------------------------------------

void glutIdle(void);

void glutDisplay(void);

void glutKeyboard (unsigned char key, int x, int y);

void glInit (int * pargc, char ** argv);

#endif /* METHODS_H_ */
