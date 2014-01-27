/*Hello all,
 Here is easy way to work with Kinect's motor, through OpenNI. No other
 drivers or libraries is required.
 The code is following:

// OpenNI includes
#include <XnUSB.h>
// Standard includes
#include <stdio.h>
#include <time.h>
*
 * Class to control Kinect's motor.

class KinectMotor {
public:
	KinectMotor();
	virtual ~KinectMotor();
	*
	 * Open device.
	 * @return true if succeeded, false - overwise

	bool Open();
	*
	 * Close device.

	void Close();
	*
	 * Move motor up or down to specified angle value.
	 * @param angle angle value
	 * @return true if succeeded, false - overwise

	bool Move(int angle);
private:
	XN_USB_DEV_HANDLE m_dev;
	bool m_isOpen;
};

KinectMotor::KinectMotor() {
	m_isOpen = false;
}

KinectMotor::~KinectMotor() {
	Close();
}

bool KinectMotor::Open() {
	const XnUSBConnectionString *paths;
	XnUInt32 count;
	XnStatus res;
	// Init OpenNI USB
	res = xnUSBInit();
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBInit failed");
		return false;
	}
	// Open "Kinect motor" USB device
	res = xnUSBEnumerateDevices(0x045E  VendorID , 0x02B0 ProductID
	 , &paths, &count);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBEnumerateDevices failed");
		return false;
	}
	// Open first found device
	res = xnUSBOpenDeviceByPath(paths[0], &m_dev);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBOpenDeviceByPath failed");
		return false;
	}
	XnUChar buf[1]; // output buffer
	// Init motor
	res = xnUSBSendControl(m_dev, (XnUSBControlType) 0xc0, 0x10, 0x00, 0x00, buf, sizeof(buf), 0);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBSendControl failed");
		Close();
		return false;
	}
	res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBSendControl failed");
		Close();
		return false;
	}
	return true;
}

void KinectMotor::Close() {
	if (m_isOpen) {
		xnUSBCloseDevice(m_dev);
		m_isOpen = false;
	}
}

bool KinectMotor::Move(int angle) {
	XnStatus res;
	// Send move control request
	res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x31, angle,
			0x00, NULL, 0, 0);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBSendControl failed");
		return false;
	}
	return true;
}

int main() {
	KinectMotor motor;
	if (!motor.Open()) // Open motor device
		return 1;
	motor.Move(31); // move it up to 31 degree
	//Sleep(1000);
	//motor.Move(-31); // move it down to 31 degree
	//Sleep(1000);
	//motor.Move(0);
	return 0;
}*/

//-------------------------------------------------------------------------------

#include <stdlib.h>
#include <stdexcept>
#include <iostream>
#include <XnCppWrapper.h>
#include <XnUSB.h>

#define VID_MICROSOFT 0x45e
#define PID_NUI_MOTOR 0x02b0

XN_USB_DEV_HANDLE dev2;

void errorCheck2(XnStatus rc) {
    if (rc != XN_STATUS_OK) {
        std::cout << xnGetStatusString(rc) << std::endl;
        throw std::runtime_error("Error");
    }
}

int main () {

    int angle = 10;

    XnStatus rc = XN_STATUS_OK;

    rc = xnUSBInit();
    errorCheck2(rc);

    rc = xnUSBOpenDevice(VID_MICROSOFT, PID_NUI_MOTOR, NULL, NULL, &dev2);
    errorCheck2(rc);

    XnUChar empty[0x1];
    angle = angle * 2;

    rc = xnUSBSendControl(dev2,
                          XN_USB_CONTROL_TYPE_VENDOR,
                          0x31,
                          (XnUInt16)angle,
                          0x0,
                          empty,
                          0x0, 0);
    errorCheck2(rc);

    rc = xnUSBCloseDevice(dev2);
    errorCheck2(rc);

    return 0;
}
