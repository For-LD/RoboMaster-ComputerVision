#include "GxIAPI.h"
#include "DxImageProc.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50              ///< Save image file name length

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success

//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        return emStatus;                   \
    }


class DH_CAMERA
{
private:
	GX_DEV_HANDLE g_hDevice = NULL;                     ///< Device handle
	bool g_bColorFilter = false;                        ///< Color filter support flag
	int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device
	bool g_bAcquisitionFlag = false;                    ///< Thread running flag
	bool g_bSavePPMImage = false;                       ///< Save raw image flag
	pthread_t g_nAcquisitonThreadID = 0;                ///< Thread ID of Acquisition thread

	unsigned char* g_pRGBImageBuf = NULL;               ///< Memory for RAW8toRGB24
	unsigned char* g_pRaw8Image = NULL;                 ///< Memory for RAW16toRAW8

	int64_t g_nPayloadSize = 0;                         ///< Payload size

	PGX_FRAME_BUFFER pFrameBuffer = NULL;

	time_t lInit;
	time_t lEnd;
	uint32_t ui32FrameCount = 0;
	uint32_t ui32AcqFrameRate = 0;

	cv::Mat image;
public:
	DH_CAMERA();
	DH_CAMERA(int isAutoExposure, float exposureTime, int ROI_width = 1280, int ROI_height = 1024);
	//~DH_CAMERA();

	//Allocate the memory for pixel format transform 
	void PreForAcquisition();

	//Release the memory allocated
	void UnPreForAcquisition();

	int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer);

	// RGB image convert to OpenCV BGR image
	//void RGB2BGR(cv::Mat & img);

	//Acquisition thread function
	int ProcGetImage(cv::Mat& image);

	//Get description of error
	void GetErrorString(GX_STATUS emErrorStatus);

	int initCamera();

	void initGrabStream();

	int closeCamera();

};



