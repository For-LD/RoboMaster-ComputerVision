#include "getStream.hpp"

//-------------------------------------------------
/**
\brief Convert frame date to suitable pixel format
\param pParam[in]           pFrameBuffer       FrameData from camera
\return void
*/
//-------------------------------------------------
int DH_CAMERA::PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer)
{
	GX_STATUS emStatus = GX_STATUS_SUCCESS;
	VxInt32 emDXStatus = DX_OK;

	// Convert RAW8 or RAW16 image to RGB24 image
	switch (pFrameBuffer->nPixelFormat)
	{
	case GX_PIXEL_FORMAT_BAYER_GR8:
	case GX_PIXEL_FORMAT_BAYER_RG8:
	case GX_PIXEL_FORMAT_BAYER_GB8:
	case GX_PIXEL_FORMAT_BAYER_BG8:
	{
		// Convert to the RGB image
		emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
			RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
			return PIXFMT_CVT_FAIL;
		}
		break;
	}
	case GX_PIXEL_FORMAT_BAYER_GR10:
	case GX_PIXEL_FORMAT_BAYER_RG10:
	case GX_PIXEL_FORMAT_BAYER_GB10:
	case GX_PIXEL_FORMAT_BAYER_BG10:
	case GX_PIXEL_FORMAT_BAYER_GR12:
	case GX_PIXEL_FORMAT_BAYER_RG12:
	case GX_PIXEL_FORMAT_BAYER_GB12:
	case GX_PIXEL_FORMAT_BAYER_BG12:
	{
		// Convert to the Raw8 image
		emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, g_pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
			return PIXFMT_CVT_FAIL;
		}
		// Convert to the RGB24 image
		emDXStatus = DxRaw8toRGB24((unsigned char*)g_pRaw8Image, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
			RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
		if (emDXStatus != DX_OK)
		{
			printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
			return PIXFMT_CVT_FAIL;
		}
		break;
	}
	default:
	{
		printf("Error : PixelFormat of this camera is not supported\n");
		return PIXFMT_CVT_FAIL;
	}
	}
	return PIXFMT_CVT_SUCCESS;
}

//-------------------------------------------------
/**
\brief Allocate the memory for pixel format transform
\return void
*/
//-------------------------------------------------
void DH_CAMERA::PreForAcquisition()
{
	g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3];
	g_pRaw8Image = new unsigned char[g_nPayloadSize];

	return;
}

//-------------------------------------------------
/**
\brief Release the memory allocated
\return void
*/
//-------------------------------------------------
void DH_CAMERA::UnPreForAcquisition()
{
	//Release resources
	if (g_pRaw8Image != NULL)
	{
		delete[] g_pRaw8Image;
		g_pRaw8Image = NULL;
	}
	if (g_pRGBImageBuf != NULL)
	{
		delete[] g_pRGBImageBuf;
		g_pRGBImageBuf = NULL;
	}

	return;
}


//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void DH_CAMERA::GetErrorString(GX_STATUS emErrorStatus)
{
	char* error_info = NULL;
	size_t size = 0;
	GX_STATUS emStatus = GX_STATUS_SUCCESS;

	// Get length of error description
	emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		printf("<Error when calling GXGetLastError>\n");
		return;
	}

	// Alloc error resources
	error_info = new char[size];
	if (error_info == NULL)
	{
		printf("<Failed to allocate memory>\n");
		return;
	}

	// Get error description
	emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		printf("<Error when calling GXGetLastError>\n");
	}
	else
	{
		printf("%s\n", (char*)error_info);
	}

	// Realease error resources
	if (error_info != NULL)
	{
		delete[]error_info;
		error_info = NULL;
	}
}


DH_CAMERA::DH_CAMERA()
{
	initCamera();
	initGrabStream();

}


DH_CAMERA::DH_CAMERA(int isAutoExposure, float exposureTime, int ROI_width = 1280, int ROI_height = 1024)
{

	initCamera();

	int64_t nWidth = ROI_width;
	int64_t nHeight = ROI_height;
	int64_t nOffsetX = 640 - (ROI_width / 2);
	int64_t nOffsetY = 512 - (ROI_height / 2);

	GX_STATUS emStatus = GX_STATUS_SUCCESS;

	GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, 0);

	emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposureTime);
	std::cout << emStatus << std::endl;

	double dValue = 0;
	emStatus = GXGetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, &dValue);
	std::cout << "expo_time:" << dValue << std::endl;

	GXSetInt(g_hDevice, GX_INT_WIDTH, nWidth);
	GXSetInt(g_hDevice, GX_INT_HEIGHT, nHeight);
	GXSetInt(g_hDevice, GX_INT_OFFSET_X, nOffsetX);
	GXSetInt(g_hDevice, GX_INT_OFFSET_Y, nOffsetY);

	initGrabStream();

}

int DH_CAMERA::initCamera()
{
	GX_STATUS emStatus = GX_STATUS_SUCCESS;

	uint32_t ui32DeviceNum = 0;

	//Initialize libary
	emStatus = GXInitLib();
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		return emStatus;
	}

	//Get device enumerated number
	emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		GXCloseLib();
		return emStatus;
	}

	//If no device found, app exit
	if (ui32DeviceNum <= 0)
	{
		printf("<No device found>\n");
		GXCloseLib();
		return emStatus;
	}

	//Open first device enumerated
	emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		GXCloseLib();
		return emStatus;
	}

	//Get Device Info
	printf("***********************************************\n");
	//Get libary version
	printf("<Libary Version : %s>\n", GXGetLibVersion());
	size_t nSize = 0;
	//Get string length of Vendor name
	emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
	GX_VERIFY_EXIT(emStatus);
	//Alloc memory for Vendor name
	char* pszVendorName = new char[nSize];
	//Get Vendor name
	emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		delete[] pszVendorName;
		pszVendorName = NULL;
		GX_VERIFY_EXIT(emStatus);
	}

	printf("<Vendor Name : %s>\n", pszVendorName);
	//Release memory for Vendor name
	delete[] pszVendorName;
	pszVendorName = NULL;

	//Get string length of Model name
	emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
	GX_VERIFY_EXIT(emStatus);
	//Alloc memory for Model name
	char* pszModelName = new char[nSize];
	//Get Model name
	emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		delete[] pszModelName;
		pszModelName = NULL;
		GX_VERIFY_EXIT(emStatus);
	}

	printf("<Model Name : %s>\n", pszModelName);
	//Release memory for Model name
	delete[] pszModelName;
	pszModelName = NULL;

	//Get string length of Serial number
	emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
	GX_VERIFY_EXIT(emStatus);
	//Alloc memory for Serial number
	char* pszSerialNumber = new char[nSize];
	//Get Serial Number
	emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		delete[] pszSerialNumber;
		pszSerialNumber = NULL;
		GX_VERIFY_EXIT(emStatus);
	}

	printf("<Serial Number : %s>\n", pszSerialNumber);
	//Release memory for Serial number
	delete[] pszSerialNumber;
	pszSerialNumber = NULL;

	//Get string length of Device version
	emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VERSION, &nSize);
	GX_VERIFY_EXIT(emStatus);
	char* pszDeviceVersion = new char[nSize];
	//Get Device Version
	emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		delete[] pszDeviceVersion;
		pszDeviceVersion = NULL;
		GX_VERIFY_EXIT(emStatus);
	}

	printf("<Device Version : %s>\n", pszDeviceVersion);
	//Release memory for Device version
	delete[] pszDeviceVersion;
	pszDeviceVersion = NULL;
	printf("***********************************************\n");

	//Get the type of Bayer conversion. whether is a color camera.
	emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
	GX_VERIFY_EXIT(emStatus);

	//This app only support color cameras
	if (!g_bColorFilter)
	{
		printf("<This app only support color cameras! App Exit!>\n");
		GXCloseDevice(g_hDevice);
		g_hDevice = NULL;
		GXCloseLib();
		return 0;
	}
	else
	{
		emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
		GX_VERIFY_EXIT(emStatus);
	}

	emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
	GX_VERIFY(emStatus);


	//Set acquisition mode
	emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
	GX_VERIFY_EXIT(emStatus);

	//Set trigger mode
	emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
	GX_VERIFY_EXIT(emStatus);

	//Set buffer quantity of acquisition queue
	uint64_t nBufferNum = ACQ_BUFFER_NUM;
	emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
	GX_VERIFY_EXIT(emStatus);

	//Set size of data transfer block
	emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
	GX_VERIFY_EXIT(emStatus);

	//Set qty. of data transfer block
	emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
	GX_VERIFY_EXIT(emStatus);

	//Set Balance White Mode : Continuous
	emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
	GX_VERIFY_EXIT(emStatus);

	//set exposure mode :off
	emStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, 0);
	GX_VERIFY_EXIT(emStatus);

	//set exposure time:8000(us)
	emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 20000);
	GX_VERIFY_EXIT(emStatus);


}


void DH_CAMERA::initGrabStream()
{
	GX_STATUS emStatus = GX_STATUS_SUCCESS;
	//Allocate the memory for pixel format transform 
	PreForAcquisition();

	//Device start acquisition
	emStatus = GXStreamOn(g_hDevice);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		//Release the memory allocated
		UnPreForAcquisition();
		GetErrorString(emStatus);
		GXCloseDevice(g_hDevice);
		g_hDevice = NULL;
		GXCloseLib();
	}
}

//-------------------------------------------------
/**
\brief Acquisition thread function
\param pParam       thread param, not used in this app
\return void*
*/
//-------------------------------------------------
int DH_CAMERA::ProcGetImage(cv::Mat& image)
{
	GX_STATUS emStatus = GX_STATUS_SUCCESS;

	/*if(!ui32FrameCount)
	{
		time(&lInit);
	}*/

	// Get a frame from Queue
	emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		if (emStatus == GX_STATUS_TIMEOUT)
		{
			return -1;
		}
		else
		{
			GetErrorString(emStatus);
			return -2;
		}
	}

	if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
	{
		printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
		return -3;
	}
	else
	{
		ui32FrameCount++;
		//time (&lEnd);
		// Print acquisition info each second.
		/*if (lEnd - lInit >= 1)
		{
			printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %llu>\n",
				ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);

		}*/

		int nRet = PixelFormatConvert(pFrameBuffer);
		if (nRet == PIXFMT_CVT_SUCCESS)
		{
			//SavePPMFile(pFrameBuffer->nWidth, pFrameBuffer->nHeight);
			cv::Mat img = cv::Mat(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);
			memcpy(img.data, g_pRGBImageBuf, pFrameBuffer->nHeight * pFrameBuffer->nWidth * 3);
			cv::resize(img, image, cv::Size(416, 416));
			cvtColor(image, image, COLOR_BGR2RGB);

		}
		else
		{
			printf("PixelFormat Convert failed!\n");
			return 1;
		}
	}

	emStatus = GXQBuf(g_hDevice, pFrameBuffer);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		return 2;
	}

	return 0;
}

int DH_CAMERA::closeCamera()
{
	GX_STATUS emStatus = GX_STATUS_SUCCESS;

	//Device stop acquisition
	emStatus = GXStreamOff(g_hDevice);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		//Release the memory allocated
		UnPreForAcquisition();
		GX_VERIFY_EXIT(emStatus);
	}

	//Release the resources and stop acquisition thread
	UnPreForAcquisition();

	//Close device
	emStatus = GXCloseDevice(g_hDevice);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		g_hDevice = NULL;
		GXCloseLib();
		return emStatus;
	}

	//Release libary
	emStatus = GXCloseLib();
	if (emStatus != GX_STATUS_SUCCESS)
	{
		GetErrorString(emStatus);
		return emStatus;
	}
}








