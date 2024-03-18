//***************************************************************************************/
// 
// File name: SmartRay_ECCO_M10PP3.cpp  
//
// Synopsis:  This program contains an example of 3d reconstruction by interfacing with
//            a SmartRay ECCO.
//            See the PrintHeader() function below for detailed description.
//
// Copyright © 1992-2024 Zebra Technologies Corp. and/or its affiliates
// All Rights Reserved

#include <mil.h>

// Once the SMARTRAY SDK is installed and the project is configured correctly (see
// PrintHeader()), set SMARTRAY_INSTALLED to 1 to enable the SmartRay-specific code
// in SmartRayAcquisition.h.
// You need an actual SmartRay connected to your computer.
// Adjust the exposure time to get the best result. 

//****************************************************************************
// Example description.
//****************************************************************************
void PrintHeader()   
   {
   MosPrintf(MIL_TEXT("[EXAMPLE NAME]\n")
             MIL_TEXT("SmartRay_ECCO_M10PP3\n\n")

             MIL_TEXT("[SYNOPSIS]\n")
             MIL_TEXT("This example acquires a point cloud using a SmartRay ECCO profile\n")
             MIL_TEXT("sensor with the SmartRay API. The point cloud is then extracted\n")
             MIL_TEXT("into a depth map that is displayed.\n\n")

             MIL_TEXT("[MODULES USED]\n")
             MIL_TEXT("Modules used: application, buffer, calibration, display,\n")
             MIL_TEXT("              image processing, system.\n\n"));
   }

//*****************************************************************************
// DirectX display
//*****************************************************************************

// DirectX display is only available under Windows.
#if M_MIL_USE_WINDOWS && !M_MIL_USE_RT
   #define USE_D3D_DISPLAY  1
#else
   #define USE_D3D_DISPLAY  0
#endif

#if USE_D3D_DISPLAY
   #include "MdispD3D.h"

   // D3D display parameters.
   const MIL_INT    D3D_DISPLAY_SIZE_X = 640;
   const MIL_INT    D3D_DISPLAY_SIZE_Y = 480;
   const MIL_DOUBLE MAX_Z_GAP_DISTANCE = 2.0; // in mm
#endif

//*****************************************************************************
// SmartRay-specific includes.
//*****************************************************************************
#include "SmartRayAcquisition.h"
#if SMARTRAY_INSTALLED
#ifndef WIN64
   #pragma comment(lib, "SR_API.lib")
#else
   #pragma comment(lib, "SR_API-x64.lib")
#endif
#include <iostream>
#endif

// Name of the sensor. Please adjust the name to match the folder of 
// the parameter set file.

static const char* SENSOR_NAME = "ECCO75";

static const char* SMARTRAY_PATH = "C:\\SmartRay\\SmartRay DevKit";

static const MIL_INT EXPOSURE_TIME_1 = 1000; // in us
static const MIL_INT EXPOSURE_TIME_2 = 1000; // in us
static const MIL_INT NB_PROFILES = 400;
static const MIL_FLOAT CONVEYOR_SPEED = 0.1f; // in mm/frame
static const MIL_INT MAX_PATH = 260;


#define CHECK_SR( call )       \
   {                           \
   int apiReturnCode = call;   \
   if( apiReturnCode != 0 )    \
      return apiReturnCode;    \
   }                           \

//*****************************************************************************
// SmartRay acquisition prototypes.
//*****************************************************************************
int InitCamera(MIL_ID MilSystem);
int GrabPointCloud(MIL_ID MilPointCloudContainer, MIL_UINT *pNbValidPoint);
int GetDepthMapInformation(MIL_INT* pDepthMapWidth, MIL_DOUBLE* pPixelSizeX, MIL_DOUBLE* pBoxMinX,
                           MIL_DOUBLE* pBoxMaxX);
int ReleaseCamera();

//*****************************************************************************
// Main.
//*****************************************************************************
int MosMain()
   {
   PrintHeader();

#if !SMARTRAY_INSTALLED
   MosPrintf(MIL_TEXT("This example is designed for a SmartRay ECCO profile sensor\n"));
   MosPrintf(MIL_TEXT("and the SmartRay API. To run the example:\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Install the SmartRay SDK.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Follow the SmartRay ECCO user manual instructions for\n"));
   MosPrintf(MIL_TEXT("  your sensor model:\n"));
   MosPrintf(MIL_TEXT("  - Connect the SmartRay ECCO sensor to your computer.\n"));
   MosPrintf(MIL_TEXT("  - Change your network adapter settings.\n"));
   MosPrintf(MIL_TEXT("  - Verify the camera connection using SR_Studio.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Update the code in this example:\n"));
   MosPrintf(MIL_TEXT("  - Change the sensor name to reflect your actual model.\n"));
   MosPrintf(MIL_TEXT("    Follow the convention of the parameter set file.\n"));
   MosPrintf(MIL_TEXT("       Ex.: ECCO55, ECCO75...\n"));
   MosPrintf(MIL_TEXT("  - Change the SMARTRAY_PATH to the installation directory\n"));
   MosPrintf(MIL_TEXT("    of SmartRay.\n"));
   MosPrintf(MIL_TEXT("       Ex: C:\\SmartRay\\SmartRay DevKit\n"));
   MosPrintf(MIL_TEXT("  - Set the SMARTRAY_INSTALLED define to 1 in SmartRayAcquisition.h.\n"));
   MosPrintf(MIL_TEXT("  - Recompile the example.\n"));
   MosPrintf(MIL_TEXT("\n\n"));
   MosPrintf(MIL_TEXT("The example has been tested with the following setups:\n"));
   MosPrintf(MIL_TEXT("- Windows 10 64-bit.\n"));
   MosPrintf(MIL_TEXT("- SmartRay ECCO 55.\n"));
   MosPrintf(MIL_TEXT("- SmartRay ECCO 75.\n"));
   MosPrintf(MIL_TEXT("- SmartRay API  (versions 5.0.0.357).\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("* Note: The exposure time needs to be adjusted to acquire the best result.\n\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
   MosGetch();
#else
   MosPrintf(MIL_TEXT("Press <Enter> to start.\n\n"));
   MosGetch();

   MIL_ID MilApplication = MappAlloc(M_DEFAULT, M_NULL);
   MIL_ID MilSystem = MsysAlloc(MilApplication, M_SYSTEM_HOST, M_DEFAULT, M_DEFAULT, M_NULL);
   MIL_ID MilDisplay = MdispAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, M_NULL);

   MIL_ID MilDisplayLUT = MbufAllocColor(MilSystem, 3, 65536, 1, 8 + M_UNSIGNED, M_LUT, M_NULL);
   MgenLutFunction(MilDisplayLUT, M_COLORMAP_JET, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   MIL_UINT NbValidPoints = 0;
   MIL_UINT8 ColorForInvalidDepth[3] = {128, 128, 128};
   MbufPutColor2d(MilDisplayLUT, M_PLANAR, M_ALL_BANDS, 65535, 0, 1, 1, ColorForInvalidDepth);
   MdispLut(MilDisplay, MilDisplayLUT);

   // Allocate the point cloud container.
   MIL_ID MilPointCloudContainer = M3dmapAllocResult(MilSystem, M_POINT_CLOUD_CONTAINER, M_DEFAULT, M_NULL);

   // Initialize the SmartRay camera.
   int SRApiRetCode = InitCamera(MilSystem);
   if(SRApiRetCode == SUCCESS)
      {
      // Grab a point cloud from the SmartRay camera.
      SRApiRetCode = GrabPointCloud(MilPointCloudContainer, &NbValidPoints);

      if(SRApiRetCode == SUCCESS && NbValidPoints > 0)
         {
         // Get the depth map information.
         MIL_DOUBLE BoxMinX;
         MIL_DOUBLE BoxMaxX;
         MIL_INT DepthMapWidth;
         MIL_DOUBLE PixelSizeX;
         SRApiRetCode = GetDepthMapInformation(&DepthMapWidth, &PixelSizeX, &BoxMinX, &BoxMaxX);
         if(SRApiRetCode == SUCCESS)
            {
            // Set the extraction box. The box X dimension is based on SmartRay
            // suggested ZMap. The Y dimension is based on the conveyor speed. The Z dimension
            // is based on the Z range of the depth data extracted.
            M3dmapSetBox(MilPointCloudContainer, M_EXTRACTION_BOX, M_BOUNDING_BOX,
                         M_ALL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
            MIL_DOUBLE BoxMinZ;
            MIL_DOUBLE BoxMaxZ;
            M3dmapInquire(MilPointCloudContainer, M_GENERAL, M_EXTRACTION_BOX_MIN_Z, &BoxMinZ);
            M3dmapInquire(MilPointCloudContainer, M_GENERAL, M_EXTRACTION_BOX_MAX_Z, &BoxMaxZ);
            M3dmapSetBox(MilPointCloudContainer, M_EXTRACTION_BOX, M_BOTH_CORNERS,
                         BoxMinX, 0, BoxMinZ, BoxMaxX, CONVEYOR_SPEED * NB_PROFILES, BoxMaxZ);

            // Allocate the depth map and extract the point cloud.
            MIL_ID MilDepthMap = MbufAlloc2d(MilSystem, DepthMapWidth, SRAcq.NbProfileAdded(),
                                             16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
            MIL_ID MilIntensityMap = MbufAlloc2d(MilSystem, DepthMapWidth, SRAcq.NbProfileAdded(),
                                                 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
            M3dmapControl(MilPointCloudContainer, M_GENERAL, M_AUTO_SCALE_ASPECT_RATIO, M_UNCONSTRAINED);
            M3dmapControl(MilPointCloudContainer, M_GENERAL, M_FILL_MODE, M_X_THEN_Y);
            M3dmapControl(MilPointCloudContainer, M_GENERAL, M_FILL_THRESHOLD_X, PixelSizeX*3.0);
            M3dmapExtract(MilPointCloudContainer, MilDepthMap, MilIntensityMap,
                          M_CORRECTED_DEPTH_MAP, M_DEFAULT, M_DEFAULT);

            MosPrintf(MIL_TEXT("The acquired 3d point cloud was extracted to a MIL depth map and is now\n")
                      MIL_TEXT("being displayed.\n\n"));
            MdispSelect(MilDisplay, MilDepthMap);
            MdispControl(MilDisplay, M_SCALE_DISPLAY, M_ONCE);

#if USE_D3D_DISPLAY
            MIL_ID MilIntensityMap8 = MbufAlloc2d(MilSystem, DepthMapWidth, SRAcq.NbProfileAdded(),
                                                  8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);

            MimRemap(M_DEFAULT, MilIntensityMap, MilIntensityMap8, M_FIT_SRC_DATA);

            // Try to allocate D3D display.
            MIL_DISP_D3D_HANDLE DispHandle = MdepthD3DAlloc(MilDepthMap,
                                                            MilIntensityMap8,
                                                            D3D_DISPLAY_SIZE_X,
                                                            D3D_DISPLAY_SIZE_Y,
                                                            M_DEFAULT,
                                                            M_DEFAULT,
                                                            M_DEFAULT,
                                                            M_DEFAULT,
                                                            M_DEFAULT,
                                                            MAX_Z_GAP_DISTANCE,
                                                            0);
            MbufFree(MilIntensityMap8);

            if(DispHandle != NULL)
               {
               MosPrintf(MIL_TEXT("The depth map and its associated intensity map are\n")
                         MIL_TEXT("also displayed in a Direct3D window.\n\n"));
               MdispD3DShow(DispHandle);
               MdispD3DPrintHelp(DispHandle);
               }
#endif

            MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
            MosGetch();

            MbufFree(MilIntensityMap);
            MbufFree(MilDepthMap);

#if USE_D3D_DISPLAY
            if(DispHandle != M_NULL)
               {
               MdispD3DHide(DispHandle);
               MdispD3DFree(DispHandle);
               }
#endif  
            }
         }
      }

   if(SRApiRetCode == SUCCESS && NbValidPoints == 0)
      {
      MosPrintf(MIL_TEXT("The camera could not capture any data; there is nothing in the scene.\n"));
      MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
      MosGetch();
      }

   if(SRApiRetCode)
      {
      char *SmartRayApiError = NULL;
      SR_API_GetErrorMsg(SRApiRetCode, &SmartRayApiError);
      MosPrintfA("SmartRay api failed with %s\n", SmartRayApiError);
      MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
      MosGetch();
      }

   // Release the SmartRay camera.
   ReleaseCamera();

   // Cleanup MIL objects.
   M3dmapFree(MilPointCloudContainer);
   MbufFree(MilDisplayLUT);
   MdispFree(MilDisplay);
   MsysFree(MilSystem);
   MappFree(MilApplication);
#endif

   return 0;
   }

#if SMARTRAY_INSTALLED
//*****************************************************************************
// Initializes the api and the sensor.
//*****************************************************************************
int InitCamera(MIL_ID MilSystem)
   {
   // Init SmartRay api.
   MosPrintf(MIL_TEXT("SmartRay: Initializing API...\n"));
   CHECK_SR(SR_API_Initalize(CSRAcquisition::ApiStatusCallback))

   // Register PIL image callback.
   CHECK_SR(SR_API_RegisterPilImageCB(CSRAcquisition::PilImageCallback))

   // Create the sensor.
   MosPrintf(MIL_TEXT("SmartRay: Creating sensor...\n"));
   SRAcq.Init(MilSystem, NB_PROFILES, SENSOR_NAME, 0, DEFAULT_IP_ADR, DEFAULT_PORT_NUM);

   // Connect the sensor.
   MosPrintf(MIL_TEXT("SmartRay: Connecting sensor...\n"));
   int timeoutS = 20;
   CHECK_SR(SR_API_ConnectSensor(SRAcq.GetSRSensor(), timeoutS))
   MosPrintf(MIL_TEXT("SmartRay: Sensor connected.\n"));

   // Get the sensor model and part number.
   char partNumber[16];
   char modelName[16];
   CHECK_SR(SR_API_GetSensorModelName(SRAcq.GetSRSensor(), modelName, partNumber))
   MosPrintfA("SmartRay: Sensor model name : %s\n"
              "SmartRay: Sensor part number: %s\n",
              modelName, partNumber);
      
   // Load the calibration data from the sensor.
   MosPrintf(MIL_TEXT("SmartRay: Loading calibration from the sensor...\n"));
   CHECK_SR(SR_API_LoadCalibrationDataFromSensor(SRAcq.GetSRSensor()))

   // Load the parameter set file based on the sensor series.
   MosPrintf(MIL_TEXT("SmartRay: Loading parameter set from file...\n"));
   char ParameterSetPath[MAX_PATH];
   MosSprintfA(ParameterSetPath, MAX_PATH, "%s\\SR_API\\sr_parameter_sets\\Pars_%s\\%s_3D_Snapshot.par",
               SMARTRAY_PATH, SENSOR_NAME, SENSOR_NAME);
   CHECK_SR(SR_API_LoadParameterSetFromFile(SRAcq.GetSRSensor(), ParameterSetPath))

   // Setup the acquisition.
   MosPrintf(MIL_TEXT("SmartRay: Acquisition setup...\n"));
   CHECK_SR(SR_API_SetImageAcquisitionType(SRAcq.GetSRSensor(), ImageAquisitionType_ProfileIntensityLaserLineThickness))
   CHECK_SR(SR_API_SetNumberOfProfilesToCapture(SRAcq.GetSRSensor(), NB_PROFILES))
   CHECK_SR(SR_API_SetExposureTime(SRAcq.GetSRSensor(), 0, EXPOSURE_TIME_1))
   CHECK_SR(SR_API_SetExposureTime(SRAcq.GetSRSensor(), 1, EXPOSURE_TIME_2))

   // Send the parameters to the sensor.
   MosPrintf(MIL_TEXT("SmartRay: Sending parameter set to sensor...\n"));
   CHECK_SR(SR_API_SendParameterSetToSensor(SRAcq.GetSRSensor()))

   return 0;
   }

//*****************************************************************************
// Grabs a point cloud.
//*****************************************************************************
int GrabPointCloud(MIL_ID MilPointCloudContainer, MIL_UINT *pNbValidPoints)
   {
   // Acquire the point cloud.
   MosPrintf(MIL_TEXT("SmartRay: Acquiring point cloud..."));
   CHECK_SR(SR_API_StartAcquisition(SRAcq.GetSRSensor()))
   SRAcq.WaitAcquisitionEnd();
   CHECK_SR(SR_API_StopAcquisition(SRAcq.GetSRSensor()))

   // Create a 3D point cloud from the profile image data.
   MosPrintf(MIL_TEXT("\nSmartRay: Creating point cloud...\n\n"));
   SR_3DPOINT* pointCloud = new SR_3DPOINT[SRAcq.NbProfileAdded() * SRAcq.ProfileWidth()];
   MIL_UINT16* pProfileData = (MIL_UINT16*) MbufInquire(SRAcq.MilProfileImage(), M_HOST_ADDRESS, M_NULL);
   CHECK_SR(SR_API_CreatePointCloudMultipleProfile(SRAcq.GetSRSensor(), pProfileData,
           (int)SRAcq.ProfileOriginX(), (int)SRAcq.ProfileWidth(),
           (int)SRAcq.NbProfileAdded(), pointCloud))
   SR_3DPOINT* pointCloudLine = pointCloud;
   for(int y = 0; y < SRAcq.NbProfileAdded(); y++)
      {
      for(int x = 0; x < SRAcq.ProfileWidth(); x++)
         {
         pointCloudLine[x].x = pointCloudLine[x].y;

         // Check if data is valid
         if(pointCloudLine[x].x > NONWORLDMARK)
            {
            pointCloudLine[x].y = y*CONVEYOR_SPEED;
            (*pNbValidPoints)++;
            }
         else
            {
            pointCloudLine[x].y = M_INVALID_POINT_FLOAT;
            }

         pointCloudLine[x].z = -pointCloudLine[x].z;
         }
      pointCloudLine += SRAcq.ProfileWidth();
      }

   // Put the point cloud in the MIL point cloud container.   
   MIL_INT NbPoints = SRAcq.NbProfileAdded() * SRAcq.ProfileWidth();
   M3dmapPut(MilPointCloudContainer, M_POINT_CLOUD_LABEL(1), M_XYZ, 32 + M_FLOAT, NbPoints * 3,
             (MIL_FLOAT*)pointCloud, M_NULL, M_NULL, M_NULL, M_DEFAULT);

   // Free the point cloud.
   delete[] pointCloud;

   // Put the intensity of the point cloud data.
   MIL_UINT16* pIntensityData = (MIL_UINT16*)MbufInquire(SRAcq.MilIntensityImage(), M_HOST_ADDRESS, M_NULL);
   M3dmapPut(MilPointCloudContainer, M_POINT_CLOUD_LABEL(1), M_INTENSITY, 16 + M_UNSIGNED, NbPoints,
             pIntensityData, M_NULL, M_NULL, M_NULL, M_DEFAULT);

   return SUCCESS;
   }

//*****************************************************************************
// Gets the extraction box based on the SmartRay ZMap information.
//*****************************************************************************
int GetDepthMapInformation(MIL_INT* pDepthMapWidth, MIL_DOUBLE* pPixelSizeX, MIL_DOUBLE* pBoxMinX, MIL_DOUBLE* pBoxMaxX)
   {
   // Get the ROI.
   int RoiX = 0;
   int RoiY = 0;
   int RoiWidth = 0;
   int RoiHeight = 0;
   CHECK_SR(SR_API_GetROI(SRAcq.GetSRSensor(), &RoiX, &RoiWidth, &RoiY, &RoiHeight))

   // Get the Zmap resolution information.
   float VerticalResolution;
   float LateralResolution;
   CHECK_SR(SR_API_GetZmapResolution(SRAcq.GetSRSensor(), &LateralResolution, &VerticalResolution))
   
   // Get the Zmap dimensions.
   unsigned int ZMapWidth;
   float ZRange;
   float StartYPos;
   float EndYPos;
   CHECK_SR(SR_API_GetZmapDimensions(SRAcq.GetSRSensor(), RoiX, RoiWidth, LateralResolution, VerticalResolution,
                                     &ZMapWidth, &ZRange, &StartYPos, &EndYPos))

   *pDepthMapWidth = ZMapWidth;
   *pPixelSizeX = LateralResolution;
   *pBoxMinX = StartYPos;
   *pBoxMaxX = EndYPos;

   return 0;
   }

//*****************************************************************************
// Releases the api and the sensor.
//*****************************************************************************
int ReleaseCamera()
   {
   // Disconnect and release the sensor.
   if(SRAcq.IsConnected())
      { SR_API_DisconnectSensor(SRAcq.GetSRSensor()); }

   // Exit the SmartRay api.
   SR_API_Exit();
   SRAcq.Release();

   return 0;
   }

#endif
