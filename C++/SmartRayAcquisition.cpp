/************************************************************************************/
/*
* File name: SmartRayAcquisition.cpp
*
* Synopsis:  This file contains the implementation of the CSmartRayAcquisition class that
*            handles the acquisition using a SmartRay 3d camera.
*
* Copyright © 1992-2024 Zebra Technologies Corp. and/or its affiliates
* All Rights Reserved
*/

#include <mil.h>
#include "SmartRayAcquisition.h"

#if SMARTRAY_INSTALLED
#include <SR_API_public.h>

CSRAcquisition SRAcq;

//*****************************************************************************
// Constructor. Initialize members.
//*****************************************************************************
CSRAcquisition::CSRAcquisition()
   : m_IsConnected(false),
     m_MilProfileImage(M_NULL),
     m_MilIntensityImage(M_NULL),
     m_MilLaserLineThicknessImage(M_NULL),
     m_MilAcquisitionEndEvent(M_NULL),
     m_NbProfileAdded(0),
     m_ProfileWidth(0),
     m_NbTotalProfile(0),
     m_SRSensor(NULL)
   {
   };

//*****************************************************************************
// Destructor.
//*****************************************************************************
CSRAcquisition::~CSRAcquisition()
   {
   };

//*****************************************************************************
// Initialize the SRSensor structure and allocate the end event.
//*****************************************************************************
void CSRAcquisition::Init(MIL_ID MilSystem, MIL_INT NbTotalProfile, const char* name,
                          int multiSensorIndex, const char* ipAddress, unsigned short port)
   {
   m_SRSensor = new SRSensor();

   // Sensor index.
   m_SRSensor->cam_index = multiSensorIndex;

   // Sensor name.
   strcpy_s(m_SRSensor->name, name);

   // Sensor IP address (can be changed ATTENTION!!!! Don't forget new IPAddress!).
   strcpy_s(m_SRSensor->IPAdr, ipAddress);

   // Sensor IP port number (can be changed ATTENTION!!!! Don't forget new IPAddress!). 
   m_SRSensor->portnum = port;

   // Allocate the event.
   MthrAlloc(MilSystem, M_EVENT, M_NOT_SIGNALED + M_AUTO_RESET, M_NULL, M_NULL, &m_MilAcquisitionEndEvent);

   m_MilSystem = MilSystem;
   m_NbTotalProfile = NbTotalProfile;
   }

//*****************************************************************************
// Free SR Sensor and MIL objects.
//*****************************************************************************
void CSRAcquisition::Release()
   {
   if(m_MilLaserLineThicknessImage != M_NULL)
      {
      MbufFree(m_MilLaserLineThicknessImage);
      m_MilLaserLineThicknessImage = M_NULL;
      }

   if(m_MilIntensityImage != M_NULL)
      {
      MbufFree(m_MilIntensityImage);
      m_MilIntensityImage = M_NULL;
      }

   if(m_MilProfileImage != M_NULL)
      {
      MbufFree(m_MilProfileImage);
      m_MilProfileImage = M_NULL;
      }

   if(m_MilAcquisitionEndEvent != M_NULL)
      {
      MthrFree(m_MilAcquisitionEndEvent);
      m_MilAcquisitionEndEvent = M_NULL;
      }

   if(m_SRSensor != NULL)
      {
      delete m_SRSensor;
      m_SRSensor = NULL;
      }
   }

//*****************************************************************************
// Static callback that deals with the status of the camera.
//*****************************************************************************
int CSRAcquisition::ApiStatusCallback(SRSensor* sensorObject, MessageType msgType, SubMessageType subMsgType,
                                      int msgData, char *msg)
   {
   return SRAcq.ApiStatus(sensorObject, msgType, subMsgType, msgData, msg);
   }

//*****************************************************************************
// Deals with the status of the camera.
//*****************************************************************************
int CSRAcquisition::ApiStatus(SRSensor* sensorObject, MessageType msgType, SubMessageType subMsgType,
                              int msgData, char *msg)
   {
   // Check for message data available.
   if(msg == NULL)
      return -1;

   // Handle Ethernet connection messages.
   if(msgType == MessageType_Connection)
      m_IsConnected = subMsgType == SubMessageType_Connection_SensorConnected;
   return 0;
   }

//*****************************************************************************
// Static callback called after that acquisition of PIL images.
//*****************************************************************************
int CSRAcquisition::PilImageCallback(SRSensor *sensorObject, ImageDataType imageType, int originX, int height, int width,
                                     uint16_t* profileImage, uint16_t* intensityImage, uint16_t* lltImage,
                                     int numExtData, void *extData)
   {
   return SRAcq.AddPilImage(sensorObject, imageType, originX, height, width,
                            profileImage, intensityImage, lltImage, numExtData, extData);
   }

//*****************************************************************************
// Adds the PIL packet to the grabbed profile image. Allocates the MIL images.
//*****************************************************************************
int CSRAcquisition::AddPilImage(SRSensor *sensorObject, ImageDataType imageType, int originX, int height, int width,
                                uint16_t* profileImage, uint16_t* intensityImage, uint16_t* lltImage,
                                int numExtData, void *extData)
   {
   if(m_NbProfileAdded >= m_NbTotalProfile)
      m_NbProfileAdded = 0;

   if(!m_MilProfileImage)
      {
      MbufAlloc1d(m_MilSystem, m_NbTotalProfile * width, 16 + M_UNSIGNED, M_IMAGE, &m_MilProfileImage);
      MbufAlloc1d(m_MilSystem, m_NbTotalProfile * width, 16 + M_UNSIGNED, M_IMAGE, &m_MilIntensityImage);
      MbufAlloc2d(m_MilSystem, width, m_NbTotalProfile, 16 + M_UNSIGNED, M_IMAGE, &m_MilLaserLineThicknessImage);
      m_ProfileWidth = width;
      m_ProfileOriginX = originX;
      }
   switch(imageType)
      {
      case ImageDataType_Profile:
         MbufPut1d(m_MilProfileImage, m_NbProfileAdded*width, width*height, profileImage);
         break;

      case ImageDataType_Intensity:
         MbufPut1d(m_MilIntensityImage, m_NbProfileAdded*width, width*height, intensityImage);
         break;

      case ImageDataType_ProfileIntensity:
         MbufPut1d(m_MilProfileImage, m_NbProfileAdded*width, width*height, profileImage);
         MbufPut1d(m_MilIntensityImage, m_NbProfileAdded*width, width*height, intensityImage);
         break;

      case ImageDataType_ProfileIntensityLaserLineThickness:
         MbufPut1d(m_MilProfileImage, m_NbProfileAdded*width, width*height, profileImage);
         MbufPut1d(m_MilIntensityImage, m_NbProfileAdded*width, width*height, intensityImage);
         MbufPut2d(m_MilLaserLineThicknessImage, 0, m_NbProfileAdded, width, height, intensityImage);
         break;

      default:
         break;
      }

   m_NbProfileAdded += height;

   if(m_NbProfileAdded >= m_NbTotalProfile)
      MthrControl(m_MilAcquisitionEndEvent, M_EVENT_SET, M_SIGNALED);

   MosPrintf(MIL_TEXT("\rSmartRay: Acquiring point cloud... %.2f%%"),
             (100.0*m_NbProfileAdded)/m_NbTotalProfile);

   return 0;
   }

//*****************************************************************************
// Function that waits for the end event of an acquisition.
//*****************************************************************************
void CSRAcquisition::WaitAcquisitionEnd()
   {
   MthrWait(m_MilAcquisitionEndEvent, M_EVENT_WAIT, M_NULL);
   }

#endif
