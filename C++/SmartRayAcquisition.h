/************************************************************************************/
/*
* File name: SmartRayAcquisition.h
*
* Synopsis:  This file contains the declaration of the CSmartRayAcquisition class that
*            handles the acquisition using a SmartRay 3d camera.
*
* Copyright © 1992-2024 Zebra Technologies Corp. and/or its affiliates
* All Rights Reserved
*/

#ifndef SMARTRAY_ACQUISITION_H
#define SMARTRAY_ACQUISITION_H

#define SMARTRAY_INSTALLED  0

#if SMARTRAY_INSTALLED

#include <sr_api_public.h>
#include <SR_API_errorcodes.h>

class CSRAcquisition
   {
   public:

      CSRAcquisition();
      virtual ~CSRAcquisition();

      // Allocation and release.
      void Init(MIL_ID MilSystem, MIL_INT NbTotalProfile, const char* name,
                int multiSensorIndex, const char* ipAddress, unsigned short port);
      void Release();
      
      // SmartRay status callback.
      static int ApiStatusCallback(SRSensor* sensorObject, MessageType msgType, SubMessageType subMsgType,
                                   int msgData, char *msg);
      
      // SmartRay grab callback.
      static int PilImageCallback(SRSensor *sensorObject, ImageDataType imageType, int originX, int height, int width,
                                  uint16_t* profileImage, uint16_t* intensityImage, uint16_t* lltImage,
                                  int numExtData, void *extData);
      void WaitAcquisitionEnd();

      // Accessors.
      SRSensor* GetSRSensor()             const { return m_SRSensor; }
      bool IsConnected()                  const { return m_IsConnected; }
      MIL_INT NbProfileAdded()            const { return m_NbProfileAdded; }
      MIL_INT ProfileWidth()              const { return m_ProfileWidth; }
      MIL_INT ProfileOriginX()            const { return m_ProfileOriginX; }
      MIL_ID MilProfileImage()            const { return m_MilProfileImage; }
      MIL_ID MilIntensityImage()          const { return m_MilIntensityImage; }
      MIL_ID MilLaserLineThicknessImage() const { return m_MilLaserLineThicknessImage; }

   private:

      // Effective callbacks functions.
      int ApiStatus(SRSensor* sensorObject, MessageType msgType, SubMessageType subMsgType, int msgData, char *msg);
      int AddPilImage(SRSensor *sensorObject, ImageDataType imageType, int originX, int height, int width,
                      uint16_t* profileImage, uint16_t* intensityImage, uint16_t* lltImage,
                      int numExtData, void *extData);

      MIL_ID m_MilSystem;
      MIL_ID m_MilProfileImage;
      MIL_ID m_MilIntensityImage;
      MIL_ID m_MilLaserLineThicknessImage;
      MIL_ID m_MilAcquisitionEndEvent;

      MIL_INT m_NbTotalProfile;
      MIL_INT m_NbProfileAdded;
      MIL_INT m_ProfileWidth;
      MIL_INT m_ProfileOriginX;

      SRSensor* m_SRSensor;
      bool m_IsConnected;
   };

extern CSRAcquisition SRAcq;

#endif // SMARTRAY_INSTALLED

#endif // SMARTRAY_ACQUISITION_H
