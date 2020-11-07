// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <arv.h>

#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <functional>
#include <memory>
#include <string>

#include <glib.h>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace std::chrono_literals;

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)
// typedef camera_aravis::CameraAravisConfig Config; //ビルド時に生成される

static gboolean SoftwareTrigger_callback (void *);

typedef struct
{
	const char *szName;
	const char *szTag;
	ArvDomNode *pNode;
	ArvDomNode *pNodeSibling;
} NODEEX;

typedef struct
{
	double ExposureTimeAbs;
	double Gain;
	int GainRaw;
	int FocusPos;
	int AcquisitionFrameRate;
} Config;

// Global variables -------------------------
struct global_s
{
    bool                                       bCancel;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    // image_transport::CameraPublisher           publisher;
	// MinimalPublisher                           publisher;
    // camera_info_manager::CameraInfoManager     *pCameraInfoManager;
    // sensor_msgs::CameraInfo                    camerainfo;
	camera_info_manager::CameraInfo            camerainfo;
    // Config                                     config;
    // Config                                     configMin;
    // Config                                     configMax;
    int                                        idSoftwareTriggerTimer;
    
    int                                        isImplementedAcquisitionFrameRate;
    int                                        isImplementedAcquisitionFrameRateEnable;
    int                                        isImplementedGain;
    int                                        isImplementedExposureTimeAbs;
    int                                        isImplementedExposureAuto;
    int                                        isImplementedGainAuto;
    int                                        isImplementedFocusPos;
    int                                        isImplementedTriggerSelector;
    int                                        isImplementedTriggerSource;
    int                                        isImplementedTriggerMode;
    int                                        isImplementedAcquisitionMode;
    int                                        isImplementedMtu;

    int                                        xRoi;
    int                                        yRoi;
    int                                        widthRoi;
    int                                        widthRoiMin;
    int                                        widthRoiMax;
    int                                        heightRoi;
    int                                        heightRoiMin;
    int                                        heightRoiMax;
    
    int                                        widthSensor;
    int                                        heightSensor;

    const char                                 *pszPixelformat;
    unsigned                                   nBytesPixel;
	std::shared_ptr<rclcpp::Node>              node;
    ArvCamera                                  *pCamera;
    ArvDevice                                  *pDevice;
    int                                        mtu;
    int                                        Acquire;
    const char                                 *keyAcquisitionFrameRate;
} global;

typedef struct 
{
    GMainLoop  *main_loop;
    int         nBuffers;	// Counter for Hz calculation.
} ApplicationData;
// ------------------------------------

// Conversions from integers to Arv types.
const char	*szBufferStatusFromInt[] = {
										"ARV_BUFFER_STATUS_SUCCESS",
										"ARV_BUFFER_STATUS_CLEARED",
										"ARV_BUFFER_STATUS_TIMEOUT",
										"ARV_BUFFER_STATUS_MISSING_PACKETS",
										"ARV_BUFFER_STATUS_WRONG_PACKET_ID",
										"ARV_BUFFER_STATUS_SIZE_MISMATCH",
										"ARV_BUFFER_STATUS_FILLING",
										"ARV_BUFFER_STATUS_ABORTED"
										};

static void set_cancel (int signal)
{
    global.bCancel = TRUE;
}

ArvGvStream *CreateStream(void)
{
	gboolean 		bAutoBuffer = FALSE;
	gboolean 		bPacketResend = TRUE;
	unsigned int 	timeoutPacket = 40; // milliseconds
	unsigned int 	timeoutFrameRetention = 200;

	
	ArvGvStream *pStream = (ArvGvStream *)arv_device_create_stream (global.pDevice, NULL, NULL);
	if (pStream)
	{
		ArvBuffer	*pBuffer;
		gint 		 nbytesPayload;


		if (!ARV_IS_GV_STREAM (pStream))
		RCLCPP_WARN ( global.node->get_logger(), "Stream is not a GV_STREAM");

		if (bAutoBuffer)
			g_object_set (pStream,
					      "socket-buffer",
						  ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
						  "socket-buffer-size", 0,
						  NULL);
		if (!bPacketResend)
			g_object_set (pStream,
					      "packet-resend",
						  bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
						  NULL);
		g_object_set (pStream,
				          "packet-timeout",
						  (unsigned) timeoutPacket * 1000,
						  "frame-retention", (unsigned) timeoutFrameRetention * 1000,
					  NULL);
	
		// Load up some buffers.
		nbytesPayload = arv_camera_get_payload (global.pCamera);
		for (int i=0; i<50; i++)
		{
			pBuffer = arv_buffer_new (nbytesPayload, NULL);
			arv_stream_push_buffer ((ArvStream *)pStream, pBuffer);
		}
	}
	return pStream;
} // CreateStream()


static void NewBuffer_callback (ArvStream *pStream, ApplicationData *pApplicationdata)
{
	static uint64_t  cm = 0L;	// Camera time prev
	uint64_t  		 cn = 0L;	// Camera time now

#ifdef TUNING			
	static uint64_t  rm = 0L;	// ROS time prev
#endif
	// uint64_t  		 rn = 0L;	// ROS time now
	rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);
	rclcpp::Time     rn;

	static uint64_t	 tm = 0L;	// Calculated image time prev
	uint64_t		 tn = 0L;	// Calculated image time now
		
	static int64_t   em = 0L;	// Error prev.
	int64_t  		 en = 0L;	// Error now between calculated image time and ROS time.
	int64_t  		 de = 0L;	// derivative.
	int64_t  		 ie = 0L;	// integral.
	int64_t			 u = 0L;	// Output of controller.
	
	int64_t			 kp1 = 0L;		// Fractional gains in integer form.
	int64_t			 kp2 = 1024L;
	int64_t			 kd1 = 0L;
	int64_t			 kd2 = 1024L;
	int64_t			 ki1 = -1L;		// A gentle pull toward zero.
	int64_t			 ki2 = 1024L;

	static uint32_t	 iFrame = 0;	// Frame counter.
    
	ArvBuffer		*pBuffer;

	
#ifdef TUNING			
	std_msgs::Int64  msgInt64;
	int 			 kp = 0;
	int 			 kd = 0;
	int 			 ki = 0;
    
	if (global.phNode->hasParam(ros::this_node::getName()+"/kp"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/kp", kp);
		kp1 = kp;
	}
	
	if (global.phNode->hasParam(ros::this_node::getName()+"/kd"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/kd", kd);
		kd1 = kd;
	}
	
	if (global.phNode->hasParam(ros::this_node::getName()+"/ki"))
	{
		global.phNode->getParam(ros::this_node::getName()+"/ki", ki);
		ki1 = ki;
	}
#endif
	
    pBuffer = arv_stream_try_pop_buffer (pStream);
    if (pBuffer != NULL) 
    {
        if (arv_buffer_get_status (pBuffer) == ARV_BUFFER_STATUS_SUCCESS) 
        {
			sensor_msgs::msg::Image msg;
			
        	pApplicationdata->nBuffers++;
			std::vector<uint8_t> this_data(pBuffer->priv->size);
			memcpy(&this_data[0], pBuffer->priv->data, pBuffer->priv->size);


			// Camera/ROS Timestamp coordination.
			//cn				= (uint64_t)pBuffer->timestamp_ns;				// Camera now
			cn              = arv_buffer_get_timestamp(pBuffer);
			// rn	 			= ros::Time::now().toNSec();					// ROS now
			// rn	 			= system_clock.now();
			
			// if (iFrame < 10)
			// {
			// 	cm = cn;
			// 	tm  = rn.now();
			// }
			
			// Control the error between the computed image timestamp and the ROS timestamp.
			// en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
			// de = en-em;
			// ie += en;
			// u = kp1*(en/kp2) + ki1*(ie/ki2) + kd1*(de/kd2);  // kp<0, ki<0, kd>0
			
			// Compute the new timestamp.
			tn = (uint64_t)((int64_t)tm + (int64_t)cn-(int64_t)cm + u);
#ifdef TUNING			
			RCLCPP_WARN ( global.node->get_logger(), "en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de, kp1*(en/kp2), ki1*(ie/ki2), kd1*(de/kd2), u);
			RCLCPP_WARN ( global.node->get_logger(), "cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn-cm, rn-rm, (int64_t)tn-(int64_t)tm, tn-rn);
			msgInt64.data = tn-rn; //cn-cm+tn-tm; //
			global.ppubInt64->publish(msgInt64);
			rm = rn;
#endif
			
			// Save prior values.
			cm = cn;
			tm = tn;
			em = en;
			
			// Construct the image message.
			// msg.header.stamp.fromNSec(tn);               //TBD
			// msg.header.seq = pBuffer->priv->frame_id;    //TBD
			// msg.header.frame_id = global.config.frame_id;
			msg.width = global.widthRoi;
			msg.height = global.heightRoi;
			msg.encoding = global.pszPixelformat;
			msg.step = msg.width * global.nBytesPixel;
			msg.data = this_data;

			// get current CameraInfo data
			// global.camerainfo = global.pCameraInfoManager->getCameraInfo();
			// global.camerainfo.header.stamp = msg.header.stamp;
			// global.camerainfo.header.seq = msg.header.seq;
			// global.camerainfo.header.frame_id = msg.header.frame_id;
			// global.camerainfo.width = global.widthRoi;
			// global.camerainfo.height = global.heightRoi;
			global.publisher->publish(msg);
			// global.publisher.Publish(msg);
				
        }
        else
        	RCLCPP_WARN ( global.node->get_logger(), "Frame error: %s", szBufferStatusFromInt[pBuffer->priv->status]);
        arv_stream_push_buffer (pStream, pBuffer);
        iFrame++;
    }
} // NewBuffer_callback()

static void ControlLost_callback (ArvGvDevice *pGvDevice)
{
    RCLCPP_ERROR ( global.node->get_logger(), "Control lost.");

    global.bCancel = TRUE;
}

static gboolean SoftwareTrigger_callback (void *pCamera)
{
	arv_device_execute_command (global.pDevice, "TriggerSoftware");

    return TRUE;
}


// PeriodicTask_callback()
// Check for termination, and spin for ROS.
static gboolean PeriodicTask_callback (void *applicationdata)
{
    ApplicationData *pData = (ApplicationData*)applicationdata;

    RCLCPP_INFO ( global.node->get_logger(), "Frame rate = %d Hz", pData->nBuffers);
    pData->nBuffers = 0;

    if (global.bCancel)
    {
        g_main_loop_quit (pData->main_loop);
        return FALSE;
    }

    // ros::spinOnce();
	rclcpp::spin_some(global.node);

    return TRUE;
} // PeriodicTask_callback()


int main(int argc, char * argv[])
{
    char   		*pszGuid = NULL;
    char    	 szGuid[512];
    int			 nInterfaces = 0;
    int			 nDevices = 0;
    int 		 i = 0;
	const char	*pkeyAcquisitionFrameRate[2] = {"AcquisitionFrameRate", "AcquisitionFrameRateAbs"};
    ArvGcNode	*pGcNode;
	GError		*error=NULL;

    global.bCancel = FALSE;
    // global.config = global.config.__getDefault__();
    global.idSoftwareTriggerTimer = 0;

	rclcpp::init(argc, argv);
	global.node = rclcpp::Node::make_shared("camera");

    // Print out some useful info.
    RCLCPP_INFO ( global.node->get_logger(), "Attached cameras:");
    arv_update_device_list();
    nInterfaces = arv_get_n_interfaces();
    RCLCPP_INFO ( global.node->get_logger(), "# Interfaces: %d", nInterfaces);

    nDevices = arv_get_n_devices();
    RCLCPP_INFO ( global.node->get_logger(), "# Devices: %d", nDevices);
    for (i=0; i<nDevices; i++)
    RCLCPP_INFO ( global.node->get_logger(), "Device%d: %s", i, arv_get_device_id(i));

    if (nDevices<=0) {
      RCLCPP_ERROR ( global.node->get_logger(), "No cameras detected.");
      return -1;
    } 

    // Get the camera gui from either the command-line or as a parameter.
    if (argc==2)
    {
        strcpy(szGuid, argv[1]);
        pszGuid = szGuid;
    }
    else
    {
		RCLCPP_INFO( global.node->get_logger(), "Not Implemented!");
		return -1;

        std::string		stGuid;

        // TODO:
        // global.phNode->get_parameter();
        //example guid:Basler-21237813
        // strcpy (szGuid, stGuid.c_str());
        // pszGuid = szGuid;
        pszGuid = "guid";
    }

    //Open the camera, and set it up.
    RCLCPP_INFO( global.node->get_logger(), "Opening: %s", pszGuid ? pszGuid : "(any)");
    while (1)
    {
      global.pCamera = arv_camera_new(pszGuid);
      if (global.pCamera)
          break;
      else
      {
          RCLCPP_WARN ( global.node->get_logger(), "Could not open camera %s.  Retrying...", pszGuid);
          rclcpp::Duration(1.0);
          // rclcpp::spinOnce();
      }
    }

    global.pDevice = arv_camera_get_device(global.pCamera);
    RCLCPP_INFO( global.node->get_logger(), "Opend: %s-%s",
                arv_device_get_string_feature_value (global.pDevice, "DeviceVendorName"), 
                arv_device_get_string_feature_value (global.pDevice, "DeviceID"));

    // See if some basic camera features exist;
    		// See if some basic camera features exist.
		pGcNode = arv_device_get_feature (global.pDevice, "AcquisitionMode");
		global.isImplementedAcquisitionMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GainRaw");
		global.isImplementedGain = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
		pGcNode = arv_device_get_feature (global.pDevice, "Gain");
		global.isImplementedGain |= ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "ExposureTimeAbs");
		global.isImplementedExposureTimeAbs = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "ExposureAuto");
		global.isImplementedExposureAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GainAuto");
		global.isImplementedGainAuto = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerSelector");
		global.isImplementedTriggerSelector = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerSource");
		global.isImplementedTriggerSource = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "TriggerMode");
		global.isImplementedTriggerMode = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "FocusPos");
		global.isImplementedFocusPos = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "GevSCPSPacketSize");
		global.isImplementedMtu = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

		pGcNode = arv_device_get_feature (global.pDevice, "AcquisitionFrameRateEnable");
		global.isImplementedAcquisitionFrameRateEnable = ARV_GC_FEATURE_NODE (pGcNode) ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;

    // Find the key name for framerate.
		global.keyAcquisitionFrameRate = NULL;
		for (i=0; i<2; i++)
		{
			pGcNode = arv_device_get_feature (global.pDevice, pkeyAcquisitionFrameRate[i]);
			global.isImplementedAcquisitionFrameRate = pGcNode ? arv_gc_feature_node_is_implemented (ARV_GC_FEATURE_NODE (pGcNode), &error) : FALSE;
			if (global.isImplementedAcquisitionFrameRate)
			{
				global.keyAcquisitionFrameRate = pkeyAcquisitionFrameRate[i];
				break;
			}
		}

    // Get parameter bounds.
		// arv_camera_get_exposure_time_bounds	(global.pCamera, &global.configMin.ExposureTimeAbs, &global.configMax.ExposureTimeAbs);
		// arv_camera_get_gain_bounds			(global.pCamera, &global.configMin.Gain, &global.configMax.Gain);
		// arv_camera_get_sensor_size			(global.pCamera, &global.widthSensor, &global.heightSensor);
		// arv_camera_get_width_bounds			(global.pCamera, &global.widthRoiMin, &global.widthRoiMax);
		// arv_camera_get_height_bounds		(global.pCamera, &global.heightRoiMin, &global.heightRoiMax);

		if (global.isImplementedFocusPos)
		{
			gint64 focusMin64, focusMax64;
			arv_device_get_integer_feature_bounds (global.pDevice, "FocusPos", &focusMin64, &focusMax64);
			// global.configMin.FocusPos = focusMin64;
			// global.configMax.FocusPos = focusMax64;
		}
		else
		{
			// global.configMin.FocusPos = 0;
			// global.configMax.FocusPos = 0;
		}

    // global.configMin.AcquisitionFrameRate =    0.0;
	// 	global.configMax.AcquisitionFrameRate = 1000.0;

    		// Initial camera settings.
		// if (global.isImplementedExposureTimeAbs)
		// 	arv_device_set_float_feature_value(global.pDevice, "ExposureTimeAbs", global.config.ExposureTimeAbs);
		// if (global.isImplementedGain)
		// 	arv_camera_set_gain(global.pCamera, global.config.Gain);
			//arv_device_set_integer_feature_value(global.pDevice, "GainRaw", global.config.GainRaw);
		if (global.isImplementedAcquisitionFrameRateEnable)
			arv_device_set_integer_feature_value(global.pDevice, "AcquisitionFrameRateEnable", 1);
		// if (global.isImplementedAcquisitionFrameRate)
		// 	arv_device_set_float_feature_value(global.pDevice, global.keyAcquisitionFrameRate, global.config.AcquisitionFrameRate);


		// Set up the triggering.
		if (global.isImplementedTriggerMode)
		{
			if (global.isImplementedTriggerSelector && global.isImplementedTriggerMode)
			{
				arv_device_set_string_feature_value(global.pDevice, "TriggerSelector", "AcquisitionStart");
				arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
				arv_device_set_string_feature_value(global.pDevice, "TriggerSelector", "FrameStart");
				arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
			}
		}

    // WriteCameraFeaturesFromRosparam ();

    // Start the camerainfo manager.
    // global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(rclcpp::NodeHandle(rclcpp::this_node::getName()), arv_device_get_string_feature_value (global.pDevice, "DeviceID"));

    // Start the dynamic_reconfigure server.
    // ROS2 Navigation2.


		// Get parameter current values.
		global.xRoi=0; global.yRoi=0; global.widthRoi=0; global.heightRoi=0;
		arv_camera_get_region (global.pCamera, &global.xRoi, &global.yRoi, &global.widthRoi, &global.heightRoi);
		// global.config.ExposureTimeAbs 	= global.isImplementedExposureTimeAbs ? arv_device_get_float_feature_value (global.pDevice, "ExposureTimeAbs") : 0;
		// global.config.Gain      		= global.isImplementedGain ? arv_camera_get_gain (global.pCamera) : 0.0;
		global.pszPixelformat   		= g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(global.pDevice, "PixelFormat")))->str;
		global.nBytesPixel      		= ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(arv_device_get_integer_feature_value(global.pDevice, "PixelFormat"));
		// global.config.FocusPos  		= global.isImplementedFocusPos ? arv_device_get_integer_feature_value (global.pDevice, "FocusPos") : 0;
		
		
		// Print information.
		RCLCPP_INFO ( global.node->get_logger(), "    Using Camera Configuration:");
		RCLCPP_INFO ( global.node->get_logger(), "    ---------------------------");
		RCLCPP_INFO ( global.node->get_logger(), "    Vendor name          = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceVendorName"));
		RCLCPP_INFO ( global.node->get_logger(), "    Model name           = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceModelName"));
		RCLCPP_INFO ( global.node->get_logger(), "    Device id            = %s", arv_device_get_string_feature_value (global.pDevice, "DeviceID"));
		RCLCPP_INFO ( global.node->get_logger(), "    Sensor width         = %d", global.widthSensor);
		RCLCPP_INFO ( global.node->get_logger(), "    Sensor height        = %d", global.heightSensor);
		RCLCPP_INFO ( global.node->get_logger(), "    ROI x,y,w,h          = %d, %d, %d, %d", global.xRoi, global.yRoi, global.widthRoi, global.heightRoi);
		RCLCPP_INFO ( global.node->get_logger(), "    Pixel format         = %s", global.pszPixelformat);
		RCLCPP_INFO ( global.node->get_logger(), "    BytesPerPixel        = %d", global.nBytesPixel);
		RCLCPP_INFO ( global.node->get_logger(), "    Acquisition Mode     = %s", global.isImplementedAcquisitionMode ? arv_device_get_string_feature_value (global.pDevice, "AcquisitionMode") : "(not implemented in camera)");
		RCLCPP_INFO ( global.node->get_logger(), "    Trigger Mode         = %s", global.isImplementedTriggerMode ? arv_device_get_string_feature_value (global.pDevice, "TriggerMode") : "(not implemented in camera)");
		RCLCPP_INFO ( global.node->get_logger(), "    Trigger Source       = %s", global.isImplementedTriggerSource ? arv_device_get_string_feature_value(global.pDevice, "TriggerSource") : "(not implemented in camera)");
		RCLCPP_INFO ( global.node->get_logger(), "    Can set FrameRate:     %s", global.isImplementedAcquisitionFrameRate ? "True" : "False");
		if (global.isImplementedAcquisitionFrameRate)
		{
			// global.config.AcquisitionFrameRate = arv_device_get_float_feature_value (global.pDevice, global.keyAcquisitionFrameRate);
			// RCLCPP_INFO ( global.node->get_logger(), "    AcquisitionFrameRate = %g hz", global.config.AcquisitionFrameRate);
		}

		RCLCPP_INFO ( global.node->get_logger(), "    Can set Exposure:      %s", global.isImplementedExposureTimeAbs ? "True" : "False");
		if (global.isImplementedExposureTimeAbs)
		{
			RCLCPP_INFO ( global.node->get_logger(), "    Can set ExposureAuto:  %s", global.isImplementedExposureAuto ? "True" : "False");
			// RCLCPP_INFO ( global.node->get_logger(), "    Exposure             = %g us in range [%g,%g]", global.config.ExposureTimeAbs, global.configMin.ExposureTimeAbs, global.configMax.ExposureTimeAbs);
		}

		RCLCPP_INFO ( global.node->get_logger(), "    Can set Gain:          %s", global.isImplementedGain ? "True" : "False");
		if (global.isImplementedGain)
		{
			RCLCPP_INFO ( global.node->get_logger(), "    Can set GainAuto:      %s", global.isImplementedGainAuto ? "True" : "False");
			// RCLCPP_INFO ( global.node->get_logger(), "    Gain                 = %f %% in range [%f,%f]", global.config.Gain, global.configMin.Gain, global.configMax.Gain);
		}

		RCLCPP_INFO ( global.node->get_logger(), "    Can set FocusPos:      %s", global.isImplementedFocusPos ? "True" : "False");

		if (global.isImplementedMtu)
			RCLCPP_INFO ( global.node->get_logger(), "    Network mtu          = %lu", arv_device_get_integer_feature_value(global.pDevice, "GevSCPSPacketSize"));

		RCLCPP_INFO ( global.node->get_logger(), "    ---------------------------");

        ArvGvStream *pStreamDummy;
		ArvGvStream *pStream;
		while (TRUE)
		{
			pStream = CreateStream();
			if (pStream)
				break;
			else
			{
				RCLCPP_WARN(  global.node->get_logger(), "Could not create image stream for %s.  Retrying...", pszGuid);
				rclcpp::Duration(1.0);
			    // rclcpp::spinOnce();
			}
		}

 		ApplicationData applicationdata;
		applicationdata.nBuffers=0;
		applicationdata.main_loop = 0;

        // Set up image_raw.
        // image_transport::ImageTransport		*pTransport = new image_transport::ImageTransport(*global.node.get()); //TBD
		// global.publisher = pTransport->advertiseCamera(global.node.get_name()+"/image_raw", 1);
		/*
		auto qos = rclcpp::QoS(
			rclcpp::QoSInitialization(
				history_policy_,
				depth_
			));
		qos.reliability(reliability_policy_);
		*/
		global.publisher = global.node->create_publisher<sensor_msgs::msg::Image>("image", rclcpp::QoS(10));

		// Connect signals with callbacks.
		
		g_signal_connect (pStream,        "new-buffer",   G_CALLBACK (NewBuffer_callback),   &applicationdata);
		g_signal_connect (global.pDevice, "control-lost", G_CALLBACK (ControlLost_callback), NULL);
		g_timeout_add_seconds (1, PeriodicTask_callback, &applicationdata);
		arv_stream_set_emit_signals ((ArvStream *)pStream, TRUE);


		void (*pSigintHandlerOld)(int);
		pSigintHandlerOld = signal (SIGINT, set_cancel);
        
		arv_device_execute_command (global.pDevice, "AcquisitionStart");

		applicationdata.main_loop = g_main_loop_new (NULL, FALSE);
		g_main_loop_run (applicationdata.main_loop);

		if (global.idSoftwareTriggerTimer)
		{
			g_source_remove(global.idSoftwareTriggerTimer);
			global.idSoftwareTriggerTimer = 0;
		}

		signal (SIGINT, pSigintHandlerOld);

		g_main_loop_unref (applicationdata.main_loop);

		guint64 n_completed_buffers;
		guint64 n_failures;
		guint64 n_underruns;
		guint64 n_resent;
		guint64 n_missing;
		arv_stream_get_statistics ((ArvStream *)pStream, &n_completed_buffers, &n_failures, &n_underruns);
		RCLCPP_INFO ( global.node->get_logger(), "Completed buffers = %Lu", (unsigned long long) n_completed_buffers);
		RCLCPP_INFO ( global.node->get_logger(), "Failures          = %Lu", (unsigned long long) n_failures);
		RCLCPP_INFO ( global.node->get_logger(), "Underruns         = %Lu", (unsigned long long) n_underruns);
		arv_gv_stream_get_statistics (pStream, &n_resent, &n_missing);
		RCLCPP_INFO ( global.node->get_logger(), "Resent buffers    = %Lu", (unsigned long long) n_resent);
		RCLCPP_INFO ( global.node->get_logger(), "Missing           = %Lu", (unsigned long long) n_missing);

		arv_device_execute_command (global.pDevice, "AcquisitionStop");

		g_object_unref (pStream);

//   auto node = rclcpp::Node::make_shared("talker");
//   rclcpp::spin(std::make_shared<MinimalPublisher>());

    rclcpp::shutdown();

    return 0;
}
