#!/usr/bin/env bash

rtconf RTCPCLOpenNI0.rtc set corba 0
rtconf RTCPCLViewer0.rtc set show_timestamps 1
rtconf RTCPCLVoxelFilter0.rtc set corba 0
rtconf RTCPCLVoxelFilter0.rtc set point_type xyzrgb
rtcon RTCPCLOpenNI0.rtc:dds_xyzrgb -p ddsport.topic=office
rtcon RTCPCLViewer0.rtc:dds_k1 -p ddsport.topic=office
rtcon RTCPCLViewer0.rtc:dds_k2 -p ddsport.topic=office_filtered
rtcon RTCPCLVoxelFilter0.rtc:dds_out -p ddsport.topic=office_filtered
rtcon RTCPCLVoxelFilter0.rtc:dds_in -p ddsport.topic=office
rtact RTCPCLViewer0.rtc
rtact RTCPCLVoxelFilter0.rtc
rtact RTCPCLOpenNI0.rtc

