/* RTC:PCDBenchmark
 *
 * Component header file.
 *
 * Copyright 2011 Geoffrey Biggs geoffrey.biggs@aist.go.jp
 *     RT-Synthesis Research Group
 *     Intelligent Systems Research Institute,
 *     National Institute of Advanced Industrial Science and Technology (AIST),
 *     Japan
 *     All rights reserved.
 *
 * This file is part of RTC:PCL.
 *
 * RTC:PCL is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License,
 * or (at your option) any later version.
 *
 * RTC:PCL is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with RTM:PCL. If not, see
 * <http://www.gnu.org/licenses/>.
 */


#ifndef RTCPCDBENCHMARK_H
#define RTCPCDBENCHMARK_H

#include <rtcpcl/config.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <rtm/OutPort.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud.hh>
#include <string>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#include <rtmdds/ddspubport.h>
#endif // defined(DDS_SUPPORT)

#include <fstream>
#include <iostream>

namespace RTCPCL
{

    class RTCPCDBenchmark
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCDBenchmark(RTC::Manager* manager);
            ~RTCPCDBenchmark();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            typedef pcl::PointXYZ PCLPointType;
            typedef pcl::PointCloud<PCLPointType> PCLCloudType;

            PointCloudTypes::PointCloud in_cloud_;
            RTC::InPort<PointCloudTypes::PointCloud> corba_in_port_;
            PointCloudTypes::PointCloud out_cloud_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_out_port_;
#if defined(DDS_SUPPORT)
            PointCloudTypes_PointCloud* dds_cloud_;
            RTC::DDSSubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudSeq,
                PointCloudTypes_PointCloudDataReader> dds_in_port_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_out_port_;
#endif // defined(DDS_SUPPORT)
            PCLCloudType::Ptr loaded_cloud_;

            unsigned int seq_;
            std::string filename_;

            bool corba_;
            bool dds_;
            bool pointer_;
            bool write_;
            bool read_;
            bool echo_;
            bool echo_fast_;
            std::ofstream write_times_;
            std::ofstream read_times_;
            std::ofstream echo_times_;
            uint64_t start_time_;
            uint64_t end_time_;
            uint64_t echoed_clouds_;
            uint64_t echoed_points_;
            uint64_t echoed_bytes_;
            uint64_t written_clouds_;
            uint64_t written_points_;
            uint64_t written_bytes_;
            uint64_t read_clouds_;
            uint64_t read_points_;
            uint64_t read_bytes_;

            RTC::ReturnCode_t do_corba_latency();
            RTC::ReturnCode_t do_dds_latency();
            RTC::ReturnCode_t do_pointer_latency();
            uint64_t get_ts();
            uint64_t write_ts(std::ofstream& dest, char suffix);
    };

}; // namespace RTCPCL


extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCDBENCHMARK_H

