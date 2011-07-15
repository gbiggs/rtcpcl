/* RTC:PCDLoad
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


#ifndef RTCPCDSAVE_H
#define RTCPCDSAVE_H

#include <rtcpcl/config.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud.hh>
#include <rtcpcl/pc_mgmt.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <string>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
    class RTCPCDSave
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCDSave(RTC::Manager* manager);
            ~RTCPCDSave();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            PointCloudTypes::PointCloud corba_in_;
            RTC::InPort<PointCloudTypes::PointCloud> corba_in_port_;
#if defined(DDS_SUPPORT)
            RTC::DDSSubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudSeq,
                PointCloudTypes_PointCloudDataReader> dds_in_port_;
#endif // defined(DDS_SUPPORT)

            bool corba_;
            bool dds_;
            bool pointer_;
            bool binary_;
            bool saved_;
            bool write_once_;
            std::string filename_;

            template<typename PointT>
            void save_port()
            {
                typename pcl::PointCloud<PointT>::Ptr
                    unpacked(new pcl::PointCloud<PointT>);
                pointcloud_to_pcl<PointT>(corba_in_, unpacked);
                pcl::io::savePCDFile<PointT>(filename_, *unpacked, binary_);
            }

#if defined(DDS_SUPPORT)
            template<typename PointT>
            void save_port(PointCloudTypes_PointCloud& cloud)
            {
                typename pcl::PointCloud<PointT>::Ptr
                    unpacked(new pcl::PointCloud<PointT>);
                pointcloud_to_pcl<PointT>(cloud, unpacked);
                pcl::io::savePCDFile<PointT>(filename_, *unpacked, binary_);
            }
#endif // defined(DDS_SUPPORT)
    };
}; // namespace RTCPCL


extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCDSAVE_H

