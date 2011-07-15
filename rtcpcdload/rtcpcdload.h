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


#ifndef RTCPCDLOAD_H
#define RTCPCDLOAD_H

#include <rtcpcl/config.h>
#include <rtcpcl/pc_mgmt.h>

#include <boost/any.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/OutPort.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud.hh>
#include <string>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddspubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{

    class RTCPCDLoad
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCDLoad(RTC::Manager* manager);
            ~RTCPCDLoad();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            PointCloudTypes::PointCloud corba_out_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_out_port_;
#if defined(DDS_SUPPORT)
            PointCloudTypes_PointCloud* dds_out_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_out_port_;
#endif // defined(DDS_SUPPORT)
            boost::any loaded_cloud_;

            bool corba_;
            bool dds_;
            bool pointer_;
            bool written_;
            bool write_once_;
            unsigned int seq_;
            std::string filename_;
            std::string point_type_;

            RTC::ReturnCode_t load_cloud();
            RTC::ReturnCode_t destroy_cloud();

            template<typename PointT>
            void set_packed_clouds(typename pcl::PointCloud<PointT>::Ptr cloud)
            {
                pcl_to_pointcloud<PointT>(cloud, corba_out_);
#if defined(DDS_SUPPORT)
                pcl_to_pointcloud<PointT>(cloud, *dds_out_, true);
#endif // defined(DDS_SUPPORT)
            }
            void destroy_packed_clouds();
    };

}; // namespace RTCPCL


extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCDLOAD_H

