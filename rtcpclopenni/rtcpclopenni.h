/* RTC:PCLOPENNI
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
 * License along with RTC:PCL. If not, see
 * <http://www.gnu.org/licenses/>.
 */


#ifndef RTCPCLOPENNI_H
#define RTCPCLOPENNI_H

#include <rtcpcl/config.h>

#include <boost/shared_ptr.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/OutPort.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pointcloud.hh>
#include <string>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddspubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
    class RTCPCLOpenNI
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCLOpenNI(RTC::Manager* manager);
            ~RTCPCLOpenNI();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

        private:
            PointCloudTypes::PointCloud corba_xyz_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_xyz_port_;
            PointCloudTypes::PointCloud corba_xyzrgb_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_xyzrgb_port_;
            PointCloudTypes::PointCloud corba_ri_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_ri_port_;
#if defined(DDS_SUPPORT)
            PointCloudTypes_PointCloud* dds_xyz_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_xyz_port_;
            PointCloudTypes_PointCloud* dds_xyzrgb_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_xyzrgb_port_;
            PointCloudTypes_PointCloud* dds_ri_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_ri_port_;
#endif // defined(DDS_SUPPORT)

            bool corba_;
            bool dds_;
            bool pointer_;
            std::string device_;
            bool output_xyz_;
            bool output_xyzrgb_;
            bool output_ri_;

            boost::shared_ptr<pcl::Grabber> grabber_;
            unsigned int seq_;

            void xyz_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud);
            void xyzrgb_cb(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const& cloud);
            void make_range_image(pcl::PointCloud<pcl::PointWithRange>::Ptr& ri,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud);
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCLOPENNI_H

