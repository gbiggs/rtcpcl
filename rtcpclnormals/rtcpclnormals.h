/* RTC:PCLNormals
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


#ifndef RTCPCLNORMALS_H_
#define RTCPCLNORMALS_H_

#include <rtcpcl/config.h>
#include <rtcpcl/pc_mgmt.h>
#include <rtcpcl/pc_traits.h>

#include <boost/any.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <rtm/OutPort.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud.hh>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#include <rtmdds/ddspubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
    class RTCPCLNormals
        : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCLNormals(RTC::Manager* manager);
            virtual ~RTCPCLNormals();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            PointCloudTypes::PointCloud corba_in_;
            RTC::InPort<PointCloudTypes::PointCloud> corba_in_port_;
            PointCloudTypes::PointCloud corba_out_;
            RTC::OutPort<PointCloudTypes::PointCloud> corba_out_port_;
#if defined(DDS_SUPPORT)
            PointCloudTypes_PointCloud* dds_out_;
            RTC::DDSSubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudSeq,
                PointCloudTypes_PointCloudDataReader> dds_in_port_;
            RTC::DDSPubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudDataWriter> dds_out_port_;
#endif // defined(DDS_SUPPORT)

            bool corba_;
            bool dds_;
            bool pointer_;
            double radius_;

            boost::shared_ptr<pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> > estimator_;

            void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud);
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCLNORMALS_H_

