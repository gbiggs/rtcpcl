/* RTC:RIViewer
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


#ifndef RTCRIVIEWER_H_
#define RTCRIVIEWER_H_

#include <rtcpcl/config.h>

#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pointcloud.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{

    class RTCRIViewer
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCRIViewer(RTC::Manager* manager);
            ~RTCRIViewer();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            PointCloudTypes::PointCloud corba_ri_;
            RTC::InPort<PointCloudTypes::PointCloud> corba_ri_port_;
#if defined(DDS_SUPPORT)
            RTC::DDSSubPort<PointCloudTypes_PointCloud,
                PointCloudTypes_PointCloudSeq,
                PointCloudTypes_PointCloudDataReader> dds_ri_port_;
#endif // defined(DDS_SUPPORT)

            boost::shared_ptr<pcl::visualization::RangeImageVisualizer> viewer_;

            bool corba_;
            bool dds_;
            bool pointer_;
            bool unseen_to_max_;

            void display_range_image(
                    pcl::PointCloud<pcl::PointWithRange>::Ptr cloud);
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCRIVIEWER_H_

