/* RTC:RIViewer
 *
 * Component source file.
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


#include "rtcriviewer.h"

#include <rtcpcl/pc_mgmt.h>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCRIViewer::RTCRIViewer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_ri_port_("corba_ri", corba_ri_),
#if defined(DDS_SUPPORT)
    dds_ri_port_("dds_ri",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true), unseen_to_max_(true)
{
}


RTCRIViewer::~RTCRIViewer()
{
}


RTC::ReturnCode_t RTCRIViewer::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("unseen_to_max", unseen_to_max_, "0");

    addInPort(corba_ri_port_.getName(), corba_ri_port_);
#if defined(DDS_SUPPORT)
    addDDSSubPort(dds_ri_port_.getName(), dds_ri_port_, *this);
    register_pc_type(dds_ri_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCRIViewer::onActivated(RTC::UniqueId ec_id)
{
    viewer_.reset(new pcl::visualization::RangeImageVisualizer("RTC:RIViewer"));
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCRIViewer::onDeactivated(RTC::UniqueId ec_id)
{
    viewer_.reset();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCRIViewer::onExecute(RTC::UniqueId ec_id)
{
    if (corba_)
    {
        if (corba_ri_port_.isNew())
        {
            corba_ri_port_.read();
            pcl::PointCloud<pcl::PointWithRange>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointWithRange>);
            pointcloud_to_pcl<pcl::PointWithRange>(corba_ri_, cloud);
            display_range_image(cloud);
        }
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        if (dds_ri_port_.read_with_loan(clouds, info))
        {
            pcl::PointCloud<pcl::PointWithRange>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointWithRange>);
            pointcloud_to_pcl<pcl::PointWithRange>(clouds[0], cloud);
            display_range_image(cloud);
            dds_ri_port_.return_loan(clouds, info);
        }
    }
#endif // defined(DDS_SUPPORT)
    pcl::visualization::ImageWidgetWX::spinOnce();
    if (!viewer_->isShown())
    {
        // Deactivate self
    }
    return RTC::RTC_OK;
}


void RTCRIViewer::display_range_image(
        pcl::PointCloud<pcl::PointWithRange>::Ptr cloud)
{
    pcl::RangeImage::Ptr ri(boost::static_pointer_cast<pcl::RangeImage>(cloud));
    if (unseen_to_max_)
    {
        ri->setUnseenToMaxRange();
    }
    viewer_->setRangeImage(*ri);
}


static const char* spec[] =
{
    "implementation_id", "RTCRIViewer",
    "type_name",         "RTCRIViewer",
    "description",       "Range image viewer component",
    "version",           "1.0",
    "vendor",            "Geoffrey Biggs, AIST",
    "category",          "PCL",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.corba", "1",
    "conf.default.dds", "1",
    "conf.default.pointer", "1",
    "conf.default.unseen_to_max", "0",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.unseen_to_max", "radio",
    "conf.__widget__.border_size", "spin",
    // Constraints
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCRIViewer>,
                RTC::Delete<RTCRIViewer>);
    }
};

