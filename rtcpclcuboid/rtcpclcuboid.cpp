/* RTC:PCDLoad
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


#include "rtcpclcuboid.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <rtcpcl/pc_mgmt.h>
#include <stdlib.h>
#include <ctime>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLCuboid::RTCPCLCuboid(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_out_port_("corba_out", corba_out_),
#if defined(DDS_SUPPORT)
    dds_out_port_("dds_out",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    seq_(0), corba_(true), dds_(true), pointer_(true), count_(1000),
    low_x_(-1.0), high_x_(1.0), low_y_(-1.0), high_y_(1.0), low_z_(-1.0),
    high_z_(1.0)
{
}


RTCPCLCuboid::~RTCPCLCuboid()
{
}


RTC::ReturnCode_t RTCPCLCuboid::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("count", count_, "1000");
    bindParameter("low_x", low_x_, "-1.0");
    bindParameter("high_x", high_x_, "1.0");
    bindParameter("low_y", low_y_, "-1.0");
    bindParameter("high_y", high_y_, "1.0");
    bindParameter("low_z", low_z_, "-1.0");
    bindParameter("high_z", high_z_, "1.0");
    addOutPort(corba_out_port_.getName(), corba_out_port_);
#if defined(DDS_SUPPORT)
    addDDSPubPort(dds_out_port_.getName(), dds_out_port_, *this);
    register_pc_type(dds_out_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLCuboid::onActivated(RTC::UniqueId ec_id)
{
    seq_ = 0;
#if defined(DDS_SUPPORT)
    dds_out_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_out_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_out_->data.maximum(0);
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLCuboid::onDeactivated(RTC::UniqueId ec_id)
{
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_out_);
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLCuboid::onExecute(RTC::UniqueId ec_id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        rand_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    rand_cloud->header.stamp = get_ts();
    rand_cloud->header.seq = seq_++;
    rand_cloud->height = 1;
    rand_cloud->width = count_;
    rand_cloud->is_dense = true;
    rand_cloud->points.resize(rand_cloud->height * rand_cloud->width);
    for (size_t ii(0); ii < rand_cloud->points.size(); ++ii)
    {
        if (high_x_ == low_x_)
        {
            // X range is zero so all points in the X plane
            rand_cloud->points[ii].x = high_x_;
        }
        else
        {
            // Random point between low_x_ and high_x_
            rand_cloud->points[ii].x =
                (static_cast<float>(rand()) / RAND_MAX) * (high_x_ - low_x_) + low_x_;
        }
        if (high_y_ == low_y_)
        {
            // Y range is zero so all points in the Y plane
            rand_cloud->points[ii].y = high_y_;
        }
        else
        {
            // Random point between low_y_ and high_y_
            rand_cloud->points[ii].y =
                (static_cast<float>(rand()) / RAND_MAX) * (high_y_ - low_y_) + low_y_;
        }
        if (high_z_ == low_z_)
        {
            // Z range is zero so all points in the Z plane
            rand_cloud->points[ii].z = high_z_;
        }
        else
        {
            // Random point between low_z_ and high_z_
            rand_cloud->points[ii].z =
                (static_cast<float>(rand()) / RAND_MAX) * (high_z_ - low_z_) + low_z_;
        }
    }

    if (corba_)
    {
        pcl_to_pointcloud<pcl::PointXYZ>(rand_cloud, corba_out_);
        corba_out_port_.write();
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        pcl_to_pointcloud<pcl::PointXYZ>(rand_cloud, *dds_out_, true);
        dds_out_port_.write(*dds_out_);
        return_loan(*dds_out_);
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLCuboid",
    "type_name",         "RTCPCLCuboid",
    "description",       "Random cuboid of points generator",
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
    "conf.default.dds", "0",
    "conf.default.pointer", "0",
    "conf.default.count", "1000",
    "conf.default.low_x", "-1.0",
    "conf.default.high_x", "1.0",
    "conf.default.low_y", "-1.0",
    "conf.default.high_y", "1.0",
    "conf.default.low_z", "-1.0",
    "conf.default.high_z", "1.0",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.count", "spin",
    "conf.__widget__.low_x", "spin",
    "conf.__widget__.high_x", "spin",
    "conf.__widget__.low_y", "spin",
    "conf.__widget__.high_y", "spin",
    "conf.__widget__.low_z", "spin",
    "conf.__widget__.high_z", "spin",
    // Constraints
    "conf.__constraints__.count", "0<=x",
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLCuboid>,
                RTC::Delete<RTCPCLCuboid>);
    }
};

