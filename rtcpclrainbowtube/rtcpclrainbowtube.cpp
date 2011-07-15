/* RTC:PCLRainbowTube
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


#include "rtcpclrainbowtube.h"

#include <ctime>
#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <rtcpcl/pc_mgmt.h>
#include <stdlib.h>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLRainbowTube::RTCPCLRainbowTube(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_out_port_("corba_out", corba_out_),
#if defined(DDS_SUPPORT)
    dds_out_port_("dds_out",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    seq_(0), corba_(true), dds_(true), pointer_(true), step_(0.1),
    angle_step_(0.08726646259971647),
    x_radius_(0.5), y_radius_(0.5), length_(1.0)
{
}


RTCPCLRainbowTube::~RTCPCLRainbowTube()
{
}


RTC::ReturnCode_t RTCPCLRainbowTube::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("step", step_, "0.1");
    bindParameter("angle_step", angle_step_, "0.08726646259971647");
    bindParameter("x_radius", x_radius_, "0.5");
    bindParameter("y_radius", y_radius_, "0.5");
    bindParameter("length", length_, "1.0");
    addOutPort(corba_out_port_.getName(), corba_out_port_);
#if defined(DDS_SUPPORT)
    addDDSPubPort(dds_out_port_.getName(), dds_out_port_, *this);
    register_pc_type(dds_out_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLRainbowTube::onActivated(RTC::UniqueId ec_id)
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
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    double steps(length_ / step_);
    uint8_t r(255), g(0), b(0), colour_step(255 / steps);
    for (float z(-length_ / 2.0); z <= length_ / 2.0; z += step_)
    {
        for (float angle(0.0); angle <= 2 * M_PI; angle += angle_step_)
        {
            pcl::PointXYZRGB point;
            point.x = x_radius_ * cosf(angle);
            point.y = y_radius_ * sinf(angle);
            point.z = z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            cloud_->points.push_back(point);
        }
        if (z < 0.0)
        {
            r -= colour_step;
            g += colour_step;
            b = 0;
        }
        else
        {
            r = 0;
            g -= colour_step;
            b += colour_step;
        }
    }
    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLRainbowTube::onDeactivated(RTC::UniqueId ec_id)
{
    cloud_.reset();
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_out_);
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLRainbowTube::onExecute(RTC::UniqueId ec_id)
{
    if (corba_)
    {
        pcl_to_pointcloud<pcl::PointXYZRGB>(cloud_, corba_out_);
        corba_out_port_.write();
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        pcl_to_pointcloud<pcl::PointXYZRGB>(cloud_, *dds_out_, true);
        dds_out_port_.write(*dds_out_);
        return_loan(*dds_out_);
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLRainbowTube",
    "type_name",         "RTCPCLRainbowTube",
    "description",       "Generate a rainbow-coloured tube",
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
    "conf.default.step", "0.1",
    "conf.default.angle_step", "0.08726646259971647",
    "conf.default.x_radius", "0.5",
    "conf.default.y_radius", "0.5",
    "conf.default.length", "1.0",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.step", "spin",
    "conf.__widget__.angle_step", "spin",
    "conf.__widget__.x_radius", "spin",
    "conf.__widget__.y_radius", "spin",
    "conf.__widget__.length", "spin",
    // Constraints
    "conf.__constraints__.step", "0<=x",
    "conf.__constraints__.length", "0<=x",
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLRainbowTube>,
                RTC::Delete<RTCPCLRainbowTube>);
    }
};

