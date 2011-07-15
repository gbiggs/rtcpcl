/* RTC:PCLOpenNI
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


#include "rtcpclopenni.h"

#include <pcl/io/openni_grabber.h>
#include <rtcpcl/pc_mgmt.h>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLOpenNI::RTCPCLOpenNI(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_xyz_port_("corba_xyz", corba_xyz_),
    corba_xyzrgb_port_("corba_xyzrgb", corba_xyzrgb_),
    corba_ri_port_("corba_ri", corba_ri_),
#if defined(DDS_SUPPORT)
    dds_xyz_(0),
    dds_xyz_port_("dds_xyz",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_xyzrgb_(0),
    dds_xyzrgb_port_("dds_xyzrgb",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_ri_(0),
    dds_ri_port_("dds_ri",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true), device_("#1"),
    output_xyz_(false), output_xyzrgb_(true), output_ri_(false), seq_(0)
{
}


RTCPCLOpenNI::~RTCPCLOpenNI()
{
}


RTC::ReturnCode_t RTCPCLOpenNI::onInitialize()
{
    bindParameter("device", device_, "#1");
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("output_xyz", output_xyz_, "0");
    bindParameter("output_xyzrgb", output_xyzrgb_, "1");
    bindParameter("output_range_image", output_ri_, "0");

    addOutPort(corba_xyz_port_.getName(), corba_xyz_port_);
    addOutPort(corba_xyzrgb_port_.getName(), corba_xyzrgb_port_);
    addOutPort(corba_ri_port_.getName(), corba_ri_port_);
#if defined(DDS_SUPPORT)
    addDDSPubPort(dds_xyz_port_.getName(), dds_xyz_port_, *this);
    register_pc_type(dds_xyz_port_.get_participant());
    addDDSPubPort(dds_xyzrgb_port_.getName(), dds_xyzrgb_port_, *this);
    register_pc_type(dds_xyzrgb_port_.get_participant());
    addDDSPubPort(dds_ri_port_.getName(), dds_ri_port_, *this);
    register_pc_type(dds_ri_port_.get_participant());
#endif // defined(DDS_SUPPORT)

    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLOpenNI::onActivated(RTC::UniqueId ec_id)
{
    seq_ = 0;

#if defined(DDS_SUPPORT)
    dds_xyz_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_xyz_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_xyz_->data.maximum(0);
    dds_xyzrgb_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_xyzrgb_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_xyzrgb_->data.maximum(0);
    dds_ri_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_ri_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_ri_->data.maximum(0);
#endif // defined(DDS_SUPPORT)
    grabber_.reset(new pcl::OpenNIGrabber(device_));
    if (output_xyz_ || output_ri_)
    {
        boost::function<void (pcl::PointCloud<pcl::PointXYZ>::ConstPtr const&)> f =
            boost::bind(&RTCPCLOpenNI::xyz_cb, this, _1);
        grabber_->registerCallback(f);
    }
    if (output_xyzrgb_)
    {
        boost::function<void (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const&)> f =
            boost::bind(&RTCPCLOpenNI::xyzrgb_cb, this, _1);
        grabber_->registerCallback(f);
    }
    grabber_->start();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLOpenNI::onDeactivated(RTC::UniqueId ec_id)
{
    grabber_->stop();
    grabber_.reset();
#if defined(DDS_SUPPORT)
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_xyz_);
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_xyzrgb_);
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_ri_);
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


void RTCPCLOpenNI::xyz_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
{
    uint64_t ts(get_ts());
    if (corba_)
    {
        if (output_xyz_)
        {
            pcl_to_pointcloud<pcl::PointXYZ>(cloud, corba_xyz_);
            corba_xyz_.seq = seq_;
            corba_xyz_.tm.sec = ts / 1000000000;
            corba_xyz_.tm.nsec = ts % 1000000000;
            corba_xyz_port_.write();
        }
        if (output_ri_)
        {
            pcl::PointCloud<pcl::PointWithRange>::Ptr ri(
                    new pcl::PointCloud<pcl::PointWithRange>);
            make_range_image(ri, cloud); // Using the raw depth image leads to segfaults
            pcl_to_pointcloud<pcl::PointWithRange>(ri, corba_ri_);
            corba_ri_.seq = seq_;
            corba_ri_.tm.sec = ts / 1000000000;
            corba_ri_.tm.nsec = ts % 1000000000;
            corba_ri_port_.write();
        }
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        if (output_xyz_)
        {
            pcl_to_pointcloud<pcl::PointXYZ>(cloud, *dds_xyz_, true);
            dds_xyz_->seq = seq_;
            dds_xyz_->tm.sec = ts / 1000000000;
            dds_xyz_->tm.nsec = ts % 1000000000;
            dds_xyz_port_.write(*dds_xyz_);
            return_loan(*dds_xyz_);
        }
        if (output_ri_)
        {
            pcl::PointCloud<pcl::PointWithRange>::Ptr ri(
                    new pcl::PointCloud<pcl::PointWithRange>);
            make_range_image(ri, cloud); // Using the raw depth image leads to segfaults
            pcl_to_pointcloud<pcl::PointWithRange>(ri, *dds_ri_, true);
            dds_ri_->seq = seq_;
            dds_ri_->tm.sec = ts / 1000000000;
            dds_ri_->tm.nsec = ts % 1000000000;
            dds_ri_port_.write(*dds_ri_);
            return_loan(*dds_ri_);
        }
    }
    ++seq_;
#endif // defined(DDS_SUPPORT)
}


void RTCPCLOpenNI::xyzrgb_cb(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const& cloud)
{
    uint64_t ts(get_ts());
    if (corba_)
    {
        pcl_to_pointcloud<pcl::PointXYZRGB>(cloud, corba_xyzrgb_);
        corba_xyzrgb_.seq = seq_;
        corba_xyzrgb_.tm.sec = ts / 1000000000;
        corba_xyzrgb_.tm.nsec = ts % 1000000000;
        corba_xyzrgb_port_.write();
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        pcl_to_pointcloud<pcl::PointXYZRGB>(cloud, *dds_xyzrgb_, true);
        dds_xyzrgb_->seq = seq_;
        dds_xyzrgb_->tm.sec = ts / 1000000000;
        dds_xyzrgb_->tm.nsec = ts % 1000000000;
        dds_xyzrgb_port_.write(*dds_xyzrgb_);
        return_loan(*dds_xyzrgb_);
    }
#endif // defined(DDS_SUPPORT)
    ++seq_;
}


void RTCPCLOpenNI::make_range_image(
        pcl::PointCloud<pcl::PointWithRange>::Ptr& ri,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
{
    ri->height = cloud->height;
    ri->width = cloud->width;
    ri->points.resize(cloud->size());
    for (unsigned int ii(0); ii < cloud->size(); ++ii)
    {
        pcl::PointXYZ const& s = cloud->points[ii];
        pcl::PointWithRange& d = ri->points[ii];
        d.x = s.x;
        d.y = s.y;
        d.z = s.z;
        d.range = d.getVector3fMap().norm();
    }
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLOpenNI",
    "type_name",         "RTCPCLOpenNI",
    "description",       "Point cloud component for OpenNI devices",
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
    "conf.default.device", "#1",
    "conf.default.output_xyz", "0",
    "conf.default.output_xyzrgb", "1",
    "conf.default.output_range_image", "0",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.device", "text",
    "conf.__widget__.output_xyz", "radio",
    "conf.__widget__.output_xyzrgb", "radio",
    "conf.__widget__.output_range_image", "radio",
    // Constraints
    ""
};


extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLOpenNI>,
                RTC::Delete<RTCPCLOpenNI>);
    }
};

