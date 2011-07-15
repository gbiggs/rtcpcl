/* RTC:PCLNormals
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


#include "rtcpclnormals.h"

#include <iostream>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLNormals::RTCPCLNormals(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_in_port_("corba_in", corba_in_),
    corba_out_port_("corba_out", corba_out_),
#if defined(DDS_SUPPORT)
    dds_out_(0),
    dds_in_port_("dds_in",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_out_port_("dds_out",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true), radius_(0.03)
{
}


RTCPCLNormals::~RTCPCLNormals()
{
}


RTC::ReturnCode_t RTCPCLNormals::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("radius", radius_, "0.03");
    addInPort(corba_in_port_.getName(), corba_in_port_);
    addOutPort(corba_out_port_.getName(), corba_out_port_);
#if defined(DDS_SUPPORT)
    addDDSSubPort(dds_in_port_.getName(), dds_in_port_, *this);
    register_pc_type(dds_in_port_.get_participant());
    addDDSPubPort(dds_out_port_.getName(), dds_out_port_, *this);
    register_pc_type(dds_out_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLNormals::onActivated(RTC::UniqueId ec_id)
{
#if defined(DDS_SUPPORT)
    dds_out_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_out_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_out_->data.maximum(0);
#endif // defined(DDS_SUPPORT)

    estimator_.reset(new pcl::NormalEstimation<pcl::PointXYZRGB,
            pcl::PointXYZRGBNormal>());
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(
            new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    estimator_->setSearchMethod(tree);
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLNormals::onDeactivated(RTC::UniqueId ec_id)
{
#if defined(DDS_SUPPORT)
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_out_);
#endif // defined(DDS_SUPPORT)
    estimator_.reset();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLNormals::onExecute(RTC::UniqueId ec_id)
{
    if (corba_ && corba_in_port_.isNew())
    {
        corba_in_port_.read();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pointcloud_to_pcl<pcl::PointXYZRGB>(corba_in_, in_cloud);
        process(in_cloud, out_cloud);
        pcl_to_pointcloud<pcl::PointXYZRGBNormal>(out_cloud, corba_out_);
        corba_out_port_.write();
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pointcloud_to_pcl<pcl::PointXYZRGB>(clouds[0], in_cloud);
            process(in_cloud, out_cloud);
            pcl_to_pointcloud<pcl::PointXYZRGBNormal>(out_cloud, *dds_out_,
                    true);
            dds_out_port_.write(*dds_out_);
            return_loan(*dds_out_);
            dds_in_port_.return_loan(clouds, info);
        }
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


void RTCPCLNormals::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud)
{
    estimator_->setInputCloud(in_cloud);
    estimator_->setRadiusSearch(radius_);
    estimator_->compute(*out_cloud);
    for (size_t ii(0); ii < in_cloud->size(); ++ii)
    {
        out_cloud->points[ii].x = in_cloud->points[ii].x;
        out_cloud->points[ii].y = in_cloud->points[ii].y;
        out_cloud->points[ii].z = in_cloud->points[ii].z;
        out_cloud->points[ii].rgb = in_cloud->points[ii].rgb;
    }
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLNormals",
    "type_name",         "RTCPCLNormals",
    "description",       "Point cloud component for voxel filtering",
    "version",           "1.0",
    "vendor",            "Geoffrey Biggs, AIST",
    "category",          "PCL",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.radius", "0.03",
    // Widget
    "conf.default.corba", "1",
    "conf.default.dds", "1",
    "conf.default.pointer", "1",
    "conf.__widget__.radius", "spin",
    // Constraints
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__constraints__.radius", "0<=x",
    ""
};


extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLNormals>,
                RTC::Delete<RTCPCLNormals>);
    }
};

