/* RTC:PCLPlaneSegmentation
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


#include "rtcpclplanesegmentation.h"

#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLPlaneSegmentation::RTCPCLPlaneSegmentation(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_in_port_("corba_in", corba_in_),
    corba_inliers_port_("corba_inliers", corba_inliers_),
    corba_outliers_port_("corba_outliers", corba_outliers_),
#if defined(DDS_SUPPORT)
    dds_in_port_("dds_in",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_inliers_(0),
    dds_inliers_port_("dds_inliers",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_outliers_(0),
    dds_outliers_port_("dds_outliers",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true),
    ndist_weight_(0.1), max_iters_(100), dist_thresh_(0.03)
{
}


RTCPCLPlaneSegmentation::~RTCPCLPlaneSegmentation()
{
}


RTC::ReturnCode_t RTCPCLPlaneSegmentation::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("ndist_weight", ndist_weight_, "0.1");
    bindParameter("max_iters", max_iters_, "100");
    bindParameter("dist_thresh", dist_thresh_, "0.03");
    addInPort(corba_in_port_.getName(), corba_in_port_);
    addOutPort(corba_inliers_port_.getName(), corba_inliers_port_);
    addOutPort(corba_outliers_port_.getName(), corba_outliers_port_);
#if defined(DDS_SUPPORT)
    addDDSSubPort(dds_in_port_.getName(), dds_in_port_, *this);
    register_pc_type(dds_in_port_.get_participant());
    addDDSPubPort(dds_inliers_port_.getName(), dds_inliers_port_, *this);
    register_pc_type(dds_inliers_port_.get_participant());
    addDDSPubPort(dds_outliers_port_.getName(), dds_outliers_port_, *this);
    register_pc_type(dds_outliers_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLPlaneSegmentation::onActivated(RTC::UniqueId ec_id)
{
#if defined(DDS_SUPPORT)
    dds_inliers_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_inliers_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_inliers_->data.maximum(0);
    dds_outliers_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_outliers_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_outliers_->data.maximum(0);
#endif // defined(DDS_SUPPORT)

    segmenter_.setOptimizeCoefficients(true);
    segmenter_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    segmenter_.setMethodType(pcl::SAC_RANSAC);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLPlaneSegmentation::onDeactivated(RTC::UniqueId ec_id)
{
#if defined(DDS_SUPPORT)
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_inliers_);
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_outliers_);
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLPlaneSegmentation::onExecute(RTC::UniqueId ec_id)
{
    if (corba_ && corba_in_port_.isNew())
    {
        corba_in_port_.read();
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in_cloud(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inliers_cloud(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outliers_cloud(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pointcloud_to_pcl<pcl::PointXYZRGBNormal>(corba_in_, in_cloud);
        process(in_cloud, inliers_cloud, outliers_cloud);
        pcl_to_pointcloud<pcl::PointXYZRGBNormal>(inliers_cloud, corba_inliers_);
        pcl_to_pointcloud<pcl::PointXYZRGBNormal>(outliers_cloud, corba_outliers_);
        corba_inliers_port_.write();
        corba_outliers_port_.write();
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inliers_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outliers_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pointcloud_to_pcl<pcl::PointXYZRGBNormal>(clouds[0], in_cloud);
            process(in_cloud, inliers_cloud, outliers_cloud);
            pcl_to_pointcloud<pcl::PointXYZRGBNormal>(inliers_cloud, *dds_inliers_,
                    true);
            dds_inliers_port_.write(*dds_inliers_);
            return_loan(*dds_inliers_);
            pcl_to_pointcloud<pcl::PointXYZRGBNormal>(outliers_cloud, *dds_outliers_,
                    true);
            dds_outliers_port_.write(*dds_outliers_);
            return_loan(*dds_outliers_);
            dds_in_port_.return_loan(clouds, info);
        }
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


void RTCPCLPlaneSegmentation::process(
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in_cloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inliers,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outliers)
{
    std::cerr << "Processing " << in_cloud->points.size() << " points\n";
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

    segmenter_.setNormalDistanceWeight(ndist_weight_);
    segmenter_.setMaxIterations(max_iters_);
    segmenter_.setDistanceThreshold(dist_thresh_);
    segmenter_.setInputCloud(in_cloud);
    segmenter_.setInputNormals(in_cloud);
    segmenter_.segment(*indices, *coeffs);

    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractor;
    extractor.setInputCloud(in_cloud);
    extractor.setIndices(indices);
    extractor.setNegative(false);
    extractor.filter(*inliers);
    extractor.setNegative(true);
    extractor.filter(*outliers);

    std::cerr << "Filtered to " << inliers->points.size() << " points\n";
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLPlaneSegmentation",
    "type_name",         "RTCPCLPlaneSegmentation",
    "description",       "Point cloud component for planar segmentation",
    "version",           "1.0",
    "vendor",            "Geoffrey Biggs, AIST",
    "category",          "PCL",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.ndist_weight", "0.1",
    "conf.default.max_iters", "100",
    "conf.default.dist_thresh", "0.03",
    // Widget
    "conf.default.corba", "1",
    "conf.default.dds", "1",
    "conf.default.pointer", "1",
    "conf.__widget__.ndist_weight", "spin",
    "conf.__widget__.max_iters", "spin",
    "conf.__widget__.dist_thresh", "spin",
    // Constraints
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__constraints__.max_iters", "1<=x",
    "conf.__constraints__.dist_thresh", "0<=x",
    ""
};


extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLPlaneSegmentation>,
                RTC::Delete<RTCPCLPlaneSegmentation>);
    }
};

