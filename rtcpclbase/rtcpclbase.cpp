/* RTC:PCLBase
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


#include <rtcpcl/rtcpclbase.h>

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <iostream>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLBase::RTCPCLBase(RTC::Manager* manager)
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
    corba_(true), dds_(true), pointer_(true)
{
}


RTCPCLBase::~RTCPCLBase()
{
}


RTC::ReturnCode_t RTCPCLBase::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");

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


RTC::ReturnCode_t RTCPCLBase::onActivated(RTC::UniqueId ec_id)
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
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLBase::onDeactivated(RTC::UniqueId ec_id)
{
#if defined(DDS_SUPPORT)
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_out_);
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}

RTC::ReturnCode_t RTCPCLBase::onExecute(RTC::UniqueId ec_id)
{
#define RTCPCLBASE_CORBA(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr in_cloud( \
                new pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>); \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr out_cloud( \
                new pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>); \
        pointcloud_to_pcl<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(corba_in_, \
                in_cloud); \
        bool write(process(in_cloud, out_cloud)); \
        if (write) \
        { \
            pcl_to_pointcloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(out_cloud, \
                    corba_out_); \
            corba_out_port_.write(); \
        } \
    }

#define RTCPCLBASE_DDS(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr in_cloud( \
                new pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>); \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr out_cloud( \
                new pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>); \
        pointcloud_to_pcl<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(clouds[0], \
                in_cloud); \
        bool write(process(in_cloud, out_cloud)); \
        if (write) \
        { \
            pcl_to_pointcloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(out_cloud, \
                    *dds_out_, true); \
            dds_out_port_.write(*dds_out_); \
            return_loan(*dds_out_); \
        } \
    }

    if (corba_ && corba_in_port_.isNew())
    {
        corba_in_port_.read();
        std::string cloud_type(corba_in_.type);
        BOOST_PP_SEQ_FOR_EACH(RTCPCLBASE_CORBA, _, PCL_TAGGED_POINT_TYPES);
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            std::string cloud_type(clouds[0].type);
            BOOST_PP_SEQ_FOR_EACH(RTCPCLBASE_DDS, _, PCL_TAGGED_POINT_TYPES);
            dds_in_port_.return_loan(clouds, info);
        }
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}

