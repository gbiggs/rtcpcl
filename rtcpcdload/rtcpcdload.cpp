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


#include "rtcpcdload.h"

#include <rtcpcl/pc_traits.h>

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCDLoad::RTCPCDLoad(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_out_port_("corba_out", corba_out_),
#if defined(DDS_SUPPORT)
    dds_out_(0),
    dds_out_port_("dds_out",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true), written_(false),
    write_once_(true), seq_(0), point_type_("xyz")
{
}


RTCPCDLoad::~RTCPCDLoad()
{
}


RTC::ReturnCode_t RTCPCDLoad::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("write_once", write_once_, "1");
    bindParameter("pcd_file", filename_, "");
    bindParameter("point_type", point_type_, "xyz");
    addOutPort(corba_out_port_.getName(), corba_out_port_);
#if defined(DDS_SUPPORT)
    addDDSPubPort(dds_out_port_.getName(), dds_out_port_, *this);
    register_pc_type(dds_out_port_.get_participant());
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDLoad::onActivated(RTC::UniqueId ec_id)
{
    // Reset written_ so that the cloud will be written upon activation
    written_ = false;
    seq_ = 0;

    RTC::ReturnCode_t ret(RTC::RTC_OK);
    if ((ret = load_cloud()) != RTC::RTC_OK)
    {
        return ret;
    }
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDLoad::onDeactivated(RTC::UniqueId ec_id)
{
    destroy_cloud();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDLoad::onExecute(RTC::UniqueId ec_id)
{
    if (!written_ || !write_once_)
    {
        if (corba_)
        {
            corba_out_port_.write();
        }
#if defined(DDS_SUPPORT)
        if (dds_)
        {
            dds_out_port_.write(*dds_out_);
        }
#endif // defined(DDS_SUPPORT)
    }
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDLoad::load_cloud()
{
#define RTCPCDLOAD_LOAD_CLOUD(r, _, elem) \
    if (point_type_ == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr cloud( \
                new pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>); \
        if (pcl::io::loadPCDFile<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(filename_, \
                    *cloud) == -1) \
        { \
            std::cerr << "Error reading PCD file " << filename_ << \
                " as "BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 1, elem))"\n"; \
            return RTC::RTC_ERROR; \
        } \
        loaded_cloud_ = cloud; \
        set_packed_clouds<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(cloud); \
        return RTC::RTC_OK; \
    }
    /***/

#if defined(DDS_SUPPORT)
    dds_out_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_out_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_out_->data.maximum(0);
#endif // defined(DDS_SUPPORT)

    BOOST_PP_SEQ_FOR_EACH(RTCPCDLOAD_LOAD_CLOUD, _, PCL_TAGGED_POINT_TYPES);
    std::cerr << "Unknown point type: " << point_type_ << '\n';
    return RTC::RTC_ERROR;
}


RTC::ReturnCode_t RTCPCDLoad::destroy_cloud()
{
#define RTCPCDLOAD_DEST_CLOUD(r, _, elem) \
    if (point_type_ == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        destroy_packed_clouds(); \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr cloud( \
            boost::any_cast<pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr>( \
                loaded_cloud_)); \
        cloud.reset(); \
        return RTC::RTC_OK; \
    }
    /***/

    BOOST_PP_SEQ_FOR_EACH(RTCPCDLOAD_DEST_CLOUD, _, PCL_TAGGED_POINT_TYPES);
    std::cerr << "Unknown point type: " << point_type_ << '\n';
    return RTC::RTC_ERROR;
}


void RTCPCDLoad::destroy_packed_clouds()
{
#if defined(DDS_SUPPORT)
    if (!dds_out_->data.has_ownership())
    {
        return_loan(*dds_out_);
    }
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_out_);
#endif // defined(DDS_SUPPORT)
}


static const char* spec[] =
{
    "implementation_id", "RTCPCDLoad",
    "type_name",         "RTCPCDLoad",
    "description",       "Point cloud data file loader component",
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
    "conf.default.write_once", "1",
    "conf.default.pcd_file", "",
    "conf.default.point_type", "xyz",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.write_once", "radio",
    "conf.__widget__.pcd_file", "text",
    "conf.__widget__.point_type", "text",
    // Constraints
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCDLoad>,
                RTC::Delete<RTCPCDLoad>);
    }
};

