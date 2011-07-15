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


#include "rtcpcdsave.h"

#include <rtcpcl/pc_traits.h>

#include <iostream>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCDSave::RTCPCDSave(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_in_port_("corba_in", corba_in_),
#if defined(DDS_SUPPORT)
    dds_in_port_("dds_in",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    corba_(true), dds_(true), pointer_(true), binary_(true), saved_(false),
    write_once_(true)
{
}


RTCPCDSave::~RTCPCDSave()
{
}


RTC::ReturnCode_t RTCPCDSave::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("binary", binary_, "1");
    bindParameter("pcd_file", filename_, "");
    bindParameter("write_once", write_once_, "1");
    addInPort(corba_in_port_.getName(), corba_in_port_);
#if defined(DDS_SUPPORT)
    addDDSSubPort(dds_in_port_.getName(), dds_in_port_, *this);
    register_pc_type(dds_in_port_.get_participant());
#endif // defined(DDS_SUPPORT)

    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDSave::onActivated(RTC::UniqueId ec_id)
{
    // Reset saved_ so that the first cloud received will be saved
    saved_ = false;

    if (filename_ == "")
    {
        std::cerr << "No file name given\n";
        return RTC::RTC_ERROR;
    }

    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDSave::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDSave::onExecute(RTC::UniqueId ec_id)
{
#define RTCPCDSAVE_CORBA(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        save_port<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(); \
        saved_ = true; \
    }
    /***/

#define RTCPCDSAVE_DDS(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        save_port<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(clouds[0]); \
        saved_ = true; \
    }
    /***/



    if (corba_ && corba_in_port_.isNew())
    {
        corba_in_port_.read();
        if (!saved_ || !write_once_)
        {
            std::string cloud_type(corba_in_.type);
            BOOST_PP_SEQ_FOR_EACH(RTCPCDSAVE_CORBA, _, PCL_TAGGED_POINT_TYPES);
        }
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            if (!saved_ || !write_once_)
            {
                std::string cloud_type(clouds[0].type);
                BOOST_PP_SEQ_FOR_EACH(RTCPCDSAVE_DDS, _, PCL_TAGGED_POINT_TYPES);
            }
            dds_in_port_.return_loan(clouds, info);
        }
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCDSave",
    "type_name",         "RTCPCDSave",
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
    "conf.default.dds", "0",
    "conf.default.pointer", "0",
    "conf.default.binary", "1",
    "conf.default.write_once", "1",
    "conf.default.pcd_file", "",
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.binary", "radio",
    "conf.__widget__.write_once", "radio",
    "conf.__widget__.pcd_file", "text",
    // Constraints
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCDSave>,
                RTC::Delete<RTCPCDSave>);
    }
};

