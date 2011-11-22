/* RTC:PCLPassthrough
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


#include "rtcpclpassthrough.h"

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <iostream>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLPassthrough::RTCPCLPassthrough(RTC::Manager* manager)
    : RTCPCLBase(manager),
    field_("z"), lower_bound_(0.0), upper_bound_(2.0), point_type_("xyz")
{
}


RTC::ReturnCode_t RTCPCLPassthrough::onInitialize()
{
    RTCPCLBase::onInitialize();
    bindParameter("field", field_, "z");
    bindParameter("lower_bound", lower_bound_, "0.0");
    bindParameter("upper_bound", upper_bound_, "2.0");
    bindParameter("point_type", point_type_, "xyz");
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLPassthrough::onActivated(RTC::UniqueId ec_id)
{
    RTC::ReturnCode_t ret(RTC::RTC_OK);
    if ((ret = RTCPCLBase::onActivated(ec_id)) != RTC::RTC_OK)
    {
        return ret;
    }
    filter_ = create_type<pcl::PassThrough, HasXYZ>(point_type_);
    if (filter_.empty())
    {
        std::cerr << "Unsupported point type for passthrough filter\n";
        return RTC::RTC_ERROR;
    }
    return RTC::RTC_OK;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLPassthrough",
    "type_name",         "RTCPCLPassthrough",
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
    "conf.default.field", "z",
    "conf.default.lower_bound", "0.0",
    "conf.default.upper_bound", "2.0",
    "conf.default.point_type", "xyz",
    // Widget
    "conf.default.corba", "1",
    "conf.default.dds", "1",
    "conf.default.pointer", "1",
    "conf.__widget__.field", "text",
    "conf.__widget__.lower_bound", "spin",
    "conf.__widget__.upper_bound", "spin",
    "conf.__widget__.point_type", "text",
    // Constraints
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    ""
};


extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLPassthrough>,
                RTC::Delete<RTCPCLPassthrough>);
    }
};

