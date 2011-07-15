/* RTC:PCLVoxelFilter
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


#include "rtcpclvoxelfilter.h"

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <iostream>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLVoxelFilter::RTCPCLVoxelFilter(RTC::Manager* manager)
    : RTCPCLBase(manager),
    res_x_(0.01), res_y_(0.01), res_z_(0.01), point_type_("xyz")
{
}


RTC::ReturnCode_t RTCPCLVoxelFilter::onInitialize()
{
    RTCPCLBase::onInitialize();
    bindParameter("res_x", res_x_, "0.01");
    bindParameter("res_y", res_y_, "0.01");
    bindParameter("res_z", res_z_, "0.01");
    bindParameter("point_type", point_type_, "xyz");
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLVoxelFilter::onActivated(RTC::UniqueId ec_id)
{
    RTC::ReturnCode_t ret(RTC::RTC_OK);
    if ((ret = RTCPCLBase::onActivated(ec_id)) != RTC::RTC_OK)
    {
        return ret;
    }
    voxel_filter_ = create_type<pcl::VoxelGrid, HasXYZ>(point_type_);
    if (voxel_filter_.empty())
    {
        std::cerr << "Unsupported point type for voxel filter\n";
        return RTC::RTC_ERROR;
    }
    return RTC::RTC_OK;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLVoxelFilter",
    "type_name",         "RTCPCLVoxelFilter",
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
    "conf.default.res_x", "0.01",
    "conf.default.res_y", "0.01",
    "conf.default.res_z", "0.01",
    "conf.default.point_type", "xyz",
    // Widget
    "conf.default.corba", "1",
    "conf.default.dds", "1",
    "conf.default.pointer", "1",
    "conf.__widget__.res_x", "spin",
    "conf.__widget__.res_y", "spin",
    "conf.__widget__.res_z", "spin",
    "conf.__widget__.point_type", "text",
    // Constraints
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__constraints__.res_x", "0<=x",
    "conf.__constraints__.res_y", "0<=x",
    "conf.__constraints__.res_z", "0<=x",
    ""
};


extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLVoxelFilter>,
                RTC::Delete<RTCPCLVoxelFilter>);
    }
};

