/* RTM:PCL Point Cloud Type
 *
 * Source file for point cloud management functions.
 *
 * Copyright 2011 Geoffrey Biggs geoffrey.biggs@aist.go.jp
 *     RT-Synthesis Research Group
 *     Intelligent Systems Research Institute,
 *     National Institute of Advanced Industrial Science and Technology (AIST),
 *     Japan
 *     All rights reserved.
 *
 * This file is part of RTM:PCL.
 *
 * RTM:PCL is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License,
 * or (at your option) any later version.
 *
 * RTM:PCL is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with RTM:PCL. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <rtcpcl/pc_mgmt.h>

#if defined(DDS_SUPPORT)
#include <pointcloudSupport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
#if defined(DDS_SUPPORT)
    void register_pc_type(DDSDomainParticipant* participant)
    {
        if (PointCloudTypes_PointCloudTypeSupport::register_type(participant,
                PointCloudTypes_PointCloudTypeSupport::get_type_name()) !=
                DDS_RETCODE_OK)
        {
            throw std::runtime_error("Failed to register DDS data type");
        }
    }


    void return_loan(PointCloudTypes_PointCloud& cloud)
    {
        cloud.data.unloan();
    }
#endif // defined(DDS_SUPPORT)
}; // namespace RTCPCL

