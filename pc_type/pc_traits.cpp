/* RTM:PCL Point Cloud Type
 *
 * Source file for point type traits.
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

#include <rtcpcl/pc_traits.h>


namespace RTCPCL
{
    std::string PointName<pcl::PointXYZ>::value = "xyz";
    std::string PointName<pcl::PointXYZI>::value = "xyzi";
    std::string PointName<pcl::PointXYZRGBA>::value = "xyzrgba";
    std::string PointName<pcl::PointXYZRGB>::value = "xyzrgb";
    std::string PointName<pcl::PointXY>::value = "xy";
    std::string PointName<pcl::InterestPoint>::value = "interestpoint";
    std::string PointName<pcl::Normal>::value = "normal";
    std::string PointName<pcl::PointNormal>::value = "pointnormal";
    std::string PointName<pcl::PointXYZRGBNormal>::value = "xyzrgbnormal";
    std::string PointName<pcl::PointXYZINormal>::value = "xyzinormal";
    std::string PointName<pcl::PointWithRange>::value = "pointwithrange";
    std::string PointName<pcl::PointWithViewpoint>::value = "pointwithviewpoint";
    std::string PointName<pcl::MomentInvariants>::value = "momentinvariants";
    std::string PointName<pcl::PrincipalRadiiRSD>::value = "principalradiirsd";
    std::string PointName<pcl::Boundary>::value = "boundary";
    std::string PointName<pcl::PrincipalCurvatures>::value = "principalcurvatures";
    std::string PointName<pcl::PFHSignature125>::value = "pfhsignature125";
    //std::string PointName<pcl::PPFSignature>::value = "ppfsignature";
    std::string PointName<pcl::FPFHSignature33>::value = "fpfhsignature33";
    std::string PointName<pcl::VFHSignature308>::value = "vfhsignature308";
    std::string PointName<pcl::Narf36>::value = "narf36";
    std::string PointName<pcl::IntensityGradient>::value = "intensitygradient";
    std::string PointName<pcl::PointWithScale>::value = "pointwithscale";
    std::string PointName<pcl::PointSurfel>::value = "pointsurfel";
}; // namespace RTCPCL


