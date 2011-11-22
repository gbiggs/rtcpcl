/* RTC:PCLFPFH
 *
 * Component header file.
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
 * License along with RTC:PCL. If not, see
 * <http://www.gnu.org/licenses/>.
 */


#ifndef RTCPCLFPFH_H
#define RTCPCLFPFH_H

#include <rtcpcl/config.h>
#include <rtcpcl/pc_mgmt.h>
#include <rtcpcl/pc_traits.h>
#include <rtcpcl/rtcpclbase.h>

#include <boost/any.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <rtm/OutPort.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud.hh>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#include <rtmdds/ddspubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
    template<bool b>
    struct process_impl
    {
        template<typename PointT>
        static bool do_it(boost::any& vf_any, double res_x, double res_y,
                double res_z,
                typename pcl::PointCloud<PointT>::Ptr& input,
                typename pcl::PointCloud<PointT>::Ptr &output)
        {
            std::cerr <<
                "RTCPCLFPFH: Cloud does not have XYZ fields.\n";
            return false;
        }
    };

    template<>
    struct process_impl<true>
    {
        template<typename PointT>
        static bool do_it(boost::any& vf_any, double res_x, double res_y,
                double res_z,
                typename pcl::PointCloud<PointT>::Ptr& input,
                typename pcl::PointCloud<PointT>::Ptr& output)
        {
            boost::shared_ptr<pcl::VoxelGrid<PointT> > vf;
            try
            {
                vf = boost::any_cast<boost::shared_ptr<
                    pcl::VoxelGrid<PointT> > >(
                            vf_any);
            }
            catch(boost::bad_any_cast const&)
            {
                std::cerr << "RTCPCLFPFH: Input has wrong point type\n";
                throw std::runtime_error("Wrong point type");
            }
            vf->setLeafSize(res_x, res_y, res_z);
            vf->setInputCloud(input);
            vf->filter(*output);
            return true;
        }
    };

    class RTCPCLFPFH
        : public RTCPCLBase
    {
        public:
            RTCPCLFPFH(RTC::Manager* manager);

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

        private:
            double res_x_, res_y_, res_z_;
            std::string point_type_;

            boost::any estimator_;

#define FPFH_APPLY(r, _, elem) \
            bool process(pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr& input, \
                    pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr& output) \
            { \
                return process_impl< \
                    HasXYZ<BOOST_PP_TUPLE_ELEM(2, 1, elem) && \
                    HasNormals<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::value>::do_it< \
                        BOOST_PP_TUPLE_ELEM(2, 1, elem)>( \
                            estimator_, res_x_, res_y_, res_z_, input, \
                            output); \
            }

            BOOST_PP_SEQ_FOR_EACH(VOXELFILTER_APPLY, _, PCL_TAGGED_POINT_TYPES);
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCLFPFH_H

