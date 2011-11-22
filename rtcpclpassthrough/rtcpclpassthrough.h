/* RTC:PCLPassthrough
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


#ifndef RTCPCLPassthrough_H
#define RTCPCLPassthrough_H

#include <rtcpcl/config.h>
#include <rtcpcl/pc_mgmt.h>
#include <rtcpcl/pc_traits.h>
#include <rtcpcl/rtcpclbase.h>

#include <boost/any.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <rtm/OutPort.h>
#include <pcl/filters/passthrough.h>
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
        static bool do_it(boost::any& vf_any, std::string field,
                double lower_bound, double upper_bound,
                typename pcl::PointCloud<PointT>::Ptr& input,
                typename pcl::PointCloud<PointT>::Ptr &output)
        {
            std::cerr <<
                "RTCPCLPassthrough: Cloud does not have XYZ fields.\n";
            return false;
        }
    };

    template<>
    struct process_impl<true>
    {
        template<typename PointT>
        static bool do_it(boost::any& vf_any, std::string field,
                double lower_bound, double upper_bound,
                typename pcl::PointCloud<PointT>::Ptr& input,
                typename pcl::PointCloud<PointT>::Ptr& output)
        {
            boost::shared_ptr<pcl::PassThrough<PointT> > f;
            try
            {
                f = boost::any_cast<boost::shared_ptr<
                    pcl::PassThrough<PointT> > >(
                            vf_any);
            }
            catch(boost::bad_any_cast const&)
            {
                std::cerr << "RTCPCLPassthrough: Input has wrong point type\n";
                throw std::runtime_error("Wrong point type");
            }
            f->setFilterFieldName(field);
            f->setFilterLimits(lower_bound, upper_bound);
            f->setInputCloud(input);
            f->filter(*output);
            std::cerr << "Filtered from " << input->points.size() <<
                " to " << output->points.size() << '\n';
            return true;
        }
    };

    class RTCPCLPassthrough
        : public RTCPCLBase
    {
        public:
            RTCPCLPassthrough(RTC::Manager* manager);

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

        private:
            std::string field_;
            double lower_bound_, upper_bound_;
            std::string point_type_;

            boost::any filter_;

#define FILTER_APPLY(r, _, elem) \
            bool process(pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr& input, \
                    pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::Ptr& output) \
            { \
                return process_impl< \
                    HasXYZ<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::value>::do_it< \
                        BOOST_PP_TUPLE_ELEM(2, 1, elem)>( \
                            filter_, field_, lower_bound_, upper_bound_, \
                            input, output); \
            }

            BOOST_PP_SEQ_FOR_EACH(FILTER_APPLY, _, PCL_TAGGED_POINT_TYPES);
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCLPassthrough_H

