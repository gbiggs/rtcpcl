/* RTC:PCLViewer
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
 * License along with RTM:PCL. If not, see
 * <http://www.gnu.org/licenses/>.
 */


#ifndef RTCPCLVIEWER_H
#define RTCPCLVIEWER_H

#include <rtcpcl/config.h>
#include <rtcpcl/pc_mgmt.h>
#include <rtcpcl/pc_traits.h>

#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pointcloud.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/InPort.h>
#include <string>
#include <utility>
#include <vector>

#if defined(DDS_SUPPORT)
#include <pointcloud.h>
#include <pointcloudSupport.h>
#include <rtmdds/ddssubport.h>
#endif // defined(DDS_SUPPORT)


namespace RTCPCL
{
    struct PortType
    {
        PortType()
            : viewport(0), display_type(DT_POINT), normals_level(100),
            normals_scale(0.02)
        {}

        std::string name;
        int viewport; // 0 for all viewports
        typedef enum {DT_POINT, DT_NORMAL, DT_PRINCUR} DisplayType;
        DisplayType display_type;
        int normals_level;
        double normals_scale;
        struct Colour
        {
            Colour()
                : type(CT_PLAIN), r(255), g(255), b(255)
            {}

            typedef enum {CT_PLAIN, CT_RANDOM, CT_RGB, CT_CUSTOM, CT_FIELD} Type;
            Type type;
            unsigned int r;
            unsigned int g;
            unsigned int b;
            std::string field;
        };
        Colour colour;
    };


    template<bool b>
    struct get_colour_handler
    {
        template<typename PointT>
        static typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr do_it(
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ and RGB information.\n";
            throw std::runtime_error("Cloud " + p.name +
                    " does not have XYZ and RGB information");
        }
    };

    template<>
    struct get_colour_handler<true>
    {
        template<typename PointT>
        static typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr do_it(
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p)
        {
            typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr ch;
            switch (p.colour.type)
            {
                case PortType::Colour::CT_RANDOM:
                    ch.reset(new typename
                            pcl::visualization::PointCloudColorHandlerRandom<PointT>(
                                c));
                case PortType::Colour::CT_CUSTOM:
                    ch.reset(new typename
                            pcl::visualization::PointCloudColorHandlerCustom<PointT>(
                                c, p.colour.r, p.colour.g, p.colour.g));
                case PortType::Colour::CT_FIELD:
                    ch.reset(new typename
                            pcl::visualization::PointCloudColorHandlerGenericField<PointT>(
                                c, p.colour.field));
                    break;
                case PortType::Colour::CT_PLAIN:
                    // Already done in other branch of if
                    break;
                case PortType::Colour::CT_RGB:
                    // Already done in caller
                    break;
            }
            return ch;
        }
    };


    template<bool b>
    struct get_rgb_handler
    {
        template<typename PointT>
        static typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr do_it(
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ and RGB information.\n";
            return typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr(
                    new typename pcl::visualization::PointCloudColorHandlerCustom<PointT>(
                c, 255, 255, 255));
        }
    };

    template<>
    struct get_rgb_handler<true>
    {
        template<typename PointT>
        static typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr do_it(
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p)
        {
            return typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr(
                    new typename pcl::visualization::PointCloudColorHandlerRGBField<PointT>(
                c));
        }
    };


    template<bool b>
    struct draw_points
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ information.\n";
        }

        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, 
                typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr ch,
                PortType& p, int viewport)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ information.\n";
        }
    };

    template<>
    struct draw_points<true>
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            v->removePointCloud(p.name);
            v->addPointCloud<PointT>(c, p.name, viewport);
        }

        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c,
                typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr ch,
                PortType& p, int viewport)
        {
            v->removePointCloud(p.name);
            v->addPointCloud<PointT>(c, *ch, p.name, viewport);
        }
    };


    template<bool b>
    struct draw_normals
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ and normals information.\n";
        }
    };

    template<>
    struct draw_normals<true>
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            v->removePointCloud(p.name + "_normals");
            v->addPointCloudNormals<PointT>(c, p.normals_level,
                    p.normals_scale, p.name + "_normals", viewport);
        }
    };


    template<bool b>
    struct draw_curvatures
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            std::cerr << "Cloud " << p.name <<
                " does not have XYZ, normals and curvature information.\n";
        }
    };

    template<>
    struct draw_curvatures<true>
    {
        template<typename PointT>
        static void do_it(boost::shared_ptr<pcl::visualization::PCLVisualizer> v,
                typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p,
                int viewport)
        {
            v->removePointCloud(p.name + "_curvatures");
            v->addPointCloudPrincipalCurvatures(c, c, c,
                    p.name + "_curvatures", p.normals_level, p.normals_scale,
                    viewport);
        }
    };


    class RTCPCLViewer
    : public RTC::DataFlowComponentBase
    {
        public:
            RTCPCLViewer(RTC::Manager* manager);
            ~RTCPCLViewer();

            virtual RTC::ReturnCode_t onInitialize();
            virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
            virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        private:
            std::vector<PortType> port_types_;

            typedef RTC::InPort<PointCloudTypes::PointCloud> CorbaPortType;
            typedef boost::shared_ptr<CorbaPortType> CorbaPortPtr;
            typedef boost::shared_ptr<PointCloudTypes::PointCloud> CorbaCloudPtr;
            struct CorbaPort
            {
                CorbaPortPtr port;
                CorbaCloudPtr cloud;
                PortType type;
            };
            typedef std::vector<CorbaPort> CorbaPortVector;
            CorbaPortVector corba_ports_;

#if defined(DDS_SUPPORT)
            typedef RTC::DDSSubPort<PointCloudTypes_PointCloud,
                    PointCloudTypes_PointCloudSeq,
                    PointCloudTypes_PointCloudDataReader> DDSPortType;
            typedef boost::shared_ptr<DDSPortType> DDSPortPtr;
            struct DDSPort
            {
                DDSPortPtr port;
                PortType type;
            };
            typedef std::vector<DDSPort> DDSPortVector;
            DDSPortVector dds_ports_;
#endif // defined(DDS_SUPPORT)

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
            std::map<int, int> viewport_ids_;

            bool corba_;
            bool dds_;
            bool pointer_;
            float axes_scale_;
            std::string port_types_str_; // name:viewport:p|n;100;0.1|c:p|a|r|0.0.0|"x",...
            bool show_ts_;

            void get_port_types();
            void print_port_types();
            void create_input_ports();
            void create_dds_ports();
            void create_viewports();

            template<typename PointT>
            void draw_cloud(typename pcl::PointCloud<PointT>::ConstPtr c, PortType& p)
            {
                if (p.colour.type == PortType::Colour::CT_PLAIN)
                {
                    // Yay, easy!
                    draw_points<HasXYZ<PointT>::value>::
                        template do_it<PointT>(viewer_, c, p,
                                viewport_ids_[p.viewport]);
                    switch (p.display_type)
                    {
                        case PortType::DT_POINT:
                            // Already added the cloud
                            break;
                        case PortType::DT_NORMAL:
                            draw_normals<HasXYZ<PointT>::value &&
                                    HasNormals<PointT>::value>::
                                template do_it<PointT>(viewer_, c, p,
                                        viewport_ids_[p.viewport]);
                            break;
                        case PortType::DT_PRINCUR:
                            draw_curvatures<HasXYZ<PointT>::value &&
                                    HasNormals<PointT>::value &&
                                    HasCurvatures<PointT>::value>::
                                template do_it<PointT>(viewer_, c, p,
                                        viewport_ids_[p.viewport]);
                            break;
                    }
                }
                else
                {
                    typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr ch;
                    switch (p.colour.type)
                    {
                        case PortType::Colour::CT_RGB:
                            ch = get_rgb_handler<HasXYZ<PointT>::value &&
                                        HasRGB<PointT>::value>::
                                    template do_it<PointT>(c, p);
                            break;
                        default:
                            ch = get_colour_handler<HasXYZ<PointT>::value>::
                                    template do_it<PointT>(c, p);
                            break;
                    }
                    draw_points<HasXYZ<PointT>::value>::
                        template do_it<PointT>(viewer_, c, ch, p,
                                viewport_ids_[p.viewport]);
                    switch (p.display_type)
                    {
                        case PortType::DT_POINT:
                            // Already added the cloud
                            break;
                        case PortType::DT_NORMAL:
                            draw_normals<HasXYZ<PointT>::value &&
                                    HasNormals<PointT>::value>::
                                template do_it<PointT>(viewer_, c, p,
                                        viewport_ids_[p.viewport]);
                            break;
                        case PortType::DT_PRINCUR:
                            draw_curvatures<HasXYZ<PointT>::value &&
                                    HasNormals<PointT>::value &&
                                    HasCurvatures<PointT>::value>::
                                template do_it<PointT>(viewer_, c, p,
                                        viewport_ids_[p.viewport]);
                            break;
                    }
                }
            }


            template<typename PointT>
            typename pcl::PointCloud<PointT>::ConstPtr get_corba_port(CorbaPort& p)
            {
                typename pcl::PointCloud<PointT>::Ptr
                    pcl_cloud(new pcl::PointCloud<PointT>);
                pointcloud_to_pcl<PointT>(*p.cloud, pcl_cloud);
                return pcl_cloud;
            }

#if defined(DDS_SUPPORT)
            template<typename PointT>
            typename pcl::PointCloud<PointT>::ConstPtr get_dds_port(
                    PointCloudTypes_PointCloud& cloud)
            {
                typename pcl::PointCloud<PointT>::Ptr
                    pcl_cloud(new pcl::PointCloud<PointT>);
                pointcloud_to_pcl<PointT>(cloud, pcl_cloud);
                return pcl_cloud;
            }
#endif // defined(DDS_SUPPORT)
    };
}; // namespace RTCPCL

extern "C"
{
    DLL_EXPORT void rtc_init(RTC::Manager* manager);
};

#endif // RTCPCLVIEWER_H

