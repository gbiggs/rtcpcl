/* RTC:PCLViewer
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


#include "rtcpclviewer.h"

#include <rtcpcl/pc_traits.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <iomanip>

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCLViewer::RTCPCLViewer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_(true), dds_(true), pointer_(true), axes_scale_(0.0),
    port_types_str_("in"), show_ts_(false)
{
}


RTCPCLViewer::~RTCPCLViewer()
{
}


RTC::ReturnCode_t RTCPCLViewer::onInitialize()
{
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "1");
    bindParameter("pointer", pointer_, "1");
    bindParameter("axes_scale", axes_scale_, "0.0");
    bindParameter("ports", port_types_str_, "in");
    bindParameter("show_timestamps", show_ts_, "0");
    std::string active_set =
        m_properties.getProperty("configuration.active_config", "default");
    m_configsets.update(active_set.c_str());

    get_port_types();
    std::cout << "Ports:\n";
    print_port_types();
    create_input_ports();
    create_dds_ports();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLViewer::onActivated(RTC::UniqueId ec_id)
{
    viewer_.reset(new pcl::visualization::PCLVisualizer("RTC:PCLViewer"));
    create_viewports();
    viewer_->setBackgroundColor(0, 0, 0);
    if (axes_scale_ > 0.0)
    {
        viewer_->addCoordinateSystem(axes_scale_);
    }
    viewer_->initCameraParameters();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLViewer::onDeactivated(RTC::UniqueId ec_id)
{
    viewer_.reset();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCLViewer::onExecute(RTC::UniqueId ec_id)
{
#define RTCPCLVIEWER_DISP_CORBA(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::ConstPtr c( \
                get_corba_port<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(p)); \
        draw_cloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(c, p.type); \
    }
    /***/

#define RTCPCLVIEWER_DISP_DDS(r, _, elem) \
    if (cloud_type == BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))) \
    { \
        pcl::PointCloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>::ConstPtr c( \
                get_dds_port<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(clouds[0])); \
        draw_cloud<BOOST_PP_TUPLE_ELEM(2, 1, elem)>(c, p.type); \
    }
    /***/

    if (corba_)
    {
        BOOST_FOREACH(CorbaPort p, corba_ports_)
        {
            if (p.port->isNew())
            {
                unsigned long ts(get_ts());
                p.port->read();
                std::string cloud_type(p.cloud->type);
                BOOST_PP_SEQ_FOR_EACH(RTCPCLVIEWER_DISP_CORBA, _,
                        PCL_TAGGED_POINT_TYPES);
                if (show_ts_)
                {
                    unsigned long created_ts(
                            static_cast<unsigned long>(p.cloud->tm.sec) * 1000000000 +
                            p.cloud->tm.nsec);
                    std::cout << p.type.name << " Received: " << ts <<
                        " Created: " << created_ts << " Lag: " <<
                        ts - created_ts << '\n';
                }
            }
        }
    }
#if defined(DDS_SUPPORT)
    if (dds_)
    {
        BOOST_FOREACH(DDSPort p, dds_ports_)
        {
            PointCloudTypes_PointCloudSeq clouds;
            DDS_SampleInfoSeq info;
            if (p.port->read_with_loan(clouds, info))
            {
                unsigned long ts(get_ts());
                std::string cloud_type(clouds[0].type);
                BOOST_PP_SEQ_FOR_EACH(RTCPCLVIEWER_DISP_DDS, _,
                        PCL_TAGGED_POINT_TYPES);
                if (show_ts_)
                {
                    unsigned long created_ts(
                            static_cast<unsigned long>(clouds[0].tm.sec) * 1000000000 +
                            clouds[0].tm.nsec);
                    std::cout << p.type.name << " Received: " << ts <<
                        " Created: " << created_ts << " Lag: " <<
                        ts - created_ts << '\n';
                }
                p.port->return_loan(clouds, info);
            }
        }
    }
#endif // defined(DDS_SUPPORT)
    viewer_->spinOnce();
    if (viewer_->wasStopped())
    {
        // Deactivate self
    }
    return RTC::RTC_OK;
}


void RTCPCLViewer::get_port_types()
{
    static const boost::regex normal_re("n(;(?<l>\\d+);(?<s>[\\d.]+))?");
    static const boost::regex princur_re("c(;(?<l>\\d+);(?<s>[\\d.]+))?");
    static const boost::regex colour_re(
            "(?<r>\\d{1,3})\\.(?<g>\\d{1,3})\\.(?<b>\\d{1,3})");
    static const boost::regex field_re("\"(?<f>[a-zA-Z]+)\"");
    std::vector<std::string> split_port_types;
    boost::split(split_port_types, port_types_str_, boost::is_any_of(","));
    BOOST_FOREACH(std::string p, split_port_types)
    {
        std::vector<std::string> opts;
        boost::split(opts, p, boost::is_any_of(":"));
        if (opts.size() < 1)
        {
            std::cerr << "Bad port format: " << p << '\n';
            continue;
        }
        PortType new_port;
        new_port.name = opts[0];
        if (opts.size() > 1)
        {
            if (opts[1].size() == 0)
            {
                new_port.viewport = 0;
            }
            else
            {
                new_port.viewport = boost::lexical_cast<int>(opts[1]);
            }
        }
        if (opts.size() > 2)
        {
            boost::smatch fields;
            if (opts[2].size() == 0)
            {
                new_port.display_type = PortType::DT_POINT;
            }
            else if (opts[2] == "p")
            {
                new_port.display_type = PortType::DT_POINT;
            }
            else if (boost::regex_match(opts[2], fields, normal_re))
            {
                new_port.display_type = PortType::DT_NORMAL;
                if (fields["l"].matched)
                {
                    new_port.normals_level =
                        boost::lexical_cast<int>(fields["l"]);
                    new_port.normals_scale =
                        boost::lexical_cast<double>(fields["s"]);
                }
            }
            else if (boost::regex_match(opts[2], fields, princur_re))
            {
                new_port.display_type = PortType::DT_PRINCUR;
                if (fields["l"].matched)
                {
                    new_port.normals_level =
                        boost::lexical_cast<int>(fields["l"]);
                    new_port.normals_scale =
                        boost::lexical_cast<double>(fields["s"]);
                }
            }
            else
            {
                std::cerr << "Bad port format (type): " << p << '\n';
                continue;
            }
        }
        if (opts.size() > 3)
        {
            boost::smatch fields;
            if (opts[3] == "p")
            {
                new_port.colour.type = PortType::Colour::CT_PLAIN;
            }
            else if (opts[3] == "a")
            {
                new_port.colour.type = PortType::Colour::CT_RANDOM;
            }
            else if (opts[3] == "r")
            {
                new_port.colour.type = PortType::Colour::CT_RGB;
            }
            else if (boost::regex_match(opts[3], fields, colour_re))
            {
                new_port.colour.type = PortType::Colour::CT_CUSTOM;
                new_port.colour.r = boost::lexical_cast<int>(fields["r"]);
                new_port.colour.g = boost::lexical_cast<int>(fields["g"]);
                new_port.colour.b = boost::lexical_cast<int>(fields["b"]);
            }
            else if (boost::regex_match(opts[3], fields, field_re))
            {
                new_port.colour.type = PortType::Colour::CT_FIELD;
                new_port.colour.field = fields["f"];
            }
            else
            {
                std::cerr << "Bad port format (colour): " << p << '\n';
                continue;
            }
        }
        port_types_.push_back(new_port);
    }
}


void RTCPCLViewer::print_port_types()
{
    BOOST_FOREACH(PortType p, port_types_)
    {
        std::cout << "Name: " << p.name << "  Viewport: " << p.viewport <<
            "  Display type: ";
        switch(p.display_type)
        {
            case PortType::DT_POINT:
                std::cout << "Point";
                break;
            case PortType::DT_NORMAL:
                std::cout << "Normal (" << p.normals_level << ", " <<
                    p.normals_scale << ")";
                break;
            case PortType::DT_PRINCUR:
                std::cout << "Principal curves";
                break;
        }
        std::cout << "  Colour: ";
        switch (p.colour.type)
        {
            case PortType::Colour::CT_PLAIN:
                std::cout << "Plain";
                break;
            case PortType::Colour::CT_RANDOM:
                std::cout << "Random";
                break;
            case PortType::Colour::CT_RGB:
                std::cout << "RGB";
                break;
            case PortType::Colour::CT_CUSTOM:
                std::cout << "Custom (" << p.colour.r << "," << p.colour.g <<
                    "," << p.colour.b << ")";
                break;
            case PortType::Colour::CT_FIELD:
                std::cout << "Field (" << p.colour.field << ")";
                break;
        }
        std::cout << '\n';
    }
}


void RTCPCLViewer::create_input_ports()
{
    BOOST_FOREACH(PortType p, port_types_)
    {
        CorbaCloudPtr cloud(new PointCloudTypes::PointCloud());
        CorbaPortPtr port(new CorbaPortType(("corba_" + p.name).c_str(),
                    *cloud));
        CorbaPort corba_port;
        corba_port.port = port;
        corba_port.cloud = cloud;
        corba_port.type = p;
        corba_ports_.push_back(corba_port);
        addInPort(port->getName(), *port);
    }
}


void RTCPCLViewer::create_dds_ports()
{
#if defined(DDS_SUPPORT)
    BOOST_FOREACH(PortType p, port_types_)
    {
        DDSPortPtr port(new DDSPortType(("dds_" + p.name).c_str(),
                    PointCloudTypes_PointCloudTypeSupport::get_type_name()));
        DDSPort dds_port;
        dds_port.port = port;
        dds_port.type = p;
        dds_ports_.push_back(dds_port);
        addDDSSubPort(port->getName(), *port, *this);
        register_pc_type(port->get_participant());
    }
#endif // defined(DDS_SUPPORT)
}


void RTCPCLViewer::create_viewports()
{
    viewport_ids_[0] = 0; // Viewport 0 is all viewports
    // Get the total number of viewports
    int max(0);
    BOOST_FOREACH(PortType p, port_types_)
    {
        if (p.viewport > max)
        {
            max = p.viewport;
        }
    }
    if (max == 0)
    {
        // No/one viewports
        return;
    }
    // Arrange viewports in an approximate grid. Number of rows is
    // floor(sqrt(max)).
    unsigned int r(floorf(sqrtf(max))); // Number of rows
    unsigned int c(floorf(max / r)); // Number of columns
    // Number of overflow viewports (i.e. 5 doesn't fit in 2x2 so we create 2
    // rows with 2 columns and 1 row with 1 column for the overflow viewport)
    unsigned int o(max % c);
    float height(0.0);
    if (o == 0)
    {
        height = 1.0 / r;
    }
    else
    {
        // Add an extra row for the overflow viewports
        height = 1.0 / (r + 1);
    }
    float width(1.0 / c); // Width of standard viewports
    float overflow_width(1.0 / o); // Width of overflow row's viewports

    // Create the standard viewports
    float min_x(0.0), max_x(width), min_y(0.0), max_y(height);
    for (unsigned int ii(0); ii < r; ++ii)
    {
        for (unsigned int jj(0); jj < c; ++jj)
        {
            int id(0);
            viewer_->createViewPort(min_x, min_y, max_x, max_y, id);
            viewport_ids_[ii * c + jj + 1] = id; // +1 because 0 = all
            min_x = max_x;
            max_x += width;
        }
        min_x = 0.0;
        max_x = width;
        min_y = max_y;
        max_y += height;
    }
    // Create the overflow viewports
    if (o > 0)
    {
        min_y = r * height;
        max_y = 1.0;
        min_x = 0.0;
        max_x = overflow_width;
        for (unsigned int ii(0); ii < o; ++ii)
        {
            int id(0);
            viewer_->createViewPort(min_x, min_y, max_x, max_y, id);
            viewport_ids_[r * c + ii + 1] = id; // +1 because 0 = all
            min_x = max_x;
            max_x += overflow_width;
        }
    }

    // Add lables to the viewports
    BOOST_FOREACH(PortType p, port_types_)
    {
        viewer_->addText(p.name, 10, 10, p.name + "_lable",
                viewport_ids_[p.viewport]);
    }
}


static const char* spec[] =
{
    "implementation_id", "RTCPCLViewer",
    "type_name",         "RTCPCLViewer",
    "description",       "Point cloud viewer component",
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
    "conf.default.axes_scale", "0.0",
    "conf.default.ports", "in",
    "conf.default.show_timestamps", "0",
#if defined(DDS_SUPPORT)
    "conf.default.dds_ports", "point_cloud_dds:xyz",
#endif // defined(DDS_SUPPORT)
    // Widget
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.ports", "text",
#if defined(DDS_SUPPORT)
    "conf.__widget__.dds_ports", "text",
#endif // defined(DDS_SUPPORT)
    // Constraints
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCLViewer>,
                RTC::Delete<RTCPCLViewer>);
    }
};

