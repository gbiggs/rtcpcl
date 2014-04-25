/* RTC:PCDBenchmark
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


#include "rtcpcdbenchmark.h"

#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <rtcpcl/pc_mgmt.h>
#if defined(__APPLE__)
    #include <sys/time.h>
#else
    #include <time.h>
#endif // defined(__APPLE__)

#if defined(DDS_SUPPORT)
#include <rtmdds/ddsportmgmt.h>
#endif // defined(DDS_SUPPORT)

using namespace RTCPCL;


RTCPCDBenchmark::RTCPCDBenchmark(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
    corba_in_port_("corba_in", in_cloud_),
    corba_out_port_("corba_out", out_cloud_),
#if defined(DDS_SUPPORT)
    dds_cloud_(0),
    dds_in_port_("dds_in",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
    dds_out_port_("dds_out",
            PointCloudTypes_PointCloudTypeSupport::get_type_name()),
#endif // defined(DDS_SUPPORT)
    seq_(0),
    corba_(true), dds_(true), pointer_(true), write_(true),
    read_(true), echo_(false), echo_fast_(true),
    echoed_clouds_(0), echoed_points_(0), echoed_bytes_(0),
    written_clouds_(0), written_points_(0), written_bytes_(0),
    read_clouds_(0), read_points_(0), read_bytes_(0)
{
}


RTCPCDBenchmark::~RTCPCDBenchmark()
{
}


RTC::ReturnCode_t RTCPCDBenchmark::onInitialize()
{
    bindParameter("pcd_file", filename_, "");
    bindParameter("corba", corba_, "1");
    bindParameter("dds", dds_, "0");
    bindParameter("pointer", pointer_, "0");
    bindParameter("write", write_, "1");
    bindParameter("read", read_, "1");
    bindParameter("echo", echo_, "0");
    bindParameter("echo_fast", echo_fast_, "1");
    addInPort(corba_in_port_.getName(), corba_in_port_);
    addOutPort(corba_out_port_.getName(), corba_out_port_);
#if defined(DDS_SUPPORT)
    addDDSSubPort(dds_in_port_.getName(), dds_in_port_, *this);
    addDDSPubPort(dds_out_port_.getName(), dds_out_port_, *this);
    register_pc_type(dds_in_port_.get_participant());
    register_pc_type(dds_out_port_.get_participant());
#endif // defined(DDS_SUPPORT)

    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::onActivated(RTC::UniqueId ec_id)
{
    seq_ = 0;
    echoed_clouds_ = 0;
    echoed_points_ = 0;
    echoed_bytes_ = 0;
    written_clouds_ = 0;
    written_points_ = 0;
    written_bytes_ = 0;
    read_clouds_ = 0;
    read_points_ = 0;
    read_bytes_ = 0;

    // Load the cloud
    if (filename_ == "")
    {
        std::cerr << "No PCD file given\n";
        return RTC::RTC_ERROR;
    }
    loaded_cloud_.reset(new PCLCloudType);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_, *loaded_cloud_) == -1)
    {
        std::cerr << "Error reading PCD file " << filename_ << '\n';
        return RTC::RTC_ERROR;
    }

    if (echo_)
    {
        echo_times_.open("echo_times.txt");
        echo_times_ << "Start,Read,Unpacked,Packed,Written,Sequence ID\n";
    }
    else
    {
        if (write_)
        {
            write_times_.open("write_times.txt");
            write_times_ << "Sequence ID,Start,Packed,Written\n";
        }
        if (read_)
        {
            read_times_.open("read_times.txt");
            read_times_ << "Start,Read,Unpacked,Sequence ID\n";
        }
    }

#if defined(DDS_SUPPORT)
    dds_cloud_ = PointCloudTypes_PointCloudTypeSupport::create_data();
    if (!dds_cloud_)
    {
        std::cerr << "Failed to create DDS data instance\n";
        return RTC::RTC_ERROR;
    }
    dds_cloud_->data.maximum(0);
#endif // defined(DDS_SUPPORT)

    start_time_ = get_ts();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::onDeactivated(RTC::UniqueId ec_id)
{
    end_time_ = get_ts();
    // Write bandwidth data
    if (echo_)
    {
        std::ofstream echo_bw("echo_bandwidth.txt");
        echo_bw << "Start time,End time,Clouds,Points,"
            "Bytes,Cloud size (points),Cloud size (bytes)\n";
        echo_bw << start_time_ << ',' << end_time_ << ',' << echoed_clouds_ <<
            ',' << echoed_points_ << ',' << echoed_bytes_ << ',' <<
            loaded_cloud_->size() << ',' <<
            loaded_cloud_->size() * sizeof(PCLPointType) << '\n';
        echo_bw.close();
    }
    else
    {
        if (write_)
        {
            std::ofstream write_bw("write_bandwidth.txt");
            write_bw << "Start time,End time,Clouds,Points,"
                "Bytes,Cloud size (points),Cloud size (bytes)\n";
            write_bw << start_time_ << ',' << end_time_ << ',' <<
                written_clouds_ << ',' << written_points_ << ',' <<
                written_bytes_ << ',' << loaded_cloud_->size() << ',' <<
                loaded_cloud_->size() * sizeof(PCLPointType) << '\n';
            write_bw.close();
        }
        if (read_)
        {
            std::ofstream read_bw("read_bandwidth.txt");
            read_bw << "Start time,End time,Clouds,Points,"
                "Bytes,Cloud size (points),Cloud size (bytes)\n";
            read_bw << start_time_ << ',' << end_time_ << ',' <<
                read_clouds_ << ',' << read_points_ << ',' <<
                read_bytes_ << ',' << loaded_cloud_->size() << ',' <<
                loaded_cloud_->size() * sizeof(PCLPointType) << '\n';
            read_bw.close();
        }
    }
#if defined(DDS_SUPPORT)
    PointCloudTypes_PointCloudTypeSupport::delete_data(dds_cloud_);
#endif // defined(DDS_SUPPORT)
    // Destroy the loaded cloud
    loaded_cloud_.reset();
    // Close benchmark data files
    echo_times_.close();
    write_times_.close();
    read_times_.close();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::onExecute(RTC::UniqueId ec_id)
{
    RTC::ReturnCode_t ret(RTC::RTC_OK);
    if (corba_)
    {
        if ((ret = do_corba_latency()) != RTC::RTC_OK)
        {
            return ret;
        }
    }
    if (dds_)
    {
        if ((ret = do_dds_latency()) != RTC::RTC_OK)
        {
            return ret;
        }
    }
    if (pointer_)
    {
        if ((ret = do_pointer_latency()) != RTC::RTC_OK)
        {
            return ret;
        }
    }
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::do_corba_latency()
{
    if (echo_)
    {
        if (corba_in_port_.isNew())
        {
            write_ts(echo_times_, ',');
            corba_in_port_.read();
            write_ts(echo_times_, ',');
            //std::cerr << "Echo has read " << in_cloud_.seq << '\n';
            if (!echo_fast_)
            {
                // Unpack and repack the point cloud
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(in_cloud_, unpacked);
                write_ts(echo_times_, ',');
                pcl_to_pointcloud<PCLPointType>(unpacked, out_cloud_);
                write_ts(echo_times_, ',');
                corba_out_port_.write();
            }
            else
            {
                // Send back the received point cloud as-is
                write_ts(echo_times_, ',');
                write_ts(echo_times_, ',');
                corba_out_port_.write(in_cloud_);
            }
            write_ts(echo_times_, ',');
            echo_times_ << in_cloud_.seq << '\n';
            ++echoed_clouds_;
            echoed_points_ += in_cloud_.height * in_cloud_.width;
            echoed_bytes_ += in_cloud_.data.length();
            //std::cerr << "Echo has written " << out_cloud_.seq << '\n';
        }
    }
    else
    {
        if (write_)
        {
            write_times_ << seq_ << ',';
            loaded_cloud_->header.stamp = get_ts();
            loaded_cloud_->header.seq = seq_++;
            write_ts(write_times_, ',');
            pcl_to_pointcloud<PCLPointType>(loaded_cloud_, out_cloud_);
            write_ts(write_times_, ',');
            corba_out_port_.write();
            write_ts(write_times_, '\n');
            ++written_clouds_;
            written_points_ += loaded_cloud_->size();
            written_bytes_ += loaded_cloud_->size() * sizeof(PCLPointType);
            //std::cerr << "Write has written " << out_cloud_.seq << '\n';
        }
        if (read_)
        {
            if (corba_in_port_.isNew())
            {
                write_ts(read_times_, ',');
                corba_in_port_.read();
                write_ts(read_times_, ',');
                //std::cerr << "Reader has read " << in_cloud_.seq << '\n';
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(in_cloud_, unpacked);
                write_ts(read_times_, ',');
                read_times_ << in_cloud_.seq << '\n';
                ++read_clouds_;
                read_points_ += unpacked->size();
                read_bytes_ += unpacked->size() * sizeof(PCLPointType);
            }
            else
            {
                // Write time stamps to balance the line
                write_ts(read_times_, ',');
                write_ts(read_times_, ',');
                write_ts(read_times_, ',');
                read_times_ << -1 << '\n';
            }
        }
    }
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::do_dds_latency()
{
#if defined(DDS_SUPPORT)
    if (echo_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        write_ts(echo_times_, ',');
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            write_ts(echo_times_, ',');
            //std::cerr << "Echo has read " << in_cloud->seq << '\n';
            if (!echo_fast_)
            {
                // Unpack and repack the point cloud
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(clouds[0], unpacked);
                write_ts(echo_times_, ',');
                PointCloudTypes_PointCloud* out_cloud =
                    PointCloudTypes_PointCloudTypeSupport::create_data();
                if (!out_cloud)
                {
                    std::cerr << "Failed to create DDS data instance\n";
                    return RTC::RTC_ERROR;
                }
                pcl_to_pointcloud<PCLPointType>(unpacked, *out_cloud, false);
                write_ts(echo_times_, ',');
                dds_out_port_.write(*out_cloud);
                //return_loan(*out_cloud);
                PointCloudTypes_PointCloudTypeSupport::delete_data(out_cloud);
            }
            else
            {
                // Send back the received point cloud as-is
                write_ts(echo_times_, ','); // Unpack time
                write_ts(echo_times_, ','); // Pack time
                dds_out_port_.write(clouds[0]);
            }
            write_ts(echo_times_, ',');
            echo_times_ << clouds[0].seq << '\n';
            //std::cerr << "Echo has written " << out_cloud->seq << '\n';
            ++echoed_clouds_;
            echoed_points_ += clouds[0].height * clouds[0].width;
            echoed_bytes_ += clouds[0].data.length();
            dds_in_port_.return_loan(clouds, info);
        }
        else
        {
            // Write time stamps to balance the line
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            echo_times_ << -1 << '\n';
        }
    }
    else
    {
        if (write_)
        {
            write_times_ << seq_ << ',';
            loaded_cloud_->header.stamp = get_ts();
            loaded_cloud_->header.seq = seq_++;
            write_ts(write_times_, ',');
            pcl_to_pointcloud<PCLPointType>(loaded_cloud_, *dds_cloud_, true);
            write_ts(write_times_, ',');
            dds_out_port_.write(*dds_cloud_);
            write_ts(write_times_, '\n');
            //std::cerr << "Write has written " << dds_cloud->seq << '\n';
            ++written_clouds_;
            written_points_ += loaded_cloud_->size();
            written_bytes_ += loaded_cloud_->size() * sizeof(PCLPointType);
            return_loan(*dds_cloud_);
        }
        if (read_)
        {
            PointCloudTypes_PointCloudSeq clouds;
            DDS_SampleInfoSeq info;
            write_ts(read_times_, ',');
            if (dds_in_port_.read_with_loan(clouds, info))
            {
                write_ts(read_times_, ',');
                //std::cerr << "Reader has read " << clouds[0]->seq << '\n';
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(clouds[0], unpacked);
                write_ts(read_times_, ',');
                read_times_ << clouds[0].seq << '\n';
                ++read_clouds_;
                read_points_ += unpacked->size();
                read_bytes_ += unpacked->size() * sizeof(PCLPointType);
                dds_in_port_.return_loan(clouds, info);
            }
            else
            {
                // Write time stamps to balance the line
                write_ts(read_times_, ',');
                write_ts(read_times_, ',');
                read_times_ << -1 << '\n';
            }
        }
    }
#endif // defined(DDS_SUPPORT)
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RTCPCDBenchmark::do_pointer_latency()
{
#if defined(RTMPOINTER_SUPPORT)
    if (echo_)
    {
        PointCloudTypes_PointCloudSeq clouds;
        DDS_SampleInfoSeq info;
        write_ts(echo_times_, ',');
        if (dds_in_port_.read_with_loan(clouds, info))
        {
            write_ts(echo_times_, ',');
            //std::cerr << "Echo has read " << in_cloud->seq << '\n';
            if (!echo_fast_)
            {
                // Unpack and repack the point cloud
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(clouds[0], unpacked);
                write_ts(echo_times_, ',');
                PCLCloudType::ConstPtr unpacked_const(unpacked);
                PointCloudTypes_PointCloud* out_cloud =
                    PointCloudTypes_PointCloudTypeSupport::create_data();
                if (!out_cloud)
                {
                    std::cerr << "Failed to create DDS data instance\n";
                    return RTC::RTC_ERROR;
                }
                pcl_to_pointcloud<PCLPointType>(unpacked_const, *out_cloud, false);
                write_ts(echo_times_, ',');
                dds_out_port_.write(*out_cloud);
                //return_loan(*out_cloud);
                PointCloudTypes_PointCloudTypeSupport::delete_data(out_cloud);
            }
            else
            {
                // Send back the received point cloud as-is
                write_ts(echo_times_, ','); // Unpack time
                write_ts(echo_times_, ','); // Pack time
                dds_out_port_.write(clouds[0]);
            }
            write_ts(echo_times_, ',');
            echo_times_ << clouds[0].seq << '\n';
            //std::cerr << "Echo has written " << out_cloud->seq << '\n';
            ++echoed_clouds_;
            echoed_points_ += clouds[0].height * clouds[0].width;
            echoed_bytes_ += clouds[0].data.length();
            dds_in_port_.return_loan(clouds, info);
        }
        else
        {
            // Write time stamps to balance the line
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            write_ts(echo_times_, ',');
            echo_times_ << -1 << '\n';
        }
    }
    else
    {
        if (write_)
        {
            write_times_ << seq_ << ',';
            loaded_cloud_->header.stamp = get_ts();
            loaded_cloud_->header.seq = seq_++;
            PCLCloudType::ConstPtr unpacked(loaded_cloud_);
            write_ts(write_times_, ',');
            pcl_to_pointcloud<PCLPointType>(unpacked, *dds_cloud_, true);
            write_ts(write_times_, ',');
            dds_out_port_.write(*dds_cloud_);
            write_ts(write_times_, '\n');
            //std::cerr << "Write has written " << dds_cloud->seq << '\n';
            ++written_clouds_;
            written_points_ += unpacked->size();
            written_bytes_ += unpacked->size() * sizeof(PCLPointType);
            return_loan(*dds_cloud_);
        }
        if (read_)
        {
            PointCloudTypes_PointCloudSeq clouds;
            DDS_SampleInfoSeq info;
            write_ts(read_times_, ',');
            if (dds_in_port_.read_with_loan(clouds, info))
            {
                write_ts(read_times_, ',');
                //std::cerr << "Reader has read " << clouds[0]->seq << '\n';
                PCLCloudType::Ptr unpacked(new PCLCloudType);
                pointcloud_to_pcl<pcl::PointXYZ>(clouds[0], unpacked);
                write_ts(read_times_, ',');
                read_times_ << clouds[0].seq << '\n';
                ++read_clouds_;
                read_points_ += unpacked->size();
                read_bytes_ += unpacked->size() * sizeof(PCLPointType);
                dds_in_port_.return_loan(clouds, info);
            }
            else
            {
                // Write time stamps to balance the line
                write_ts(read_times_, ',');
                write_ts(read_times_, ',');
                read_times_ << -1 << '\n';
            }
        }
    }
#endif // defined(RTMPOINTER_SUPPORT)
    return RTC::RTC_OK;
}


uint64_t RTCPCDBenchmark::get_ts()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000000000 + ts.tv_nsec;
}


uint64_t RTCPCDBenchmark::write_ts(std::ofstream& dest, char suffix)
{
    uint64_t timestamp(get_ts());
    dest << timestamp << suffix;
    return timestamp;
}


static const char* spec[] =
{
    "implementation_id", "RTCPCDBenchmark",
    "type_name",         "RTCPCDBenchmark",
    "description",       "Transport benchmark component using point clouds",
    "version",           "1.0",
    "vendor",            "Geoffrey Biggs, AIST",
    "category",          "Test",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.pcd_file", "",
    "conf.default.corba", "1",
    "conf.default.dds", "0",
    "conf.default.pointer", "0",
    "conf.default.write", "1",
    "conf.default.read", "1",
    "conf.default.echo", "0",
    "conf.default.echo_fast", "1",
    // Widget
    "conf.__widget__.pcd_file", "text",
    "conf.__widget__.corba", "radio",
    "conf.__widget__.dds", "radio",
    "conf.__widget__.pointer", "radio",
    "conf.__widget__.write", "radio",
    "conf.__widget__.read", "radio",
    "conf.__widget__.echo", "radio",
    "conf.__widget__.echo_fast", "radio",
    // Constraints
    ""
};

extern "C"
{
    void rtc_init(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile, RTC::Create<RTCPCDBenchmark>,
                RTC::Delete<RTCPCDBenchmark>);
    }
};

