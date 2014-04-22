/* RTM:PCL Point Cloud Type
 *
 * Header file for point cloud management functions. Based on the PCL-to-ROS
 * conversion functions from PCL 1.0, in particular conversions.h.
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

#if !defined(PC_MGMT_H)
#define PC_MGMT_H

#include <rtcpcl/config.h>
#include <rtcpcl/pc_traits.h>

#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/foreach.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/preprocessor/control/iif.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <cstring>
#include <pcl/PCLPointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/for_each_type.h>
#include <pointcloud.hh> // CORBA
#include <string>
#if defined(__APPLE__)
    #include <sys/time.h>
#endif // defined(__APPLE__)

#if defined(DDS_SUPPORT)
#include <pointcloud.h> // DDS
#include <ndds/ndds_cpp.h>
#endif // defined(DDS_SUPPORT)

/*void write_diff(std::string title, uint64_t start, uint64_t end)
{
    std::cerr << title << end - start << '\n';
}*/

namespace RTCPCL
{
    uint64_t get_ts()
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return ts.tv_sec * 1000000000 + ts.tv_nsec;
    }


    template<typename PointT, typename FieldsType, typename ItrType>
    struct FieldMapper;

    template<typename PointT, typename FieldsType, typename ItrType>
    void CreateFieldMapping(FieldsType const& fields, pcl::MsgFieldMap& field_map);


    ///////////////////////////////////////////////////////////////////////////
    // CORBA data type management
    ///////////////////////////////////////////////////////////////////////////
#if defined(CORBA_SUPPORT)

    /// Sets the field information in a PointCloudTypes::PointFieldList.
    template<typename PointT>
    struct CORBAFieldAdder
    {
        CORBAFieldAdder(PointCloudTypes::PointFieldList& fields)
            : fields_(fields)
        {}

        template<typename Tag>
        void operator()()
        {
            CORBA::ULong cur_len(fields_.length());
            fields_.length(cur_len + 1);
            fields_[cur_len].name = CORBA::string_dup(pcl::traits::name<PointT,
                    Tag>::value);
            fields_[cur_len].offset = pcl::traits::offset<PointT, Tag>::value;
            fields_[cur_len].count = pcl::traits::datatype<PointT, Tag>::size;
            // Icky switch because I don't have time to write a type trait
            // right now
            switch(pcl::traits::datatype<PointT, Tag>::value)
            {
                case pcl::PCLPointField::INT8:
                    fields_[cur_len].data_type = PointCloudTypes::INT8;
                    break;
                case pcl::PCLPointField::UINT8:
                    fields_[cur_len].data_type = PointCloudTypes::UINT8;
                    break;
                case pcl::PCLPointField::INT16:
                    fields_[cur_len].data_type = PointCloudTypes::INT16;
                    break;
                case pcl::PCLPointField::UINT16:
                    fields_[cur_len].data_type = PointCloudTypes::UINT16;
                    break;
                case pcl::PCLPointField::INT32:
                    fields_[cur_len].data_type = PointCloudTypes::INT32;
                    break;
                case pcl::PCLPointField::UINT32:
                    fields_[cur_len].data_type = PointCloudTypes::UINT32;
                    break;
                case pcl::PCLPointField::FLOAT32:
                    fields_[cur_len].data_type = PointCloudTypes::FLOAT32;
                    break;
                case pcl::PCLPointField::FLOAT64:
                    fields_[cur_len].data_type = PointCloudTypes::FLOAT64;
                    break;
            }
        }

        PointCloudTypes::PointFieldList& fields_;
    };


    bool point_field_types_equal(pcl::uint8_t pcl_type,
            PointCloudTypes::DataType corba_type)
    {
        switch(pcl_type)
        {
            case pcl::PCLPointField::INT8:
                if (corba_type == PointCloudTypes::INT8)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT8:
                if (corba_type == PointCloudTypes::UINT8)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::INT16:
                if (corba_type == PointCloudTypes::INT16)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT16:
                if (corba_type == PointCloudTypes::UINT16)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::INT32:
                if (corba_type == PointCloudTypes::INT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT32:
                if (corba_type == PointCloudTypes::UINT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::FLOAT32:
                if (corba_type == PointCloudTypes::FLOAT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::FLOAT64:
                if (corba_type == PointCloudTypes::FLOAT64)
                {
                    return true;
                }
                break;
        }
        return false;
    }


    /// Get the field names from a PointCloudTypes::PointCloud as a string.
    ///
    /// @param cloud The cloud to get the types from.
    /*std::string get_cloud_type(PointCloudTypes::PointCloud& cloud)
    {
        std::string result;
        for(CORBA::ULong ii(0); ii < cloud.fields.length(); ++ii)
        {
            result += cloud.fields[ii].name;
        }
        return result;
    }*/


    /// Conversion from pcl::PointCloud to PointCloudTypes::PointCloud.
    ///
    /// Converts the data in a pcl::PointCloud structure to a
    /// PointCloudTypes::PointCloud structure.
    /// @param src The source pcl::PointCloud structure.
    /// @param dest The destination structure.
    template<typename PointT>
    void pcl_to_pointcloud(typename pcl::PointCloud<PointT>::ConstPtr src,
            PointCloudTypes::PointCloud& dest)
    {
        //uint64_t start = get_ts();
        // Sanity check
        if (src->points.size() != src->width * src->height)
        {
            throw std::runtime_error("Cloud width/height not equal to cloud "
                    "size");
        }
        // Copy basic cloud information
        if (src->width == 0 && src->height == 0)
        {
            dest.width = src->size();
            dest.height = 1;
        }
        else
        {
            dest.width = src->width;
            dest.height = src->height;
        }
        dest.point_step = sizeof(PointT);
        dest.row_step = dest.point_step * dest.width;
        dest.is_bigendian = RTCPCL_IS_BIG_ENDIAN;
        dest.is_dense = src->is_dense;
        dest.tm.sec = src->header.stamp / 1000000000;
        dest.tm.nsec = src->header.stamp % 1000000000;
        dest.seq = src->header.seq;
        dest.type = CORBA::string_dup(PointName<PointT>::value.c_str());

        // Set field information
        dest.fields.length(0);
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(
                CORBAFieldAdder<PointT>(dest.fields));

        // Copy the binary data
        dest.data.length(src->size() * dest.point_step);
        memcpy(dest.data.get_buffer(false), &(src->points[0]), dest.data.length());
        //uint64_t end = get_ts();
        //write_diff("CORBA::pcl_to_pointcloud() time taken: ", start, end);
    }

    /// Conversion from PointCloudTypes::PointCloud to pcl::PointCloud.
    ///
    /// Converts the data in a PointCloudTypes::PointCloud to a pcl::PointCloud
    /// structure for use with the PCL library.
    /// @param src The source data structure, usually received over a
    ///     transport.
    /// @param dest The destination PCL point cloud.
    template<typename PointT>
    void pointcloud_to_pcl(PointCloudTypes::PointCloud const& src,
            typename pcl::PointCloud<PointT>::Ptr& dest)
    {
        pcl::MsgFieldMap field_map;
        CreateFieldMapping<PointT, PointCloudTypes::PointFieldList, CORBA::ULong>(
                src.fields, field_map);

        if (src.is_bigendian != RTCPCL_IS_BIG_ENDIAN)
        {
            throw std::runtime_error("Endianess mismatch");
        }

        // Copy basic cloud information
        dest->width = src.width;
        dest->height = src.height;
        dest->is_dense = src.is_dense;
        dest->header.seq = src.seq;
        // Combine the time stamp fields in steps to avoid overflowing
        dest->header.stamp = src.tm.sec;
        dest->header.stamp *= 1000000000;
        dest->header.stamp += src.tm.nsec;

        // Copy point data
        unsigned int num_points = src.width * src.height;
        dest->points.resize(num_points);
        uint8_t* dest_data(reinterpret_cast<uint8_t*>(&dest->points[0]));
        // Copy adjacent fields together, non-adjacent fields individually
        if (field_map.size() == 1 && field_map[0].serialized_offset == 0 &&
                field_map[0].struct_offset == 0 &&
                src.point_step == sizeof(PointT))
        {
            unsigned int dest_row_step = sizeof(PointT) * dest->width;
            uint8_t const* src_data(src.data.get_buffer());
            if (src.row_step == dest_row_step)
            {
                // Copy all rows at once - yay!
                memcpy(dest_data, src_data, src.data.length());
            }
            else
            {
                // Copy row-by-row
                for (unsigned int ii(0); ii < dest->height;
                    ++ii, dest_data += dest_row_step, src_data += src.row_step)
                {
                    memcpy(dest_data, src_data, dest_row_step);
                }
            }
        }
        else
        {
            for (unsigned int row(0); row < src.height; ++row)
            {
                uint8_t const* row_data(
                        &(src.data.get_buffer()[row * src.row_step]));
                for (unsigned int col(0); col < src.width; ++col)
                {
                    uint8_t const* pt_data(row_data + col * src.point_step);
                    BOOST_FOREACH(pcl::detail::FieldMapping const& mapping,
                            field_map)
                    {
                        memcpy(dest_data + mapping.struct_offset,
                                pt_data + mapping.serialized_offset,
                                mapping.size);
                    }
                    dest_data += sizeof(PointT);
                }
            }
        }
    }
#endif // defined(CORBA_SUPPORT)

    ///////////////////////////////////////////////////////////////////////////
    // DDS data type management
    ///////////////////////////////////////////////////////////////////////////
#if defined(DDS_SUPPORT)
    /// Register the point cloud data type with a DDS domain participant.
    void register_pc_type(DDSDomainParticipant* participant);

    /// Sets the field information in a PointCloudTypes_PointFieldList.
    template<typename PointT>
    struct DDSFieldAdder
    {
        DDSFieldAdder(PointCloudTypes_PointFieldSeq& fields)
            : fields_(fields)
        {}

        template<typename Tag>
        void operator()()
        {
            DDS_Long cur_len(fields_.length());
            fields_.length(cur_len + 1);
            fields_[cur_len].name = DDS_String_dup(pcl::traits::name<PointT,
                    Tag>::value);
            fields_[cur_len].offset = pcl::traits::offset<PointT, Tag>::value;
            fields_[cur_len].count = pcl::traits::datatype<PointT, Tag>::size;
            // Icky switch because I don't have time to write a type trait
            // right now
            switch(pcl::traits::datatype<PointT, Tag>::value)
            {
                case pcl::PCLPointField::INT8:
                    fields_[cur_len].data_type = INT8;
                    break;
                case pcl::PCLPointField::UINT8:
                    fields_[cur_len].data_type = UINT8;
                    break;
                case pcl::PCLPointField::INT16:
                    fields_[cur_len].data_type = INT16;
                    break;
                case pcl::PCLPointField::UINT16:
                    fields_[cur_len].data_type = UINT16;
                    break;
                case pcl::PCLPointField::INT32:
                    fields_[cur_len].data_type = INT32;
                    break;
                case pcl::PCLPointField::UINT32:
                    fields_[cur_len].data_type = UINT32;
                    break;
                case pcl::PCLPointField::FLOAT32:
                    fields_[cur_len].data_type = FLOAT32;
                    break;
                case pcl::PCLPointField::FLOAT64:
                    fields_[cur_len].data_type = FLOAT64;
                    break;
            }
        }

        PointCloudTypes_PointFieldSeq& fields_;
    };


    bool point_field_types_equal(pcl::uint8_t pcl_type,
            PointCloudTypes_DataType dds_type)
    {
        switch(pcl_type)
        {
            case pcl::PCLPointField::INT8:
                if (dds_type == INT8)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT8:
                if (dds_type == UINT8)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::INT16:
                if (dds_type == INT16)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT16:
                if (dds_type == UINT16)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::INT32:
                if (dds_type == INT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::UINT32:
                if (dds_type == UINT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::FLOAT32:
                if (dds_type == FLOAT32)
                {
                    return true;
                }
                break;
            case pcl::PCLPointField::FLOAT64:
                if (dds_type == FLOAT64)
                {
                    return true;
                }
                break;
        }
        return false;
    }


    /// Get the field names from a PointCloudTypes_PointCloud as a string.
    ///
    /// @param cloud The cloud to get the types from.
    /*std::string get_cloud_type(PointCloudTypes_PointCloud& cloud)
    {
        std::string result;
        for(DDS_Long ii(0); ii < cloud.fields.length(); ++ii)
        {
            result += cloud.fields[ii].name;
        }
        return result;
    }*/


    /// Conversion from pcl::PointCloud to PointCloudTypes_PointCloud.
    ///
    /// Converts the data in a pcl::PointCloud structure to a
    /// PointCloudTypes_PointCloud structure.
    /// @param src The source pcl::PointCloud structure.
    /// @param dest The destination structure.
    /// @param loan If the memory space containing the points should be loaned
    /// from the pcl::PointCloud instance to the PointCloudTypes_PointCloud
    /// instance. Care *must* be taken when setting this to true: remember to
    /// call @ref return_loan before deallocating @ref dest, and *do not* set
    /// @ref loan to true if @ref dest will be used with an asynchronous
    /// writer.
    template<typename PointT>
    void pcl_to_pointcloud(typename pcl::PointCloud<PointT>::ConstPtr src,
            PointCloudTypes_PointCloud& dest, bool loan=false)
    {
        //uint64_t start = get_ts();
        // Sanity check
        if (src->points.size() != src->width * src->height)
        {
            throw std::runtime_error("Cloud width/height not equal to cloud "
                    "size");
        }
        // Copy basic cloud information
        if (src->width == 0 && src->height == 0)
        {
            dest.width = src->size();
            dest.height = 1;
        }
        else
        {
            dest.width = src->width;
            dest.height = src->height;
        }
        dest.point_step = sizeof(PointT);
        dest.row_step = dest.point_step * dest.width;
        dest.is_bigendian = RTCPCL_IS_BIG_ENDIAN;
        dest.is_dense = src->is_dense;
        dest.tm.sec = src->header.stamp / 1000000000;
        dest.tm.nsec = src->header.stamp % 1000000000;
        dest.seq = src->header.seq;
        dest.type = DDS_String_dup(PointName<PointT>::value.c_str());

        // Set field information
        dest.fields.length(0);
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(
                DDSFieldAdder<PointT>(dest.fields));

        if (loan)
        {
            // Loan the binary data
            //uint64_t max_start = get_ts();
            //dest.data.maximum(0);
            //uint64_t max_end = get_ts();
            //write_diff("DDS::pcl_to_pointcloud() maximum time taken: ", max_start, max_end);
            //uint64_t loan_start = get_ts();
            dest.data.loan_contiguous(const_cast<DDS_Octet*>(
                        reinterpret_cast<DDS_Octet const*>(&src->points[0])),
                    src->size() * dest.point_step, src->size() * dest.point_step);
            //uint64_t loan_end = get_ts();
            //write_diff("DDS::pcl_to_pointcloud() loan time taken: ", loan_start, loan_end);
        }
        else
        {
            // Copy the binary data
            dest.data.from_array(reinterpret_cast<DDS_Octet const*>(&src->points[0]),
                    dest.data.length());
        }
        //uint64_t end = get_ts();
        //write_diff("DDS::pcl_to_pointcloud() time taken: ", start, end);
    }

    /// Return the memory loaned to a PointCloudTypes_PointCloud structure.
    ///
    /// This function *must* be called when the structure is finished with if
    /// @ref pcl_to_pointcloud was called with @ref loan set to true. It *must*
    /// be called before deallocating the structure.
    void return_loan(PointCloudTypes_PointCloud& cload);

    /// Conversion from PointCloudTypes_PointCloud to pcl::PointCloud.
    ///
    /// Converts the data in a PointCloudTypes_PointCloud to a pcl::PointCloud
    /// structure for use with the PCL library.
    /// @param src The source data structure, usually received over a
    ///     transport.
    /// @param dest The destination PCL point cloud.
    template<typename PointT>
    void pointcloud_to_pcl(PointCloudTypes_PointCloud const& src,
            typename pcl::PointCloud<PointT>::Ptr& dest)
    {
        pcl::MsgFieldMap field_map;
        CreateFieldMapping<PointT, PointCloudTypes_PointFieldSeq, DDS_Long>(
                src.fields, field_map);

        if (src.is_bigendian != RTCPCL_IS_BIG_ENDIAN)
        {
            throw std::runtime_error("Endianess mismatch");
        }

        // Copy basic cloud information
        dest->width = src.width;
        dest->height = src.height;
        dest->is_dense = src.is_dense;
        dest->header.seq = src.seq;
        // Combine the time stamp fields in steps to avoid overflowing
        dest->header.stamp = src.tm.sec;
        dest->header.stamp *= 1000000000;
        dest->header.stamp += src.tm.nsec;

        // Copy point data
        unsigned int num_points = src.width * src.height;
        dest->points.resize(num_points);
        uint8_t* dest_data(reinterpret_cast<uint8_t*>(&dest->points[0]));
        // Copy adjacent fields together, non-adjacent fields individually
        if (field_map.size() == 1 && field_map[0].serialized_offset == 0 &&
                field_map[0].struct_offset == 0 &&
                src.point_step == sizeof(PointT))
        {
            unsigned int dest_row_step = sizeof(PointT) * dest->width;
            uint8_t const* src_data(src.data.get_contiguous_buffer());
            if (src.row_step != dest_row_step)
            {
                // Copy all rows at once - yay!
                memcpy(dest_data, src_data, src.data.length());
            }
            else
            {
                // Copy row-by-row
                for (unsigned int ii(0); ii < dest->height;
                    ++ii, dest_data += dest_row_step, src_data += src.row_step)
                {
                    memcpy(dest_data, src_data, dest_row_step);
                }
            }
        }
        else
        {
            for (unsigned int row(0); row < src.height; ++row)
            {
                uint8_t const* row_data(&(src.data.get_contiguous_buffer()[
                            row * src.row_step]));
                for (unsigned int col(0); col < src.width; ++col)
                {
                    uint8_t const* pt_data(row_data + col * src.point_step);
                    BOOST_FOREACH(pcl::detail::FieldMapping const& mapping,
                            field_map)
                    {
                        memcpy(dest_data + mapping.struct_offset,
                                pt_data + mapping.serialized_offset,
                                mapping.size);
                    }
                    dest_data += sizeof(PointT);
                }
            }
        }
    }
#endif // defined(DDS_SUPPORT)

    template<typename PointT, typename FieldsType, typename ItrType>
    struct FieldMapper
    {
        FieldMapper(FieldsType const& fields,
                pcl::MsgFieldMap& map)
            : fields_(fields), map_(map)
        {}

        template<typename Tag>
        void operator()()
        {
            const char* name = pcl::traits::name<PointT, Tag>::value;
            for(ItrType ii(0); ii < fields_.length(); ++ii)
            {
                if (strcmp(fields_[ii].name, name) == 0)
                {
                    if (!point_field_types_equal(
                                pcl::traits::datatype<PointT, Tag>::value,
                            fields_[ii].data_type))
                    {
                        throw std::runtime_error("Incompatible point field "
                                "type");
                    }
                    pcl::detail::FieldMapping mapping;
                    mapping.serialized_offset = fields_[ii].offset;
                    mapping.struct_offset = pcl::traits::offset<PointT, Tag>::value;
                    mapping.size = sizeof(
                            typename pcl::traits::datatype<PointT, Tag>::type);
                    map_.push_back(mapping);
                    return;
                }
            }
            throw std::runtime_error("Unknown point field in conversion: " +
                    std::string(name));
        }

        FieldsType const& fields_;
        pcl::MsgFieldMap& map_;
    };


    template<typename PointT, typename FieldsType, typename ItrType>
    void CreateFieldMapping(FieldsType const& fields, pcl::MsgFieldMap& field_map)
    {
        // Mapping between CORBA data segments and PCL struct fields
        FieldMapper<PointT, FieldsType, ItrType> mapper(fields, field_map);
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(mapper);

        // Collapse adjacent fields to minimise the number of memcpy calls
        if (field_map.size() > 1)
        {
            std::sort(field_map.begin(), field_map.end(),
                    pcl::detail::fieldOrdering);
            pcl::MsgFieldMap::iterator ii(field_map.begin());
            pcl::MsgFieldMap::iterator jj(ii + 1);
            while (jj != field_map.end())
            {
                // Allow padding between adjacent fields
                if (jj->serialized_offset - ii->serialized_offset ==
                        jj->struct_offset - ii->struct_offset)
                {
                    ii->size += (jj->struct_offset + jj->size) -
                        (ii->struct_offset + ii->size);
                    jj = field_map.erase(jj);
                }
                else
                {
                    ++ii;
                    ++jj;
                }
            }
        }
    }
}; // namespace RTCPCL

#endif // !defined(PC_MGMT_H)

