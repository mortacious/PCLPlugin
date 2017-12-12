//
// Created by figelbrink on 12/12/17.
//

#pragma once
#ifndef PARAVIEW_PCLPLUGINS_VTKTYPEFROMPCLTYPE_H
#define PARAVIEW_PCLPLUGINS_VTKTYPEFROMPCLTYPE_H
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>

#include <vtkIntArray.h>
#include <vtkUnsignedIntArray.h>

#include <vtkCharArray.h>
#include <vtkUnsignedCharArray.h>

#include <vtkShortArray.h>
#include <vtkUnsignedShortArray.h>

#include <pcl/point_types.h>
#include <type_traits>
/*
 * Choose a base type from the vtk type
 */
template <typename vtkArrayType, class = void>
struct BaseTypeFromVtkArrayType;

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkCharArray>::value>> {
    using type = char;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkUnsignedCharArray>::value>> {
    using type = std::uint8_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkShortArray>::value>> {
    using type = std::int16_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkUnsignedShortArray>::value>> {
    using type = std::uint16_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkIntArray>::value>> {
    using type = std::int32_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkUnsignedIntArray>::value>> {
    using type = std::uint32_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkFloatArray>::value>> {
    using type = float;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkDoubleArray>::value>> {
    using type = double;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};
//----------------------------------------------------------------------------------------------------------------------
template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<char>>::value>> {
    using type = std::int8_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<signed char>>::value>> {
    using type = std::int8_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<unsigned char>>::value>> {
    using type = std::uint8_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<short int>>::value>> {
    using type = std::int16_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<unsigned short int>>::value>> {
    using type = double;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<int>>::value>> {
    using type = std::int32_t ;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<unsigned int>>::value>> {
    using type = std::uint32_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<float>>::value>> {
    using type = float ;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<double>>::value>> {
    using type = double;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<long int>>::value>> {
    using type = std::int32_t ;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<unsigned long int>>::value>> {
    using type = std::uint32_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<long long int>>::value>> {
    using type = std::int64_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

template <typename vtkArrayType>
struct BaseTypeFromVtkArrayType<vtkArrayType, std::enable_if_t<std::is_same<vtkArrayType, vtkAOSDataArrayTemplate<unsigned long long int>>::value>> {
    using type = std::uint64_t;
    static const pcl::PCLPointField::PointFieldTypes pcl_type;
};

// set constants
template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkCharArray>::pcl_type = pcl::PCLPointField::INT8;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkUnsignedCharArray>::pcl_type = pcl::PCLPointField::UINT8;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkShortArray>::pcl_type = pcl::PCLPointField::INT16;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkUnsignedShortArray>::pcl_type = pcl::PCLPointField::UINT16;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkIntArray>::pcl_type = pcl::PCLPointField::INT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkUnsignedIntArray>::pcl_type = pcl::PCLPointField::UINT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkFloatArray>::pcl_type = pcl::PCLPointField::FLOAT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkDoubleArray>::pcl_type = pcl::PCLPointField::FLOAT64;
//------------------------------------------------------------------------------------------------------------------------
template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<char>>::pcl_type = pcl::PCLPointField::INT8;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<signed char>>::pcl_type = pcl::PCLPointField::INT8;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<unsigned char>>::pcl_type = pcl::PCLPointField::UINT8;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<short int>>::pcl_type = pcl::PCLPointField::INT16;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<unsigned short int>>::pcl_type = pcl::PCLPointField::UINT16;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType< vtkAOSDataArrayTemplate<int>>::pcl_type = pcl::PCLPointField::INT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<unsigned int>>::pcl_type = pcl::PCLPointField::UINT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<float>>::pcl_type = pcl::PCLPointField::FLOAT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<double>>::pcl_type = pcl::PCLPointField::FLOAT64;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<long int>>::pcl_type = pcl::PCLPointField::INT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<unsigned long int>>::pcl_type = pcl::PCLPointField::UINT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<long long int>>::pcl_type = pcl::PCLPointField::INT32;

template<>
const pcl::PCLPointField::PointFieldTypes BaseTypeFromVtkArrayType<vtkAOSDataArrayTemplate<unsigned long long int>>::pcl_type = pcl::PCLPointField::UINT32;

template<typename T>
using BaseType = typename BaseTypeFromVtkArrayType<T>::type;


#endif //PARAVIEW_PCLPLUGINS_VTKTYPEFROMPCLTYPE_H
