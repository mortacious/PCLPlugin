/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkAnnotateOBBs.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkPCLConversions.h"

#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkTimerLog.h>
#include <vtkNew.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDataArrayAccessor.h>
#include <vtkArrayDispatch.h>

#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>

#include <vtkIntArray.h>
#include <vtkUnsignedIntArray.h>

#include <vtkCharArray.h>
#include <vtkUnsignedCharArray.h>

#include <vtkShortArray.h>
#include <vtkUnsignedShortArray.h>

#include <vtkPointData.h>

#include <pcl/io/pcd_io.h>
#include <pcl/cloud_iterator.h>

#include <cassert>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <vtkUnstructuredGrid.h>

#include "baseTypeFromVtkArrayType.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLConversions);

//----------------------------------------------------------------------------
vtkPCLConversions::vtkPCLConversions() {
}

//----------------------------------------------------------------------------
vtkPCLConversions::~vtkPCLConversions() {
}

namespace {
    template<typename ArrayType>
    vtkSmartPointer<ArrayType>
    addPolyDataChannelFromPointCloud(pcl::PCLPointCloud2ConstPtr cloud, const pcl::PCLPointField &field,
                                     const std::vector<bool> &valid) {
        auto vtk_array = vtkSmartPointer<ArrayType>::New();
        std::size_t nr_points = cloud->width * cloud->height;
        vtk_array->SetNumberOfComponents(field.count);
        vtk_array->SetName(field.name.c_str());
        vtk_array->SetNumberOfTuples(nr_points);
        size_t j = 0;
        for (size_t point_idx = 0; point_idx < nr_points; point_idx++) {
            if (!valid[point_idx]) {
                continue; // skip invalid
            }
            std::vector<BaseType<ArrayType>> tmp(field.count);

            int point_offset = (int(point_idx) * cloud->point_step);
            int offset = point_offset + field.offset;
            memcpy(tmp.data(), &cloud->data[offset], sizeof(BaseType<ArrayType>) * field.count);
            vtk_array->SetTypedTuple(j++, tmp.data());
        }

        vtk_array->SetNumberOfTuples(j);

        return vtk_array;
    }

    struct PCLChannelFromVtkArrayWorker {
        PCLChannelFromVtkArrayWorker(pcl::PCLPointField &field, unsigned int &point_step, int &index) : field(field),
                                                                                                        point_step(
                                                                                                                point_step),
                                                                                                        index(index) {}

        template<typename ArrayT>
        void operator()(ArrayT *array) {
            vtkDataArrayAccessor<ArrayT> accessor(array);
            field.name = array->GetName();
            field.offset = point_step;
            auto nr_components = array->GetNumberOfComponents();
            field.count = nr_components;
            bool valid = true;
            field.datatype = BaseTypeFromVtkArrayType<ArrayT>::pcl_type;
            assert(array->GetDataTypeSize() == pcl::getFieldSize(field.datatype));
            point_step += pcl::getFieldSize(BaseTypeFromVtkArrayType<ArrayT>::pcl_type) * nr_components;
            index++;
        }

    private:
        pcl::PCLPointField &field;
        unsigned int &point_step;
        int &index;
    };

    struct ConversionWorker {
        ConversionWorker(pcl::PCLPointCloud2Ptr cloud, const pcl::PCLPointField &field) : cloud(cloud), field(field) {}

        template<typename ArrayT>
        void operator()(ArrayT *array) {
            vtkDataArrayAccessor<ArrayT> accessor(array);
            assert(array->GetDataTypeSize() == pcl::getFieldSize(field.datatype));
            auto type_size = array->GetDataTypeSize();
            auto nr_tuples = array->GetNumberOfTuples();
            auto nr_comp = array->GetNumberOfComponents();
            auto ptr = accessor.Array->GetPointer(0);
            for (size_t i = 0; i < nr_tuples; ++i) {
                int point_offset = (i * cloud->point_step);
                int offset = point_offset + field.offset;
                memcpy(&cloud->data[offset], &ptr[i * nr_comp], type_size * nr_comp);
            }
        }

    private:
        pcl::PCLPointCloud2Ptr cloud;
        const pcl::PCLPointField &field;
    };
}

vtkSmartPointer<vtkPolyData>
vtkPCLConversions::PolyDataFromPCDFile(const std::string &filename, bool addCoordsToPointData) {
    pcl::PCLPointCloud2Ptr cloud = boost::make_shared<pcl::PCLPointCloud2>();
    if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
        std::cout << "Error reading pcd file: " << filename;
        return 0;
    }
    return PolyDataFromGenericPointCloud(cloud, true, addCoordsToPointData);

}

void vtkPCLConversions::PointSetToPCDFile(vtkPointSet *polyData, const std::string &filename, bool write_binary) {
    pcl::PCLPointCloud2ConstPtr cloud = PointSetToGenericPointCloud(polyData);
    std::cout << "Saving to " << filename << std::endl;
    pcl::io::savePCDFile(filename, *cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), write_binary);
    std::cout << "done" << std::endl;
}


//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData>
vtkPCLConversions::PolyDataFromGenericPointCloud(pcl::PCLPointCloud2ConstPtr cloud, bool reverse,
                                                 bool addCoordsToPointData) {
    auto poly_data = vtkSmartPointer<vtkPolyData>::New();
    vtkIdType nr_points = cloud->width * cloud->height;

    std::map<std::string, pcl::PCLPointField> rem_fields;
    for (const auto &field: cloud->fields) {
        rem_fields.emplace(field.name, field);
    }

    // Add Points
    size_t x_idx = pcl::getFieldIndex(*cloud, std::string("x"));

    auto coords_data = vtkSmartPointer<vtkFloatArray>::New();
    coords_data->SetNumberOfComponents(3);
    coords_data->SetName("xyz");
    coords_data->SetNumberOfTuples(nr_points);

    vtkSmartPointer<vtkPoints> cloud_points = vtkSmartPointer<vtkPoints>::New();
    cloud_points->Resize(nr_points);
    cloud_points->SetNumberOfPoints(nr_points);
    std::vector<bool> valid(nr_points, true);
    if (cloud->is_dense) {
        for (size_t point_idx = 0; point_idx < nr_points; point_idx++) {
            float point[3];
            int point_offset = (int(point_idx) * cloud->point_step);
            int offset = point_offset + cloud->fields[x_idx].offset;
            memcpy(&point, &cloud->data[offset], sizeof(float) * 3);
            //pid[0] = cloud_points->InsertNextPoint (point);
            cloud_points->SetPoint(point_idx, point);
            coords_data->InsertTypedTuple(point_idx, point);
        }
        //set the points and vertices we created as the geometry and topology of the polydata
        poly_data->SetPoints(cloud_points);
        poly_data->SetVerts(NewVertexCells(nr_points));
    } else {
        size_t j = 0;
        for (size_t point_idx = 0; point_idx < nr_points; point_idx++) {
            float point[3];
            int point_offset = (int(point_idx) * cloud->point_step);
            int offset = point_offset + cloud->fields[x_idx].offset;
            memcpy(&point, &cloud->data[offset], sizeof(float) * 3);
            // Check if the point is invalid
            if (!pcl_isfinite (point[0]) ||
                !pcl_isfinite (point[1]) ||
                !pcl_isfinite (point[2])) {
                valid[point_idx] = false;
                continue;
            }
            cloud_points->SetPoint(j, point);
            coords_data->InsertTypedTuple(j, point);
            j++;
        }
        cloud_points->SetNumberOfPoints(j);
        coords_data->SetNumberOfTuples(j);
        //set the points and vertices we created as the geometry and topology of the polydata
        poly_data->SetPoints(cloud_points);
        poly_data->SetVerts(NewVertexCells(j));
    }

    if (addCoordsToPointData) {
        poly_data->GetPointData()->AddArray(coords_data);
    }

    rem_fields.erase("x");
    rem_fields.erase("y");
    rem_fields.erase("z");

    // Add RGB
    int rgb_idx = pcl::getFieldIndex(*cloud, "rgb");
    if (rgb_idx == -1) {
        rgb_idx = pcl::getFieldIndex(*cloud, "rgba");
    }
    if (rgb_idx != -1) {
        auto field = cloud->fields[rgb_idx];
        if (field.name == "rgba") {
            // do not parse "a" field
            field.count = 3;
        }
        auto new_array = vtkSmartPointer<vtkUnsignedCharArray>::New();
        std::size_t nr_points = cloud->width * cloud->height;
        new_array->SetNumberOfComponents(field.count);
        new_array->SetName(field.name.c_str());
        new_array->SetNumberOfTuples(cloud_points->GetNumberOfPoints());

        size_t j = 0;
        for (size_t point_idx = 0; point_idx < nr_points; point_idx++) {
            if (!valid[point_idx]) {
                continue; // skip if invalid
            }
            std::vector<BaseType<vtkUnsignedCharArray>> tmp(field.count);
            int point_offset = (int(point_idx) * cloud->point_step);
            int offset = point_offset + field.offset;
            memcpy(tmp.data(), &cloud->data[offset], sizeof(BaseType<vtkUnsignedCharArray>) * field.count);

            if (reverse) {
                // swap color values
                std::reverse(tmp.begin(), tmp.end());
            }

            new_array->SetTypedTuple(j++, tmp.data());
        }
        //auto new_array = addPolyDataChannelFromPointCloud<vtkUnsignedCharArray>(cloud, field);
        poly_data->GetPointData()->AddArray(new_array.GetPointer());
        //poly_data->GetCellData()->SetScalars(colors);
        rem_fields.erase("rgb");
        rem_fields.erase("rgba");
    }

    // Add Intensity
    int intensity_idx = pcl::getFieldIndex(*cloud, "intensity");
    if (intensity_idx != -1) {
        auto field = cloud->fields[intensity_idx];
        auto new_array = addPolyDataChannelFromPointCloud<vtkFloatArray>(cloud, field, valid);
        poly_data->GetPointData()->AddArray(new_array.GetPointer());
        if (rgb_idx == -1)
            poly_data->GetPointData()->SetActiveAttribute("intensity", vtkDataSetAttributes::SCALARS);
        rem_fields.erase("intensity");
    }

    // Add Normals
    int normal_x_idx = pcl::getFieldIndex(*cloud, std::string("normal_x"));
    if (normal_x_idx != -1) {
        auto field = cloud->fields[normal_x_idx];
        auto new_array = addPolyDataChannelFromPointCloud<vtkFloatArray>(cloud, field, valid);
        poly_data->GetPointData()->SetNormals(new_array);
        rem_fields.erase("normal_x");
    }

    // Parse any other fields
    for (const auto &field : rem_fields) {
        switch (field.second.datatype) {
            case pcl::PCLPointField::INT8: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkCharArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::INT16: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkShortArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::INT32: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkIntArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::UINT8: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkUnsignedCharArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::UINT16: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkUnsignedShortArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::UINT32: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkUnsignedIntArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::FLOAT32: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkFloatArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            case pcl::PCLPointField::FLOAT64: {
                auto new_array = addPolyDataChannelFromPointCloud<vtkDoubleArray>(cloud, field.second, valid);
                poly_data->GetPointData()->AddArray(new_array);
                break;
            }
            default:
                break;
        }
    }
    return poly_data;
}

//----------------------------------------------------------------------------
pcl::PCLPointCloud2Ptr vtkPCLConversions::PointSetToGenericPointCloud(vtkPointSet *polyData) {

    auto nr_points = polyData->GetNumberOfPoints();
    int nr_arrays = polyData->GetPointData()->GetNumberOfArrays();
    unsigned int point_step = 0;
    auto cloud = boost::make_shared<pcl::PCLPointCloud2>();
    cloud->is_bigendian = 0;
    cloud->is_dense = true;
    cloud->height = 1;
    cloud->width = nr_points;
    cloud->fields.resize(3 + nr_arrays);

    // x
    cloud->fields[0].name = "x";
    cloud->fields[0].datatype = pcl::PCLPointField::FLOAT32;
    cloud->fields[0].count = 1;
    cloud->fields[0].offset = point_step;
    point_step += sizeof(float);

    // y
    cloud->fields[1].name = "y";
    cloud->fields[1].datatype = pcl::PCLPointField::FLOAT32;
    cloud->fields[1].count = 1;
    cloud->fields[1].offset = point_step;
    point_step += sizeof(float);

    // z
    cloud->fields[2].name = "z";
    cloud->fields[2].datatype = pcl::PCLPointField::FLOAT32;
    cloud->fields[2].count = 1;
    cloud->fields[2].offset = point_step;
    point_step += sizeof(float);

    int actual_index = 0;

    std::unordered_map<int, pcl::PCLPointField> relevant_fields;
    // get pointdata arrays from the polydata object
    for (int i = 0; i < nr_arrays; ++i) {
        auto array = polyData->GetPointData()->GetArray(i);
        PCLChannelFromVtkArrayWorker worker(cloud->fields[3 + actual_index], point_step, actual_index);
        assert(vtkArrayDispatch::Dispatch::Execute(array, worker));
        relevant_fields.emplace(i, cloud->fields[3 + actual_index - 1]);
    }


    cloud->point_step = point_step;
    cloud->data.resize(point_step * nr_points);
    vtkFloatArray *floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
    vtkDoubleArray *doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());

    assert(floatPoints || doublePoints);
    int x_idx = pcl::getFieldIndex(*cloud, std::string("x"));
    if (floatPoints) {
        float *data = floatPoints->GetPointer(0);
        for (std::size_t i = 0; i < nr_points; ++i) {
            int point_offset = (i * cloud->point_step);
            int offset = point_offset + cloud->fields[x_idx].offset;
            memcpy(&cloud->data[offset], &data[i * 3], sizeof(float) * 3);

        }
    } else if (doublePoints) {
        double *tmp_data = doublePoints->GetPointer(0);
        for (std::size_t i = 0; i < nr_points; ++i) {
            // cast down to float
            float data[3];
            for (int j = 0; j < 3; ++j) {
                data[j] = static_cast<float>(tmp_data[i * 3 + j]);
            }
            int point_offset = (i * cloud->point_step);
            int offset = point_offset + cloud->fields[x_idx].offset;
            memcpy(&cloud->data[offset], &data[0 * 3], sizeof(float) * 3);
        }
    }
    // convert all other fields
    for (const auto &field : relevant_fields) {
        ConversionWorker worker(cloud, field.second);
        auto array = polyData->GetPointData()->GetArray(field.second.name.c_str());
        assert(vtkArrayDispatch::Dispatch::Execute(array, worker)); // dispatch
    }

    return cloud;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloudXYZ(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    vtkIdType nr_points = cloud->points.size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(nr_points);

    if (cloud->is_dense) {
        for (vtkIdType i = 0; i < nr_points; ++i) {
            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            points->SetPoint(i, point);
        }
    } else {
        vtkIdType j = 0;    // true point index
        for (vtkIdType i = 0; i < nr_points; ++i) {
            // Check if the point is invalid
            if (!pcl_isfinite (cloud->points[i].x) ||
                !pcl_isfinite (cloud->points[i].y) ||
                !pcl_isfinite (cloud->points[i].z))
                continue;

            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            points->SetPoint(j, point);
            j++;
        }
        nr_points = j;
        points->SetNumberOfPoints(nr_points);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points.GetPointer());
    polyData->SetVerts(NewVertexCells(nr_points));
    return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloudXYZRGB(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    vtkIdType nr_points = cloud->points.size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(nr_points);

    vtkNew<vtkUnsignedCharArray> rgbArray;
    rgbArray->SetName("rgb");
    rgbArray->SetNumberOfComponents(3);
    rgbArray->SetNumberOfTuples(nr_points);


    if (cloud->is_dense) {
        for (vtkIdType i = 0; i < nr_points; ++i) {
            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
            points->SetPoint(i, point);
            rgbArray->SetTypedTuple(i, color);
        }
    } else {
        vtkIdType j = 0;    // true point index
        for (vtkIdType i = 0; i < nr_points; ++i) {
            // Check if the point is invalid
            if (!pcl_isfinite (cloud->points[i].x) ||
                !pcl_isfinite (cloud->points[i].y) ||
                !pcl_isfinite (cloud->points[i].z))
                continue;

            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
            points->SetPoint(j, point);
            rgbArray->SetTypedTuple(j, color);
            j++;
        }
        nr_points = j;
        points->SetNumberOfPoints(nr_points);
        rgbArray->SetNumberOfTuples(nr_points);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points.GetPointer());
    polyData->GetPointData()->AddArray(rgbArray.GetPointer());
    polyData->SetVerts(NewVertexCells(nr_points));
    return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloudXYZRGBA(
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {
    vtkIdType nr_points = cloud->points.size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(nr_points);

    vtkNew<vtkUnsignedCharArray> rgbArray;
    rgbArray->SetName("rgba");
    rgbArray->SetNumberOfComponents(3);
    rgbArray->SetNumberOfTuples(nr_points);


    if (cloud->is_dense) {
        for (vtkIdType i = 0; i < nr_points; ++i) {
            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
            points->SetPoint(i, point);
            rgbArray->SetTypedTuple(i, color);
        }
    } else {
        vtkIdType j = 0;    // true point index
        for (vtkIdType i = 0; i < nr_points; ++i) {
            // Check if the point is invalid
            if (!pcl_isfinite (cloud->points[i].x) ||
                !pcl_isfinite (cloud->points[i].y) ||
                !pcl_isfinite (cloud->points[i].z))
                continue;

            float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
            points->SetPoint(j, point);
            rgbArray->SetTypedTuple(j, color);
            j++;
        }
        nr_points = j;
        points->SetNumberOfPoints(nr_points);
        rgbArray->SetNumberOfTuples(nr_points);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points.GetPointer());
    polyData->GetPointData()->AddArray(rgbArray.GetPointer());
    polyData->SetVerts(NewVertexCells(nr_points));
    return polyData;

}

//----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr vtkPCLConversions::PointSetToPointCloudXYZ(vtkPointSet *polyData) {
    const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = numberOfPoints;
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(numberOfPoints);
    if (!numberOfPoints) {
        return cloud;
    }

    vtkFloatArray *floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
    vtkDoubleArray *doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
    assert(floatPoints || doublePoints);

    if (floatPoints) {
        float *data = floatPoints->GetPointer(0);
        for (vtkIdType i = 0; i < numberOfPoints; ++i) {
            cloud->points[i].x = data[i * 3];
            cloud->points[i].y = data[i * 3 + 1];
            cloud->points[i].z = data[i * 3 + 2];
        }
    } else if (doublePoints) {
        double *data = doublePoints->GetPointer(0);
        for (vtkIdType i = 0; i < numberOfPoints; ++i) {
            cloud->points[i].x = data[i * 3];
            cloud->points[i].y = data[i * 3 + 1];
            cloud->points[i].z = data[i * 3 + 2];
        }
    }

    return cloud;
}

//----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZRGB>::Ptr vtkPCLConversions::PointSetToPointCloudXYZRGB(vtkPointSet *polyData) {
    if (polyData->GetPointData()->HasArray("rgb_colors") != 1) {
        vtkGenericWarningMacro("No color data found. color data will not be set");
        // if there is no color data use regular function instead and convert
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointSetToPointCloudXYZ(polyData);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*cloud, *cloud_out);
        return cloud_out;
    }

    const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud->width = static_cast<std::uint32_t>(numberOfPoints);
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(static_cast<unsigned long>(numberOfPoints));

    if (!numberOfPoints) {
        return cloud;
    }

    vtkFloatArray *floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
    vtkDoubleArray *doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());

    auto *colors_abstract = polyData->GetPointData()->GetAbstractArray("rgb_colors");
    assert((floatPoints || doublePoints) && colors_abstract);
    assert(colors_abstract->GetNumberOfComponents() == 3 &&
           colors_abstract->GetNumberOfTuples() == numberOfPoints); // check if malformed

    vtkUnsignedCharArray *colors = vtkUnsignedCharArray::SafeDownCast(colors_abstract);
    assert(colors);
    if (floatPoints) {
        float *data = floatPoints->GetPointer(0);
        for (vtkIdType i = 0; i < numberOfPoints; ++i) {
            cloud->points[i].x = data[i * 3];
            cloud->points[i].y = data[i * 3 + 1];
            cloud->points[i].z = data[i * 3 + 2];

            cloud->points[i].r = colors->GetTypedComponent(i, 0);
            cloud->points[i].g = colors->GetTypedComponent(i, 1);
            cloud->points[i].b = colors->GetTypedComponent(i, 2);
        }
    } else if (doublePoints) {
        double *data = doublePoints->GetPointer(0);
        for (vtkIdType i = 0; i < numberOfPoints; ++i) {
            cloud->points[i].x = static_cast<float>(data[i * 3]);
            cloud->points[i].y = static_cast<float>(data[i * 3 + 1]);
            cloud->points[i].z = static_cast<float>(data[i * 3 + 2]);
            cloud->points[i].r = colors->GetTypedComponent(i, 0);
            cloud->points[i].g = colors->GetTypedComponent(i, 1);
            cloud->points[i].b = colors->GetTypedComponent(i, 2);
        }
    }

    return cloud;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkPCLConversions::NewVertexCells(vtkIdType numberOfVerts) {
    vtkNew<vtkIdTypeArray> cells;
    cells->SetNumberOfValues(numberOfVerts * 2);
    vtkIdType *ids = cells->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfVerts; ++i) {
        ids[i * 2] = 1;
        ids[i * 2 + 1] = i;
    }

    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
    cellArray->SetCells(numberOfVerts, cells.GetPointer());
    return cellArray;
}

//----------------------------------------------------------------------------
void vtkPCLConversions::PerformPointCloudConversionBenchmark(vtkPolyData *polyData) {
    if (!polyData) {
        return;
    }

    double start;
    double elapsed;
    unsigned long kilobytes;

    const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();
    std::cout << "Number of input points: " << numberOfPoints << std::endl;

    start = vtkTimerLog::GetUniversalTime();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = PointSetToPointCloudXYZ(polyData);
    elapsed = vtkTimerLog::GetUniversalTime() - start;

    std::cout << "Conversion to pcl::PointCloud took " << elapsed << " seconds. "
              << numberOfPoints / elapsed << " points per second." << std::endl;


    start = vtkTimerLog::GetUniversalTime();
    vtkSmartPointer<vtkPolyData> tempPolyData = PolyDataFromPointCloudXYZ(tempCloud);
    elapsed = vtkTimerLog::GetUniversalTime() - start;

    std::cout << "Conversion to vtkPolyData took " << elapsed << " seconds. "
              << numberOfPoints / elapsed << " points per second." << std::endl;


    start = vtkTimerLog::GetUniversalTime();
    vtkSmartPointer<vtkCellArray> tempCells = NewVertexCells(numberOfPoints);
    elapsed = vtkTimerLog::GetUniversalTime() - start;

    std::cout << "Constructing vertex cells took " << elapsed << " seconds. "
              << numberOfPoints / elapsed << " points per second." << std::endl;


    kilobytes = tempPolyData->GetActualMemorySize();
    std::cout << "vtkPolyData uses " << kilobytes / 1024.0 << " MB. "
              << kilobytes * 1024 / numberOfPoints << " bytes per point." << std::endl;

    kilobytes = tempPolyData->GetPoints()->GetActualMemorySize();
    std::cout << "vtkPolyData's points use " << kilobytes / 1024.0 << " MB. "
              << kilobytes * 1024 / numberOfPoints << " bytes per point." << std::endl;

    kilobytes = tempPolyData->GetVerts()->GetActualMemorySize();
    std::cout << "vtkPolyData's cells use " << kilobytes / 1024.0 << " MB. "
              << kilobytes * 1024 / numberOfPoints << " bytes per point." << std::endl;
}

//----------------------------------------------------------------------------
namespace {

    vtkSmartPointer<vtkIntArray> NewLabelsArray(vtkIdType length) {
        vtkSmartPointer<vtkIntArray> labels = vtkSmartPointer<vtkIntArray>::New();
        labels->SetNumberOfComponents(1);
        labels->SetNumberOfTuples(length);
        labels->FillComponent(0, 0);
        return labels;
    }

    void LabelIndices(const std::vector<int> &indices, vtkIntArray *labels, const int labelValue) {
        const size_t numberOfIndices = indices.size();
        for (size_t k = 0; k < numberOfIndices; ++k) {
            labels->SetValue(indices[k], labelValue);
        }
    }

}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length) {
    vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
    if (indices) {
        LabelIndices(*indices, labels, 1);
    }
    return labels;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length) {
    vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
    if (indices) {
        LabelIndices(indices->indices, labels, 1);
    }
    return labels;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray>
vtkPCLConversions::NewLabelsArray(const std::vector<pcl::PointIndices> &indices, vtkIdType length) {
    vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);

    for (size_t i = 0; i < indices.size(); ++i) {
        const int labelValue = i + 1;
        LabelIndices(indices[i].indices, labels, labelValue);
    }

    return labels;
}

//----------------------------------------------------------------------------
void vtkPCLConversions::PrintSelf(ostream &os, vtkIndent indent) {
    this->Superclass::PrintSelf(os, indent);
    //os << indent << "Property: " << endl;
}
