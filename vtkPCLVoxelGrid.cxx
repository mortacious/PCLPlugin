/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLVoxelGrid.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLVoxelGrid.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkAlgorithmOutput.h"
#include <vtkPointData.h>

#include <pcl/filters/voxel_grid.h>
#include <iostream>

//----------------------------------------------------------------------------
namespace {

pcl::PCLPointCloud2Ptr ApplyVoxelGrid(
                      typename pcl::PCLPointCloud2ConstPtr cloud,
                      const double* leafSize,
                      unsigned int minimumPointsPerVoxel)
{

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
    pcl::PCLPointCloud2Ptr cloudFiltered = boost::make_shared<pcl::PCLPointCloud2>();
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
    voxelGrid.setDownsampleAllData(true);
    voxelGrid.setMinimumPointsNumberPerVoxel(minimumPointsPerVoxel);
    voxelGrid.filter(*cloudFiltered);
    return cloudFiltered;
}

}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLVoxelGrid);

//----------------------------------------------------------------------------
vtkPCLVoxelGrid::vtkPCLVoxelGrid()
{
    this->LeafSize[0] = 0.01;
    this->LeafSize[1] = 0.01;
    this->LeafSize[2] = 0.01;
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLVoxelGrid::~vtkPCLVoxelGrid()
{
}

//----------------------------------------------------------------------------
int vtkPCLVoxelGrid::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
    vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
    vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
    vtkInformation *outInfo = outputVector->GetInformationObject(0);
    vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));


    pcl::PCLPointCloud2Ptr cloud = vtkPCLConversions::PolyDataToGenericPointCloud(input);
    pcl::PCLPointCloud2Ptr cloudFiltered = ApplyVoxelGrid(cloud, this->LeafSize, MinimumPointsNumberPerVoxel);
    output->ShallowCopy(vtkPCLConversions::PolyDataFromGenericPointCloud(cloudFiltered));

    return 1;
}

//----------------------------------------------------------------------------
void vtkPCLVoxelGrid::PrintSelf(ostream& os, vtkIndent indent)
{
    this->Superclass::PrintSelf(os,indent);
}
