/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLVoxelGrid.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLVoxelGrid -
// .SECTION Description
//

#ifndef __vtkPCLVoxelGrid_h
#define __vtkPCLVoxelGrid_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLVoxelGrid : public vtkPolyDataAlgorithm
{
public:
    vtkTypeMacro(vtkPCLVoxelGrid, vtkPolyDataAlgorithm);
    void PrintSelf(ostream& os, vtkIndent indent);

    static vtkPCLVoxelGrid *New();

    vtkSetVector3Macro(LeafSize, double);
    vtkGetVector3Macro(LeafSize, double);
    vtkSetMacro(MinimumPointsNumberPerVoxel, unsigned int);
    vtkGetMacro(MinimumPointsNumberPerVoxel, unsigned int);
protected:

    double LeafSize[3];
    unsigned int MinimumPointsNumberPerVoxel;

    virtual int RequestData(vtkInformation *request,
                            vtkInformationVector **inputVector,
                            vtkInformationVector *outputVector);

    virtual int FillInputPortInformation(int port,
                                         vtkInformation* info);

    vtkPCLVoxelGrid();
    virtual ~vtkPCLVoxelGrid();

private:
    vtkPCLVoxelGrid(const vtkPCLVoxelGrid&);  // Not implemented.
    void operator=(const vtkPCLVoxelGrid&);  // Not implemented.
};

#endif


