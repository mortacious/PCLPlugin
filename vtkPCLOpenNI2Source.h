/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLOpenNISource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLOpenNISource -
// .SECTION Description
//

#ifndef __vtkPCLOpenNISource_h
#define __vtkPCLOpenNISource_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>
#include <memory>

class VTKPCLFILTERS_EXPORT vtkPCLOpenNI2Source : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLOpenNI2Source, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkPCLOpenNI2Source *New();

  bool HasNewData();

  void Poll();

  void StartGrabber();
  void StopGrabber();

protected:

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkPCLOpenNI2Source();
  virtual ~vtkPCLOpenNI2Source();

private:
    vtkPCLOpenNI2Source(const vtkPCLOpenNI2Source&);  // Not implemented.
  void operator=(const vtkPCLOpenNI2Source&);  // Not implemented.

  class vtkInternal;
  std::unique_ptr<vtkInternal> Internal;
};

#endif
