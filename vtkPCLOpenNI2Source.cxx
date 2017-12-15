/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLOpenNISource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLOpenNI2Source.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <thread>
#include <mutex>

using Cloud=pcl::PointCloud<pcl::PointXYZRGBA>;
using CloudPtr=Cloud::Ptr;
using CloudConstPtr=Cloud::ConstPtr;

//----------------------------------------------------------------------------
class vtkPCLOpenNI2Source::vtkInternal {
public:

    vtkInternal(): NewData(false) {}

    ~vtkInternal() {
    }

    void HandleIncomingCloud(const CloudConstPtr &newCloud) {
        vtkSmartPointer<vtkPolyData> newPolyData = vtkPCLConversions::PolyDataFromPointCloudXYZRGBA(newCloud);
        std::lock_guard<std::mutex> lock(this->mutex);
        this->PolyData = newPolyData;
        this->NewData = true;
    }

    vtkSmartPointer<vtkPolyData> GetLatestPolyData() {
        std::lock_guard<std::mutex> lock(this->mutex);
        vtkSmartPointer<vtkPolyData> polyData = this->PolyData;

        this->PolyData = nullptr;
        this->NewData = false;
        return polyData;
    }

    bool HasNewData() {
        std::lock_guard<std::mutex> lock(this->mutex);
        return this->NewData;
    }

    bool NewData;
    std::shared_ptr<pcl::io::OpenNI2Grabber> Grabber;
    std::mutex mutex;
    vtkSmartPointer<vtkPolyData> PolyData;

    boost::function<void(const CloudConstPtr &)> Callback;

};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLOpenNI2Source);

//----------------------------------------------------------------------------
vtkPCLOpenNI2Source::vtkPCLOpenNI2Source() {
    this->Internal = std::make_unique<vtkInternal>();
    this->SetNumberOfInputPorts(0);
    this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLOpenNI2Source::~vtkPCLOpenNI2Source() {}

//----------------------------------------------------------------------------
void vtkPCLOpenNI2Source::StartGrabber() {
    if (!this->Internal->Grabber) {
        this->Internal->Grabber = std::make_shared<pcl::io::OpenNI2Grabber>("");
        this->Internal->Callback = boost::bind(&vtkPCLOpenNI2Source::vtkInternal::HandleIncomingCloud, this->Internal.get(),
                                               _1);
        this->Internal->Grabber->registerCallback(this->Internal->Callback);
    }
    this->Internal->Grabber->start();
}

//----------------------------------------------------------------------------
void vtkPCLOpenNI2Source::StopGrabber() {
    if (this->Internal->Grabber) {
        this->Internal->Grabber->stop();
    }
}

//----------------------------------------------------------------------------
bool vtkPCLOpenNI2Source::HasNewData() {
    return this->Internal->HasNewData();
}

//----------------------------------------------------------------------------
void vtkPCLOpenNI2Source::Poll() {
    if (this->HasNewData()) {
        this->Modified();
    }
}

//----------------------------------------------------------------------------
int vtkPCLOpenNI2Source::RequestData(
        vtkInformation *vtkNotUsed(request),
        vtkInformationVector **inputVector,
        vtkInformationVector *outputVector) {
    vtkInformation *outInfo = outputVector->GetInformationObject(0);
    vtkDataSet *output = vtkDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

    if (!this->HasNewData()) {
        return 1;
    }

    output->ShallowCopy(this->Internal->GetLatestPolyData());
    return 1;
}

//----------------------------------------------------------------------------
void vtkPCLOpenNI2Source::PrintSelf(ostream &os, vtkIndent indent) {
    this->Superclass::PrintSelf(os, indent);
}
