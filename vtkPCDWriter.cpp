//
// Created by figelbrink on 12/11/17.
//

#include "vtkPCDWriter.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"

#include "vtkPCLConversions.h"

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCDWriter);

//----------------------------------------------------------------------------
vtkPCDWriter::vtkPCDWriter() : FileName(), WriteBinary(true) {
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(0);
    std::cout << "Writer created" << std::endl;
}

//----------------------------------------------------------------------------
vtkPCDWriter::~vtkPCDWriter() {
}

//----------------------------------------------------------------------------
void vtkPCDWriter::PrintSelf(ostream &os, vtkIndent indent) {
    this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------

void vtkPCDWriter::WriteData() {
    this->
    std::cout << "writing" << std::endl;
    vtkPointSet* input = vtkPointSet::SafeDownCast(this->GetInputDataObject(0,0));

    if (GetFileName().size() == 0) {
        vtkErrorMacro("Filename is not set");
        FileName = "output.ply";
    }

    vtkPCLConversions::PointSetToPCDFile(input, FileName, WriteBinary);
}

int vtkPCDWriter::FillInputPortInformation(int, vtkInformation* info)
{
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPointSet");
    return 1;
}

