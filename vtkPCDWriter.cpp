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
vtkPCDWriter::vtkPCDWriter(): FileName(), WriteBinary(true)
{
    this->SetNumberOfInputPorts(1);
    this->SetNumberOfOutputPorts(0);
}

//----------------------------------------------------------------------------
vtkPCDWriter::~vtkPCDWriter()
{
}

//----------------------------------------------------------------------------
void vtkPCDWriter::PrintSelf(ostream& os, vtkIndent indent)
{
    this->Superclass::PrintSelf(os,indent);
}

//----------------------------------------------------------------------------
int vtkPCDWriter::RequestData(
        vtkInformation *vtkNotUsed(request),
        vtkInformationVector **inputVector,
        vtkInformationVector *outputVector)
{

    if (GetFileName().size() == 0)
    {
        vtkErrorMacro("Filename is not set");
        return 0;
    }
    vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
    vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

    vtkPCLConversions::PolyDataToPCDFile(input, FileName, WriteBinary);

    return 1;
}