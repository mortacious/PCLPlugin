//
// Created by figelbrink on 12/11/17.
//
#pragma once
#ifndef PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H
#define PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>
#include <string>

class VTKPCLFILTERS_EXPORT vtkPCDWriter : public vtkPolyDataAlgorithm
{
    public:
    static vtkPCDWriter* New();
    vtkTypeMacro(vtkPCDWriter, vtkPolyDataAlgorithm);
    void PrintSelf(ostream& os, vtkIndent indent);

    vtkSetMacro(FileName, std::string);
    vtkGetMacro(FileName, std::string);
    vtkSetMacro(WriteBinary, int);
    vtkGetMacro(WriteBinary, int);

    protected:
    vtkPCDWriter();
    ~vtkPCDWriter();


    //virtual int FillOutputPortInformation(int port, vtkInformation* info);

    int RequestData(vtkInformation*        request,
                    vtkInformationVector** inputVector,
                    vtkInformationVector*  outputVector);


    std::string FileName;
    int WriteBinary;

    private:
    vtkPCDWriter(const vtkPCDWriter&); // Not implemented.
    void operator=(const vtkPCDWriter&);   // Not implemented.
};


#endif //PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H
