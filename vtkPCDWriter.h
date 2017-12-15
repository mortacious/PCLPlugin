//
// Created by figelbrink on 12/11/17.
//
#pragma once
#ifndef PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H
#define PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H

#include <vtkWriter.h>
#include <vtkPCLFiltersModule.h>
#include <string>

class VTKPCLFILTERS_EXPORT vtkPCDWriter : public vtkWriter
{
    public:
    static vtkPCDWriter* New();
    vtkTypeMacro(vtkPCDWriter, vtkWriter);
    void PrintSelf(ostream& os, vtkIndent indent);

    vtkSetMacro(FileName, std::string);
    vtkGetMacro(FileName, std::string);
    vtkSetMacro(WriteBinary, int);
    vtkGetMacro(WriteBinary, int);

    protected:
    vtkPCDWriter();
    ~vtkPCDWriter();

    virtual int FillInputPortInformation(int port, vtkInformation* info) VTK_OVERRIDE;
    virtual void WriteData() VTK_OVERRIDE;


    std::string FileName;
    int WriteBinary;

    private:
    vtkPCDWriter(const vtkPCDWriter&); // Not implemented.
    void operator=(const vtkPCDWriter&);   // Not implemented.
};


#endif //PARAVIEW_PCLPLUGINS_VTKPCDWRITER_H
