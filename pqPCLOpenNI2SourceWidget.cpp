//
// Created by figelbrink on 12/13/17.
//
#include "pqView.h"

#include <vtkSMSourceProxy.h>
#include <vtkSMIntVectorProperty.h>
//#include <vtkPCLFiltersModule.h>
#include "vtkSMBooleanDomain.h"
#include "vtkSMCompositeTreeDomain.h"
#include "vtkSMDomain.h"
#include "vtkSMDomainIterator.h"
#include "vtkSMEnumerationDomain.h"
#include "vtkSMIntRangeDomain.h"
#include "vtkSMNumberOfComponentsDomain.h"
#include "vtkSMProperty.h"
#include "vtkSMPropertyHelper.h"
#include "vtkSmartPointer.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include "pqPCLOpenNI2SourceWidget.h"

pqPCLOpenNI2SourceWidget::pqPCLOpenNI2SourceWidget(vtkSMProxy *smproxy, vtkSMPropertyGroup* smgroup, QWidget *parentObject)
: Superclass(smproxy, parentObject) {
auto *layout = new QVBoxLayout(this);

this->Timer = new QTimer(this);
this->connect(this->Timer, SIGNAL(timeout()), SLOT(onPollSource()));

QCheckBox *check = new QCheckBox("Enable OpenNI2 grabber thread");
this->connect(check, SIGNAL(toggled(bool)), SLOT(onEnableOpenNI2GrabberChecked(bool)));
layout->addWidget(check);

layout->addWidget(new QLabel);

QPushButton *refreshButton = new QPushButton("Refresh");
this->connect(refreshButton, SIGNAL(clicked()), SLOT(onRefreshClicked()));
layout->addWidget(refreshButton);

QCheckBox *autoRefreshCheck = new QCheckBox("Auto refresh");
this->connect(autoRefreshCheck, SIGNAL(toggled(bool)), SLOT(onAutoRefreshChecked(bool)));
layout->addWidget(autoRefreshCheck);

QSlider *slider = new QSlider(Qt::Horizontal);
this->connect(slider, SIGNAL(valueChanged(int)), SLOT(onSliderValueChanged(int)));
layout->addWidget(slider);
this->AutoRefreshLabel = new QLabel();
layout->addWidget(this->AutoRefreshLabel);

layout->addStretch();

slider->setMinimum(1);
slider->setMaximum(200);
slider->setValue(100);

}

pqPCLOpenNI2SourceWidget::~pqPCLOpenNI2SourceWidget() {}


void pqPCLOpenNI2SourceWidget::onEnableOpenNI2GrabberChecked(bool checked) {
    if (checked) {
        proxy()->InvokeCommand("StartGrabber");
    } else {
        proxy()->InvokeCommand("StopGrabber");

    }
}

void pqPCLOpenNI2SourceWidget::onAutoRefreshChecked(bool checked) {
    if (checked) {
        this->Timer->start();
    } else {
        this->Timer->stop();
    }
}

void pqPCLOpenNI2SourceWidget::onSliderValueChanged(int sliderValue) {
    int timeoutMax = 5000;
    int timeout = timeoutMax * sliderValue / 200.0;
    this->Timer->setInterval(timeout);
    this->AutoRefreshLabel->setText(QString("Auto refresh timeout: %0 s").arg(timeout / 1000.0, 0, 'f', 2));
}

void pqPCLOpenNI2SourceWidget::onRefreshClicked() {
    this->onPollSource();
}

void pqPCLOpenNI2SourceWidget::onPollSource() {

    vtkSMIntVectorProperty *prop = vtkSMIntVectorProperty::SafeDownCast(proxy()->GetProperty("HasNewData"));
    if (!prop) {
        return;
    }

    proxy()->InvokeCommand("Poll");
    proxy()->UpdatePropertyInformation(prop);
    int hasNewData = prop->GetElement(0);

    if (hasNewData && this->view()) {
        this->view()->render();
    }
}