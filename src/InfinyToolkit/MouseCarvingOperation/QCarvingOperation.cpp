/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, June 2021          *
 ****************************************************************************/
#include <MeshRefinement/MouseCuttingOperation/QCuttingOperation.h>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>

namespace sofa::gui::qt
{

QCuttingOperation::QCuttingOperation()
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* HLayout1 = new QHBoxLayout();
    QLabel* label = new QLabel(QString("Depth (mm)"), this);
    depthSlider = new QSlider(Qt::Horizontal, this);
    depthValue = new QSpinBox(this);
    depthValue->setMinimum(0);
    depthValue->setMaximum(100);
    depthValue->setSingleStep(1);
    depthValue->setEnabled(true);
    depthValue->setValue(0);

    HLayout1->addWidget(label);
    HLayout1->addWidget(depthSlider);
    HLayout1->addWidget(depthValue);
    layout->addLayout(HLayout1);

    connect(depthSlider, SIGNAL(valueChanged(int)), depthValue, SLOT(setValue(int)));
    connect(depthValue, SIGNAL(valueChanged(int)), depthSlider, SLOT(setValue(int)));    
}


double QCuttingOperation::getDepth() const
{
    return depthValue->value();
}


} // namespace sofa::gui::qt