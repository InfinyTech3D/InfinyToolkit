/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, June 2021          *
 ****************************************************************************/
#pragma once
#include <MeshRefinement/config.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/qt/QMouseOperations.h>
#include <MeshRefinement/MouseCuttingOperation/CuttingOperation.h>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>

namespace sofa::gui::qt
{

class QCuttingOperation : public QWidget, public sofa::gui::common::CuttingOperation
{
    Q_OBJECT
public:
    QCuttingOperation();

    double getDepth() const override;

protected:
    QSlider* depthSlider;
    QSpinBox* depthValue;
};

} // namespace sofa::gui::qt