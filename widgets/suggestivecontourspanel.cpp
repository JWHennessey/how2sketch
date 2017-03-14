#include "suggestivecontourspanel.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDebug>
#include "utils/contourparams.h"

SuggestiveContoursPanel::SuggestiveContoursPanel()
{
    setFocusPolicy(Qt::ClickFocus);
    init();
}

SuggestiveContoursPanel::~SuggestiveContoursPanel()
{

}

auto SuggestiveContoursPanel::init() -> void
{
    auto layout = new QVBoxLayout(this);
    auto groupBox = new QGroupBox(tr("Contour Params"));
    items = new QVBoxLayout;
    items->setAlignment(Qt::AlignTop);
    groupBox->setLayout(items);
    layout->addWidget(groupBox);

    auto jeroenCheckbox = new QCheckBox();
    jeroenCheckbox->setChecked(ContourParams::getInstance()->getJeroenMethod());
    connect(jeroenCheckbox, SIGNAL(toggled(bool)), this, SLOT(jeroenToggled(bool)));

    auto cLimitSpinBox = new QDoubleSpinBox();
    cLimitSpinBox->setValue(ContourParams::getInstance()->getCLimit());
    cLimitSpinBox->setRange(0.0, 10.0);
    cLimitSpinBox->setSingleStep(0.1);
    connect(cLimitSpinBox, SIGNAL(valueChanged(double)), this, SLOT(cLimitValueChanged(double)));

    auto scLimitSpinBox = new QDoubleSpinBox();
    scLimitSpinBox->setValue(ContourParams::getInstance()->getScLimit());
    scLimitSpinBox->setRange(0.0, 10.0);
    scLimitSpinBox->setSingleStep(0.1);
    connect(cLimitSpinBox, SIGNAL(valueChanged(double)), this, SLOT(scLimitValueChanged(double)));

    auto dwkrLimitSpinBox = new QDoubleSpinBox();
    dwkrLimitSpinBox->setValue(ContourParams::getInstance()->getDwkrLimit());
    dwkrLimitSpinBox->setRange(0.0, 10.0);
    dwkrLimitSpinBox->setSingleStep(0.1);
    connect(cLimitSpinBox, SIGNAL(valueChanged(double)), this, SLOT(dwkrLimitValueChanged(double)));

    auto formLayout = new QFormLayout;
    formLayout->addRow(tr("Jeroen Method"), jeroenCheckbox);
    formLayout->addRow(tr("C Limit:"), cLimitSpinBox);
    formLayout->addRow(tr("SC Limit:"), scLimitSpinBox);
    formLayout->addRow(tr("DWKR Limit:"), dwkrLimitSpinBox);
    items->addLayout(formLayout);

}

void SuggestiveContoursPanel::cLimitValueChanged(double val)
{
    ContourParams::getInstance()->setCLimit(val);
    emit updateViewer();
}

void SuggestiveContoursPanel::scLimitValueChanged(double val)
{
    ContourParams::getInstance()->setScLimit(val);
    emit updateViewer();
}

void SuggestiveContoursPanel::dwkrLimitValueChanged(double val)
{
    ContourParams::getInstance()->setDwkrLimit(val);
    emit updateViewer();
}

void SuggestiveContoursPanel::jeroenToggled(bool val)
{
    ContourParams::getInstance()->setJeroenMethod(val);
    emit updateViewer();
}
