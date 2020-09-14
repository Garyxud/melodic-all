#include "axis_limits_dialog.h"
#include "ui_axis_limits_dialog.h"

#include <QLineEdit>
#include <QDoubleValidator>

using std::numeric_limits;

AxisLimitsDialog::AxisLimitsDialog(QWidget* parent) : QDialog(parent), ui(new Ui::AxisLimitsDialog)
{
  ui->setupUi(this);

  _limits.min = (-numeric_limits<double>::max() / 2);
  _limits.max = (numeric_limits<double>::max() / 2);

  ui->lineEditMinY->setValidator(new QDoubleValidator(this));
  ui->lineEditMaxY->setValidator(new QDoubleValidator(this));
}

AxisLimitsDialog::~AxisLimitsDialog()
{
  delete ui;
}

void AxisLimitsDialog::setDefaultRange(PlotData::RangeValue range)
{
  _parent_limits = range;

  if (!ui->checkBoxMinY->isChecked())
  {
    ui->lineEditMinY->setText(QString::number(_parent_limits.min));
  }
  if (!ui->checkBoxMaxY->isChecked())
  {
    ui->lineEditMaxY->setText(QString::number(_parent_limits.max));
  }
}

void AxisLimitsDialog::enableMin(bool enabled, double value)
{
  _parent_limits.min = value;
  ui->lineEditMinY->setText(QString::number(_parent_limits.min));
  ui->checkBoxMinY->setChecked(enabled);
}

void AxisLimitsDialog::enableMax(bool enabled, double value)
{
  _parent_limits.max = value;
  ui->lineEditMaxY->setText(QString::number(_parent_limits.max));
  ui->checkBoxMaxY->setChecked(enabled);
}

bool AxisLimitsDialog::limitsEnabled() const
{
  return ui->checkBoxMinY->isChecked() || ui->checkBoxMaxY->isChecked();
}

void AxisLimitsDialog::on_checkBoxMinY_toggled(bool checked)
{
  ui->lineEditMinY->setEnabled(checked);
  ui->pushButtonMinY->setEnabled(checked);
}

void AxisLimitsDialog::on_checkBoxMaxY_toggled(bool checked)
{
  ui->lineEditMaxY->setEnabled(checked);
  ui->pushButtonMaxY->setEnabled(checked);
}

void AxisLimitsDialog::on_pushButtonDone_pressed()
{
  double ymin = -numeric_limits<double>::max();
  double ymax = numeric_limits<double>::max();

  if (!ui->lineEditMinY->text().isEmpty())
  {
    ymin = ui->lineEditMinY->text().toDouble();
  }

  if (!ui->lineEditMaxY->text().isEmpty())
  {
    ymax = ui->lineEditMaxY->text().toDouble();
  }

  if (ymin > ymax)
  {
    // swap
    ui->lineEditMinY->setText(QString::number(ymax));
    ui->lineEditMaxY->setText(QString::number(ymin));
  }
  else
  {
    _limits.min = (ui->checkBoxMinY->isChecked() ? ymin : -numeric_limits<double>::max() / 2);
    _limits.max = (ui->checkBoxMaxY->isChecked() ? ymax : numeric_limits<double>::max() / 2);
    this->accept();
  }
}

void AxisLimitsDialog::on_pushButtonMinY_pressed()
{
  ui->lineEditMinY->setText(QString::number(_parent_limits.min));
}

void AxisLimitsDialog::on_pushButtonMaxY_pressed()
{
  ui->lineEditMaxY->setText(QString::number(_parent_limits.max));
}

void AxisLimitsDialog::closeEvent(QCloseEvent* event)
{
  on_pushButtonDone_pressed();
}
