#include "removecurvedialog.h"
#include "ui_removecurvedialog.h"
#include "plotwidget.h"
#include <QDebug>
#include "plotwidget.h"

RemoveCurveDialog::RemoveCurveDialog(PlotWidget* parent)
  : QDialog(parent), ui(new Ui::RemoveCurveDialog), _parent(parent)
{
  ui->setupUi(this);
}

RemoveCurveDialog::~RemoveCurveDialog()
{
  delete ui;
}

void RemoveCurveDialog::addCurveName(const QString& name, const QColor& color)
{
  QListWidgetItem* item = new QListWidgetItem(name);
  item->setForeground(color);
  ui->listCurveWidget->addItem(item);
}

void RemoveCurveDialog::on_pushButtonRemove_pressed()
{
  auto selected_items = ui->listCurveWidget->selectedItems();

  for (const auto item : selected_items)
  {
    if (item->isHidden() == false)
    {
      _parent->removeCurve(item->text().toStdString());
      item->setHidden(true);
    }
  }

  if (ui->listCurveWidget->count() > 0)
  {
    _parent->replot();
  }
  closeIfEmpty();
}

void RemoveCurveDialog::closeIfEmpty()
{
  bool isEmpty = true;
  for (int index = 0; index < ui->listCurveWidget->count(); ++index)
  {
    QListWidgetItem* item = ui->listCurveWidget->item(index);
    if (item->isHidden() == false)
    {
      isEmpty = false;
      break;
    }
  }
  if (isEmpty)
    this->accept();
}
