#include "subwindow.h"
#include <QDebug>
#include <QSettings>
#include <QMessageBox>
#include <QCloseEvent>
#include "mainwindow.h"

SubWindow::SubWindow(QString name, PlotMatrix* first_tab, PlotDataMapRef& mapped_data, QMainWindow* parent_window)
  : QMainWindow(parent_window)
{
  tabbed_widget_ = new TabbedPlotWidget(name, parent_window, first_tab, mapped_data, this);

  MainWindow* parent_mainwin = dynamic_cast<MainWindow*>(parent_window);

  connect(parent_mainwin, &MainWindow::stylesheetChanged, tabbed_widget_, &TabbedPlotWidget::on_stylesheetChanged);

  tabbed_widget_->on_stylesheetChanged(parent_mainwin->styleDirectory());

  this->setCentralWidget(tabbed_widget_);

  Qt::WindowFlags flags = this->windowFlags();
  this->setWindowFlags(flags | Qt::SubWindow);
  this->setWindowTitle(tabbed_widget_->name());

  QSettings settings;
  restoreGeometry(settings.value(QString("SubWindow.%1.geometry").arg(name)).toByteArray());

  this->setAttribute(Qt::WA_DeleteOnClose);
}

SubWindow::~SubWindow()
{
  QSettings settings;
  settings.setValue(QString("SubWindow.%1.geometry").arg(tabbedWidget()->name()), saveGeometry());
  tabbed_widget_->close();
}

void SubWindow::closeEvent(QCloseEvent* event)
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, tr("Warning"), tr("Are you sure that you want to destroy this window?\n"),
                                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

  if (reply != QMessageBox::Yes)
  {
    event->ignore();
  }
  else
  {
    event->accept();
  }
}
