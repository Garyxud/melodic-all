#ifndef SUBWINDOW_H
#define SUBWINDOW_H

#include <QMainWindow>
#include <set>
#include "tabbedplotwidget.h"

class SubWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit SubWindow(QString name, PlotMatrix* first_tab, PlotDataMapRef& mapped_data, QMainWindow* parent_window);

  virtual ~SubWindow();

  TabbedPlotWidget* tabbedWidget()
  {
    return tabbed_widget_;
  }

signals:

  void tabAdded();

protected:
  virtual void closeEvent(QCloseEvent* event) override;
  TabbedPlotWidget* tabbed_widget_;
};

#endif  // SUBWINDOW_H
