#ifndef _PLOT_MATRIX_H_
#define _PLOT_MATRIX_H_

#include <qframe.h>
#include <QGridLayout>
#include "qwt_plot.h"
#include "plotwidget.h"

class PlotMatrix : public QFrame
{
  Q_OBJECT

public:
  PlotMatrix(QString name, PlotDataMapRef& datamap, QWidget* parent = nullptr);
  virtual ~PlotMatrix();

  void addRow();
  void addColumn();
  void removeColumn(unsigned column_to_delete);
  void removeRow(unsigned row_to_delete);

  void removeEmpty();

  unsigned rowsCount() const;
  unsigned colsCount() const;
  unsigned plotCount() const;

  bool isRowEmpty(unsigned row) const;
  bool isColumnEmpty(unsigned row) const;

  PlotWidget* plotAt(unsigned row, unsigned column);
  const PlotWidget* plotAt(unsigned row, unsigned column) const;

  PlotWidget* plotAt(unsigned index);
  const PlotWidget* plotAt(unsigned index) const;

  void setAxisScale(QwtPlot::Axis axisId, unsigned row, unsigned col, double min, double max, double step = 0);

  QDomElement xmlSaveState(QDomDocument& doc) const;

  bool xmlLoadState(QDomElement& plotmatrix_element);

  void updateLayout();

  void replot();

  void setHorizontalLink(bool linked);

  void setName(const QString& new_name);

  const QString& name() const;

  QGridLayout* gridLayout();

public slots:
  void maximumZoomOutHorizontal();

  void maximumZoomOutVertical();

  void maximumZoomOut();

private slots:
  //  void swapWidgetByName(QString name_a, QString name_b);
  void on_singlePlotScaleChanged(PlotWidget* modified_plot, QRectF range);

  void on_legendSizeChanged(int point_size);

private:
  void alignAxes(unsigned rowOrColumn, QwtPlot::Axis axisId);
  void alignScaleBorder(unsigned rowOrColumn, QwtPlot::Axis axisId);
  PlotWidget* addPlotWidget(unsigned row, unsigned col);
  void swapPlots(unsigned rowA, unsigned colA, unsigned rowB, unsigned colB);

  QGridLayout* _layout;
  unsigned _num_rows;
  unsigned _num_cols;
  bool _horizontal_link;

  PlotDataMapRef& _mapped_data;

  QString _name;
  int _legend_point_size;

signals:
  void plotAdded(PlotWidget*);
  void undoableChange();
};

#endif
