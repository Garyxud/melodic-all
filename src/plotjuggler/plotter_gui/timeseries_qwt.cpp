#include "timeseries_qwt.h"
#include <limits>
#include <stdexcept>
#include <QMessageBox>
#include <QPushButton>
#include <QString>

TimeseriesQwt::TimeseriesQwt(const PlotData* source_data, const PlotData* transformed_data)
  : DataSeriesBase(transformed_data), _source_data(source_data), _cached_data("")
{
}

PlotData::RangeValueOpt TimeseriesQwt::getVisualizationRangeY(PlotData::RangeTime range_X)
{
  int first_index = transformedData()->getIndexFromX(range_X.min);
  int last_index = transformedData()->getIndexFromX(range_X.max);

  if (first_index > last_index || first_index < 0 || last_index < 0)
  {
    return PlotData::RangeValueOpt();
  }

  if (first_index == 0 && last_index == transformedData()->size() - 1)
  {
    return PlotData::RangeValueOpt({ _bounding_box.bottom(), _bounding_box.top() });
  }

  double min_y = (std::numeric_limits<double>::max());
  double max_y = (-std::numeric_limits<double>::max());

  for (size_t i = first_index; i < last_index; i++)
  {
    const double Y = sample(i).y();
    min_y = std::min(min_y, Y);
    max_y = std::max(max_y, Y);
  }
  return PlotData::RangeValueOpt({ min_y, max_y });
}

nonstd::optional<QPointF> TimeseriesQwt::sampleFromTime(double t)
{
  int index = transformedData()->getIndexFromX(t);
  if (index < 0)
  {
    return nonstd::optional<QPointF>();
  }
  const auto& p = transformedData()->at(size_t(index));
  return QPointF(p.x, p.y);
}

bool Timeseries_NoTransform::updateCache()
{
  calculateBoundingBox();
  return true;
}

bool Timeseries_1stDerivative::updateCache()
{
  size_t data_size = _source_data->size();

  if (data_size <= 1)
  {
    _cached_data.clear();
    _bounding_box = QRectF();
    return true;
  }

  data_size = data_size - 1;
  _cached_data.resize(data_size);

  for (size_t i = 0; i < data_size; i++)
  {
    const auto& p0 = _source_data->at(i);
    const auto& p1 = _source_data->at(i + 1);
    const auto delta = p1.x - p0.x;
    const auto vel = (p1.y - p0.y) / delta;
    QPointF p((p1.x + p0.x) * 0.5, vel);
    _cached_data[i] = { p.x(), p.y() };
  }

  calculateBoundingBox();
  return true;
}

bool Timeseries_2ndDerivative::updateCache()
{
  size_t data_size = _source_data->size();

  if (data_size <= 2)
  {
    _cached_data.clear();
    _bounding_box = QRectF();
    return true;
  }

  data_size = data_size - 2;
  _cached_data.resize(data_size);

  for (size_t i = 0; i < data_size; i++)
  {
    const auto& p0 = _source_data->at(i);
    const auto& p1 = _source_data->at(i + 1);
    const auto& p2 = _source_data->at(i + 2);
    const auto delta = (p2.x - p0.x) * 0.5;
    const auto acc = (p2.y - 2.0 * p1.y + p0.y) / (delta * delta);
    QPointF p((p2.x + p0.x) * 0.5, acc);
    _cached_data[i] = { p.x(), p.y() };
  }

  calculateBoundingBox();
  return true;
}
