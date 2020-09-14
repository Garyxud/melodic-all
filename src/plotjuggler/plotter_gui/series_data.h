#ifndef SERIES_DATA_H
#define SERIES_DATA_H

#include "PlotJuggler/plotdata.h"
#include "qwt_series_data.h"

class DataSeriesBase : public QwtSeriesData<QPointF>
{
public:
  DataSeriesBase(const PlotData* transformed) : _transformed_data(transformed), _time_offset(0)
  {
  }

  virtual QPointF sample(size_t i) const override
  {
    const auto& p = _transformed_data->at(i);
    return QPointF(p.x - _time_offset, p.y);
  }

  virtual size_t size() const override
  {
    return _transformed_data->size();
  }

  QRectF boundingRect() const override
  {
    QRectF box = _bounding_box;
    box.setLeft(_bounding_box.left() - _time_offset);
    box.setRight(_bounding_box.right() - _time_offset);
    return box;
  }

  void setTimeOffset(double offset)
  {
    _time_offset = offset;
  }

  void calculateBoundingBox();

  virtual PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) = 0;

  virtual nonstd::optional<QPointF> sampleFromTime(double t) = 0;

  virtual bool updateCache() = 0;

  virtual PlotData::RangeTimeOpt getVisualizationRangeX()
  {
    if (this->size() < 2)
      return PlotData::RangeTimeOpt();
    else
    {
      return PlotData::RangeTimeOpt({ _bounding_box.left() - _time_offset, _bounding_box.right() - _time_offset });
    }
  }

  const PlotData* transformedData() const
  {
    return _transformed_data;
  }

protected:
  QRectF _bounding_box;

private:
  const PlotData* _transformed_data;
  double _time_offset;
};

//--------------------------------------------
inline void DataSeriesBase::calculateBoundingBox()
{
  if (_transformed_data->size() == 0)
  {
    _bounding_box = QRectF();
    return;
  }

  double min_y = _transformed_data->front().y;
  double max_y = _transformed_data->front().y;

  for (const auto& p : *_transformed_data)
  {
    if (p.y < min_y)
    {
      min_y = p.y;
    }
    else if (p.y > max_y)
    {
      max_y = p.y;
    }
  }

  _bounding_box.setLeft(_transformed_data->front().x);
  _bounding_box.setRight(_transformed_data->back().x);
  _bounding_box.setBottom(min_y);
  _bounding_box.setTop(max_y);
}

#endif  // SERIES_DATA_H
