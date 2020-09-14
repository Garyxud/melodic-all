#ifndef PLOTDATA_QWT_H
#define PLOTDATA_QWT_H

#include "series_data.h"
#include "PlotJuggler/plotdata.h"

class TimeseriesQwt : public DataSeriesBase
{
public:
  TimeseriesQwt(const PlotData* source_data, const PlotData* transformed_data);

  PlotData::RangeValueOpt getVisualizationRangeY(PlotData::RangeTime range_X) override;

  nonstd::optional<QPointF> sampleFromTime(double t) override;

protected:
  const PlotData* _source_data;
  PlotData _cached_data;
};

//---------------------------------------------------------

class Timeseries_NoTransform : public TimeseriesQwt
{
public:
  Timeseries_NoTransform(const PlotData* source_data) : TimeseriesQwt(source_data, source_data)
  {
    updateCache();
  }

  bool updateCache() override;
};

class Timeseries_1stDerivative : public TimeseriesQwt
{
public:
  Timeseries_1stDerivative(const PlotData* source_data) : TimeseriesQwt(source_data, &_cached_data)
  {
    updateCache();
  }

  bool updateCache() override;
};

class Timeseries_2ndDerivative : public TimeseriesQwt
{
public:
  Timeseries_2ndDerivative(const PlotData* source_data) : TimeseriesQwt(source_data, &_cached_data)
  {
    updateCache();
  }

  bool updateCache() override;
};

#endif  // PLOTDATA_H
