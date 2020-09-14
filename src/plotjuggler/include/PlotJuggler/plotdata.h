#ifndef PLOTDATA_RAW_H
#define PLOTDATA_RAW_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <deque>
#include "PlotJuggler/optional.hpp"
#include "PlotJuggler/any.hpp"
#include <QDebug>
#include <QColor>
#include <type_traits>
#include <cmath>
#include <cstdlib>
#include <unordered_map>

inline double Abs(double val)
{
  return val < 0 ? -val : val;
}

template <typename Time, typename Value>
class PlotDataGeneric
{
public:
  struct RangeTime
  {
    Time min;
    Time max;
  };

  struct RangeValue
  {
    Value min;
    Value max;
  };

  typedef nonstd::optional<RangeTime> RangeTimeOpt;
  typedef nonstd::optional<RangeValue> RangeValueOpt;

  class Point
  {
  public:
    Time x;
    Value y;
    Point(Time _x, Value _y) : x(_x), y(_y)
    {
    }
    Point() = default;
  };

  enum
  {
    MAX_CAPACITY = 1024 * 1024,
    ASYNC_BUFFER_CAPACITY = 1024
  };

  typedef Time TimeType;

  typedef Value ValueType;

  typedef typename std::deque<Point>::iterator Iterator;

  typedef typename std::deque<Point>::const_iterator ConstIterator;

  PlotDataGeneric(const std::string& name);

  PlotDataGeneric(const PlotDataGeneric<Time, Value>& other) = delete;

  PlotDataGeneric(PlotDataGeneric&& other)
  {
    _name = std::move(other._name);
    _points = std::move(other._points);
    _color_hint = std::move(other._color_hint);
    _max_range_X = other._max_range_X;
  }

  void swapData(PlotDataGeneric<Time, Value>& other)
  {
    std::swap(_points, other._points);
  }

  PlotDataGeneric& operator=(const PlotDataGeneric<Time, Value>& other) = delete;

  virtual ~PlotDataGeneric()
  {
  }

  const std::string& name() const
  {
    return _name;
  }

  virtual size_t size() const;

  int getIndexFromX(Time x) const;

  nonstd::optional<Value> getYfromX(Time x) const;

  const Point& at(size_t index) const;

  Point& at(size_t index);

  const Point& operator[](size_t index) const
  {
    return at(index);
  }

  Point& operator[](size_t index)
  {
    return at(index);
  }

  void clear();

  void pushBack(Point p);

  QColor getColorHint() const;

  void setColorHint(QColor color);

  void setMaximumRangeX(Time max_range);

  Time maximumRangeX() const
  {
    return _max_range_X;
  }

  const Point& front() const
  {
    return _points.front();
  }

  const Point& back() const
  {
    return _points.back();
  }

  ConstIterator begin() const
  {
    return _points.begin();
  }

  ConstIterator end() const
  {
    return _points.end();
  }

  Iterator begin()
  {
    return _points.begin();
  }

  Iterator end()
  {
    return _points.end();
  }

  void resize(size_t new_size)
  {
    _points.resize(new_size);
  }

  void popFront()
  {
    _points.pop_front();
  }

protected:
  std::string _name;
  std::deque<Point> _points;
  QColor _color_hint;

private:
  Time _max_range_X;
};

typedef PlotDataGeneric<double, double> PlotData;
typedef PlotDataGeneric<double, nonstd::any> PlotDataAny;

typedef struct
{
  std::unordered_map<std::string, PlotData> numeric;
  std::unordered_map<std::string, PlotDataAny> user_defined;

  std::unordered_map<std::string, PlotData>::iterator addNumeric(const std::string& name)
  {
    return numeric.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name)).first;
  }

  std::unordered_map<std::string, PlotDataAny>::iterator addUserDefined(const std::string& name)
  {
    return user_defined.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name))
        .first;
  }

} PlotDataMapRef;

//-----------------------------------
template <typename Value>
inline void AddPrefixToPlotData(const std::string& prefix, std::unordered_map<std::string, Value>& data)
{
  if (prefix.empty())
    return;

  std::unordered_map<std::string, Value> temp;

  for (auto& it : data)
  {
    std::string key;
    key.reserve(prefix.size() + 2 + it.first.size());
    if (it.first.front() == '/')
    {
      key = prefix + it.first;
    }
    else
    {
      key = prefix + "/" + it.first;
    }

    auto new_plot =
        temp.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(key)).first;

    new_plot->second.swapData(it.second);
  }
  std::swap(data, temp);
}

// template < typename Time, typename Value>
// inline PlotDataGeneric<Time, Value>::PlotDataGeneric():
//  _max_range_X( std::numeric_limits<Time>::max() )
//  , _color_hint(Qt::black)
//{
//    static_assert( std::is_arithmetic<Time>::value ,"Only numbers can be used as time");
//}

template <typename Time, typename Value>
inline PlotDataGeneric<Time, Value>::PlotDataGeneric(const std::string& name)
  : _max_range_X(std::numeric_limits<Time>::max()), _color_hint(Qt::black), _name(name)
{
  static_assert(std::is_arithmetic<Time>::value, "Only numbers can be used as time");
}

template <typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::pushBack(Point point)
{
  _points.push_back(point);

  while (_points.size() > 2 && (_points.back().x - _points.front().x) > _max_range_X)
  {
    _points.pop_front();
  }
}

template <>  // template specialization
inline void PlotDataGeneric<double, double>::pushBack(Point point)
{
  if (std::isinf(point.y) || std::isnan(point.y))
  {
    return;  // skip
  }
  _points.push_back(point);

  while (_points.size() > 2 && (_points.back().x - _points.front().x) > _max_range_X)
  {
    _points.pop_front();
  }
}

template <typename Time, typename Value>
inline int PlotDataGeneric<Time, Value>::getIndexFromX(Time x) const
{
  if (_points.size() == 0)
  {
    return -1;
  }
  auto lower = std::lower_bound(_points.begin(), _points.end(), Point(x, 0),
                                [](const Point& a, const Point& b) { return a.x < b.x; });
  auto index = std::distance(_points.begin(), lower);

  if (index >= _points.size())
  {
    return _points.size() - 1;
  }
  if (index < 0)
  {
    return 0;
  }

  if (index > 0)
  {
    if (Abs(_points[index - 1].x - x) < Abs(_points[index].x - x))
    {
      return index - 1;
    }
    else
    {
      return index;
    }
  }
  return index;
}

template <typename Time, typename Value>
inline nonstd::optional<Value> PlotDataGeneric<Time, Value>::getYfromX(Time x) const
{
  int index = getIndexFromX(x);
  if (index == -1)
  {
    return nonstd::optional<Value>();
  }
  return _points.at(index).y;
}

template <typename Time, typename Value>
inline const typename PlotDataGeneric<Time, Value>::Point& PlotDataGeneric<Time, Value>::at(size_t index) const
{
  return _points[index];
}

template <typename Time, typename Value>
inline typename PlotDataGeneric<Time, Value>::Point& PlotDataGeneric<Time, Value>::at(size_t index)
{
  return _points[index];
}

template <typename Time, typename Value>
void PlotDataGeneric<Time, Value>::clear()
{
  _points.clear();
}

template <typename Time, typename Value>
inline size_t PlotDataGeneric<Time, Value>::size() const
{
  return _points.size();
}

template <typename Time, typename Value>
inline QColor PlotDataGeneric<Time, Value>::getColorHint() const
{
  return _color_hint;
}

template <typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setColorHint(QColor color)
{
  _color_hint = color;
}

template <typename Time, typename Value>
inline void PlotDataGeneric<Time, Value>::setMaximumRangeX(Time max_range)
{
  _max_range_X = max_range;
  while (_points.size() > 2 && _points.back().x - _points.front().x > _max_range_X)
  {
    _points.pop_front();
  }
}

#endif  // PLOTDATA_H
