#ifndef MESSAGEPARSER_TEMPLATE_H
#define MESSAGEPARSER_TEMPLATE_H

#include <QtPlugin>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include "PlotJuggler/plotdata.h"

class MessageRef
{
public:
  explicit MessageRef(const uint8_t* first_ptr, size_t size) : _first_ptr(first_ptr), _size(size)
  {
  }

  explicit MessageRef(const std::vector<uint8_t>& vect) : _first_ptr(vect.data()), _size(vect.size())
  {
  }

  const uint8_t* data() const
  {
    return _first_ptr;
  }

  size_t size() const
  {
    return _size;
  }

private:
  const uint8_t* _first_ptr;
  size_t _size;
};

/**
 * @brief The MessageParser is the base class to create plugins that are able to parse one or
 * multiple Message types.
 * Each message type is uniquely identified by a MessageKey (128 bits, sufficiently large to
 * hold a MD5Sum identifier).
 *
 * You push one or more raw messages using the method pushMessageRef()
 * Once you have done, the result can be copied using plotData()
 */
class MessageParser
{
public:
  virtual ~MessageParser()
  {
  }

  virtual const std::unordered_set<std::string>& getCompatibleKeys() const = 0;

  virtual void pushMessageRef(const std::string& key, const MessageRef& msg, double timestamp) = 0;

  virtual void extractData(PlotDataMapRef& destination, const std::string& prefix) = 0;

protected:
  static void appendData(PlotDataMapRef& destination_plot_map, const std::string& field_name, PlotData& in_data)
  {
    if (in_data.size() == 0)
    {
      return;
    }
    auto plot_pair = destination_plot_map.numeric.find(field_name);
    if ((plot_pair == destination_plot_map.numeric.end()))
    {
      plot_pair = destination_plot_map.addNumeric(field_name);
      plot_pair->second.swapData(in_data);
    }
    else
    {
      PlotData& plot_data = plot_pair->second;
      for (size_t i = 0; i < in_data.size(); i++)
      {
        double val = in_data[i].y;
        if (!std::isnan(val) && !std::isinf(val))
        {
          plot_data.pushBack(in_data[i]);
        }
      }
    }
    in_data.clear();
  }
};

QT_BEGIN_NAMESPACE

#define MessageParser_iid "com.icarustechnology.PlotJuggler.MessageParser"
Q_DECLARE_INTERFACE(MessageParser, MessageParser_iid)

QT_END_NAMESPACE

#endif
