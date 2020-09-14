#include "qml_custom_function.h"

JsCustomFunction::JsCustomFunction(const std::string& linkedPlot, const SnippetData& snippet)
  : CustomFunction(linkedPlot, snippet)
{
  createReplacedFunction(0);
  initEngine();
}

void JsCustomFunction::initEngine()
{
  _qml_engine = std::make_unique<QJSEngine>();
  QJSValue globalVarResult = _qml_engine->evaluate(_global_vars);
  if (globalVarResult.isError())
  {
    throw std::runtime_error("JS Engine : " + globalVarResult.toString().toStdString());
  }

  QString calcMethodStr = QString("function calc(time, value, CHANNEL_VALUES)"
                                  "{with (Math){\n%1\n}}")
                              .arg(_function_replaced);
  _qml_engine->evaluate(calcMethodStr);

  _qml_function = _qml_engine->evaluate("calc");

  if (_qml_function.isError())
  {
    throw std::runtime_error("JS Engine : " + _qml_function.toString().toStdString());
  }

  _chan_values_qml = _qml_engine->newArray(static_cast<quint32>(_used_channels.size()));
}

PlotData::Point JsCustomFunction::calculatePoint(const PlotData& src_data,
                                                 const std::vector<const PlotData*>& channels_data, size_t point_index)
{
  const PlotData::Point& old_point = src_data.at(point_index);

  for (int chan_index = 0; chan_index < channels_data.size(); chan_index++)
  {
    double value;
    const auto& chan_data = channels_data[chan_index];
    int index = chan_data->getIndexFromX(old_point.x);
    if (index != -1)
    {
      value = chan_data->at(index).y;
    }
    else
    {
      value = std::numeric_limits<double>::quiet_NaN();
    }
    _chan_values_qml.setProperty(static_cast<quint32>(chan_index), QJSValue(value));
  }

  PlotData::Point new_point;
  new_point.x = old_point.x;

  QJSValue result = _qml_function.call({ QJSValue(old_point.x), QJSValue(old_point.y), _chan_values_qml });
  if (result.isError())
  {
    throw std::runtime_error("JS Engine : " + result.toString().toStdString());
  }

  if (result.isArray())
  {
    const int length = result.property("length").toInt();
    if (length == 2)
    {
      new_point.x = result.property(0).toNumber();
      new_point.y = result.property(1).toNumber();
    }
    else
    {
      throw std::runtime_error("JS Engine : if you return an array, the size must be "
                               "2 (time/value pair)");
    }
  }
  else
  {
    new_point.y = result.toNumber();
  }
  return new_point;
}
