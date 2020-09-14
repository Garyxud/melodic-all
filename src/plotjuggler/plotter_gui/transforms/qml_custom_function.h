#ifndef QML_CUSTOM_FUNCTION_H
#define QML_CUSTOM_FUNCTION_H

#include "custom_function.h"
#include <QJSEngine>

class JsCustomFunction : public CustomFunction
{
public:
  JsCustomFunction(const std::string& linkedPlot, const SnippetData& snippet);

  void initEngine() override;

  PlotData::Point calculatePoint(const PlotData& src_data, const std::vector<const PlotData*>& channels_data,
                                 size_t point_index) override;

  QString language() const override
  {
    return "JS";
  }

private:
  std::unique_ptr<QJSEngine> _qml_engine;
  QJSValue _qml_function;
  QJSValue _chan_values_qml;
};
#endif  // QML_CUSTOM_FUNCTION_H
