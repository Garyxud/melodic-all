#pragma once

#include <memory>
#include <string>
#include <QWidget>
#include <QString>
#include <QDomDocument>
#include <QString>
#include "PlotJuggler/plotdata.h"

class CustomFunction;

typedef std::shared_ptr<CustomFunction> CustomPlotPtr;
typedef std::unordered_map<std::string, CustomPlotPtr> CustomPlotMap;

struct SnippetData
{
  QString language;
  QString name;
  QString globalVars;
  QString equation;
};

typedef std::map<QString, SnippetData> SnippetsMap;

SnippetsMap GetSnippetsFromXML(const QString& xml_text);

SnippetsMap GetSnippetsFromXML(const QDomElement& snippets_element);

QDomElement ExportSnippets(const SnippetsMap& snippets, QDomDocument& destination_doc);

class CustomFunction
{
public:
  CustomFunction(const std::string& linkedPlot, const std::string& plotName, const QString& globalVars,
                 const QString& function);

  CustomFunction(const std::string& linkedPlot, const SnippetData& snippet);

  void clear();

  void calculateAndAdd(PlotDataMapRef& plotData);

  const std::string& name() const;

  const std::string& linkedPlotName() const;

  const QString& globalVars() const;

  const QString& function() const;

  QDomElement xmlSaveState(QDomDocument& doc) const;

  static CustomPlotPtr createFromXML(QDomElement& element);

  static QStringList getChannelsFromFuntion(const QString& function);

  void calculate(const PlotDataMapRef& plotData, PlotData* dst_data);

  virtual QString language() const = 0;

  virtual void initEngine() = 0;

  virtual PlotData::Point calculatePoint(const PlotData& src_data, const std::vector<const PlotData*>& channels_data,
                                         size_t point_index) = 0;

protected:
  const std::string _linked_plot_name;
  const std::string _plot_name;
  const QString _global_vars;
  const QString _function;
  QString _function_replaced;
  std::vector<std::string> _used_channels;
  void createReplacedFunction(int index_offset);
};
