#include "custom_function.h"

#include <limits>
#include <QFile>
#include <QMessageBox>
#include <QElapsedTimer>
#include "lua_custom_function.h"
#include "qml_custom_function.h"

CustomFunction::CustomFunction(const std::string& linkedPlot, const SnippetData& snippet)
  : CustomFunction(linkedPlot, snippet.name.toStdString(), snippet.globalVars, snippet.equation)
{
}

void CustomFunction::clear()
{
  initEngine();
}

QStringList CustomFunction::getChannelsFromFuntion(const QString& function)
{
  QStringList output;
  int offset = 0;
  while (true)
  {
    int pos1 = function.indexOf("$$", offset);
    if (pos1 == -1)
    {
      break;
    }

    int pos2 = function.indexOf("$$", pos1 + 2);
    if (pos2 == -1)
    {
      return {};
    }
    output.push_back(function.mid(pos1 + 2, pos2 - pos1 - 2));
    offset = pos2 + 2;
  }
  return output;
}

void CustomFunction::createReplacedFunction(int index_offset)
{
  QString qLinkedPlot = QString::fromStdString(_linked_plot_name);

  QString replaced_equation = _function;
  while (true)
  {
    int pos1 = replaced_equation.indexOf("$$");
    if (pos1 == -1)
    {
      break;
    }

    int pos2 = replaced_equation.indexOf("$$", pos1 + 2);
    if (pos2 == -1)
    {
      throw std::runtime_error("syntax error : invalid use of $$ macro");
    }

    QString channel_name = replaced_equation.mid(pos1 + 2, pos2 - pos1 - 2);

    if (channel_name == qLinkedPlot)
    {
      // special case : user entered linkedPlot ; no need to add another channel
      replaced_equation.replace(QStringLiteral("$$%1$$").arg(channel_name), QStringLiteral("value"));
    }
    else
    {
      QString jsExpression = QString("CHANNEL_VALUES[%1]").arg(_used_channels.size() + index_offset);
      replaced_equation.replace(QStringLiteral("$$%1$$").arg(channel_name), jsExpression);
      _used_channels.push_back(channel_name.toStdString());
    }
  }
  _function_replaced = replaced_equation;
}

CustomFunction::CustomFunction(const std::string& linkedPlot, const std::string& plotName, const QString& globalVars,
                               const QString& function)
  : _linked_plot_name(linkedPlot), _plot_name(plotName), _global_vars(globalVars), _function(function)
{
}

void CustomFunction::calculateAndAdd(PlotDataMapRef& plotData)
{
  bool newly_added = false;

  auto dst_data_it = plotData.numeric.find(_plot_name);
  if (dst_data_it == plotData.numeric.end())
  {
    dst_data_it = plotData.addNumeric(_plot_name);
    newly_added = true;
  }

  PlotData& dst_data = dst_data_it->second;
  dst_data.clear();

  try
  {
    calculate(plotData, &dst_data);
  }
  catch (...)
  {
    if (newly_added)
    {
      plotData.numeric.erase(dst_data_it);
    }
    std::rethrow_exception(std::current_exception());
  }
}

void CustomFunction::calculate(const PlotDataMapRef& plotData, PlotData* dst_data)
{
  auto src_data_it = plotData.numeric.find(_linked_plot_name);
  if (src_data_it == plotData.numeric.end())
  {
    // failed! keep it empty
    return;
  }

  const PlotData& src_data = src_data_it->second;
  if (src_data.size() == 0)
  {
    return;
  }

  // clean up old data
  dst_data->setMaximumRangeX(src_data.maximumRangeX());

  std::vector<const PlotData*> channel_data;
  channel_data.reserve(_used_channels.size());

  for (const auto& channel : _used_channels)
  {
    auto it = plotData.numeric.find(channel);
    if (it == plotData.numeric.end())
    {
      throw std::runtime_error("Invalid channel name");
    }
    const PlotData* chan_data = &(it->second);
    channel_data.push_back(chan_data);
  }

  double last_updated_stamp = -std::numeric_limits<double>::max();
  if (dst_data->size() != 0)
  {
    last_updated_stamp = dst_data->back().x;
  }

  for (size_t i = 0; i < src_data.size(); ++i)
  {
    if (src_data.at(i).x > last_updated_stamp)
    {
      dst_data->pushBack(calculatePoint(src_data, channel_data, i));
    }
  }
}

const std::string& CustomFunction::name() const
{
  return _plot_name;
}

const std::string& CustomFunction::linkedPlotName() const
{
  return _linked_plot_name;
}

const QString& CustomFunction::globalVars() const
{
  return _global_vars;
}

const QString& CustomFunction::function() const
{
  return _function;
}

QDomElement CustomFunction::xmlSaveState(QDomDocument& doc) const
{
  QDomElement snippet = doc.createElement("snippet");
  snippet.setAttribute("name", QString::fromStdString(_plot_name));
  snippet.setAttribute("language", language());

  QDomElement linked = doc.createElement("linkedPlot");
  linked.appendChild(doc.createTextNode(QString::fromStdString(_linked_plot_name)));
  snippet.appendChild(linked);

  QDomElement global = doc.createElement("global");
  global.appendChild(doc.createTextNode(_global_vars));
  snippet.appendChild(global);

  QDomElement equation = doc.createElement("equation");
  equation.appendChild(doc.createTextNode(_function));
  snippet.appendChild(equation);

  return snippet;
}

CustomPlotPtr CustomFunction::createFromXML(QDomElement& element)
{
  SnippetData snippet;
  auto linkedPlot = element.firstChildElement("linkedPlot").text().trimmed().toStdString();
  snippet.name = element.attribute("name");
  snippet.globalVars = element.firstChildElement("global").text().trimmed();
  snippet.equation = element.firstChildElement("equation").text().trimmed();

  snippet.language = element.attribute("language", "JS");

  if (snippet.language == "LUA")
  {
    return std::make_unique<LuaCustomFunction>(linkedPlot, snippet);
  }
  else if (snippet.language == "JS")
  {
    return std::make_unique<JsCustomFunction>(linkedPlot, snippet);
  }
  else
  {
    throw std::runtime_error("Snippet language not recognized");
  }
}

SnippetsMap GetSnippetsFromXML(const QString& xml_text)
{
  if (xml_text.isEmpty())
    return {};

  QDomDocument doc;
  QString parseErrorMsg;
  int parseErrorLine;
  if (!doc.setContent(xml_text, &parseErrorMsg, &parseErrorLine))
  {
    QMessageBox::critical(
        nullptr, "Error",
        QString("Failed to parse snippets (xml), error %1 at line %2").arg(parseErrorMsg).arg(parseErrorLine));
    return {};
  }
  else
  {
    QDomElement snippets_element = doc.documentElement();
    return GetSnippetsFromXML(snippets_element);
  }
}

SnippetsMap GetSnippetsFromXML(const QDomElement& snippets_element)
{
  SnippetsMap snippets;

  for (auto elem = snippets_element.firstChildElement("snippet"); !elem.isNull(); elem = elem.nextSiblingElement("snipp"
                                                                                                                 "et"))
  {
    SnippetData snippet;
    snippet.name = elem.attribute("name");
    snippet.language = elem.attribute("language", "JS");
    snippet.globalVars = elem.firstChildElement("global").text().trimmed();
    snippet.equation = elem.firstChildElement("equation").text().trimmed();
    snippets.insert({ snippet.name, snippet });
  }
  return snippets;
}

QDomElement ExportSnippets(const SnippetsMap& snippets, QDomDocument& doc)
{
  auto snippets_root = doc.createElement("snippets");

  for (const auto& it : snippets)
  {
    const auto& snippet = it.second;

    auto element = doc.createElement("snippet");
    element.setAttribute("name", it.first);
    element.setAttribute("language", snippet.language);

    auto global_el = doc.createElement("global");
    global_el.appendChild(doc.createTextNode(snippet.globalVars));

    auto equation_el = doc.createElement("equation");
    equation_el.appendChild(doc.createTextNode(snippet.equation));

    element.appendChild(global_el);
    element.appendChild(equation_el);
    snippets_root.appendChild(element);
  }
  return snippets_root;
}
