#ifndef PJ_PLUGIN_H
#define PJ_PLUGIN_H

#include <QtPlugin>
#include <QMenu>
#include <QWidget>
#include <QDomDocument>
#include "PlotJuggler/plotdata.h"

class PlotJugglerPlugin : public QObject
{
public:
  PlotJugglerPlugin()
  {
  }

  virtual const char* name() const = 0;

  virtual bool isDebugPlugin()
  {
    return false;
  }

  virtual QWidget* optionsWidget()
  {
    return nullptr;
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const
  {
    return false;
  }

  virtual bool xmlLoadState(const QDomElement& parent_element)
  {
    return false;
  }

  QDomElement xmlSaveState(QDomDocument& doc) const
  {
    QDomElement plugin_elem = doc.createElement("plugin");
    plugin_elem.setAttribute("ID", this->name());
    xmlSaveState(doc, plugin_elem);
    return plugin_elem;
  }

  virtual void addActionsToParentMenu(QMenu* menu)
  {
  }
};

inline std::vector<QString> MoveData(PlotDataMapRef& source, PlotDataMapRef& destination)
{
  std::vector<QString> added_curves;
  for (auto& it : source.numeric)
  {
    const std::string& name = it.first;
    if (it.second.size() > 0 && destination.numeric.count(name) == 0)
    {
      added_curves.push_back(QString::fromStdString(name));
    }
  }

  for (auto& it : source.numeric)
  {
    const std::string& name = it.first;
    auto& source_plot = it.second;
    auto plot_with_same_name = destination.numeric.find(name);

    // this is a new plot
    if (plot_with_same_name == destination.numeric.end())
    {
      plot_with_same_name =
          destination.numeric
              .emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name))
              .first;
    }
    auto& destination_plot = plot_with_same_name->second;
    for (size_t i = 0; i < source_plot.size(); i++)
    {
      destination_plot.pushBack(source_plot.at(i));
    }
    source_plot.clear();
  }

  for (auto& it : source.user_defined)
  {
    const std::string& name = it.first;
    auto& source_plot = it.second;
    auto plot_with_same_name = destination.user_defined.find(name);

    // this is a new plot
    if (plot_with_same_name == destination.user_defined.end())
    {
      plot_with_same_name =
          destination.user_defined
              .emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name))
              .first;
    }
    auto& destination_plot = plot_with_same_name->second;
    for (size_t i = 0; i < source_plot.size(); i++)
    {
      destination_plot.pushBack(source_plot.at(i));
    }
    source_plot.clear();
  }
  return added_curves;
}

#endif  // PJ_PLUGIN_H
