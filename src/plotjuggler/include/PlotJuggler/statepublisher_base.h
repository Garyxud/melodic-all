#ifndef STATEPUBLISHER_TEMPLATE_H
#define STATEPUBLISHER_TEMPLATE_H

#include <QObject>
#include <QtPlugin>
#include <QMenu>
#include <QDomElement>
#include <functional>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

class StatePublisher : public PlotJugglerPlugin
{
  Q_OBJECT
public:
  virtual bool enabled() const = 0;

  virtual void updateState(double current_time) = 0;

  virtual void play(double interval) = 0;

  virtual ~StatePublisher() = default;

  virtual void setEnabled(bool enabled)
  {
    auto prev = _action->blockSignals(true);
    _action->setChecked(enabled);
    _action->blockSignals(prev);
  }

  virtual void setParentMenu(QMenu* parent_menu, QAction* parent_action)
  {
    _menu = parent_menu;
    _action = parent_action;
  }

  virtual QWidget* embeddedWidget()
  {
    return nullptr;
  }

  void setDataMap(const PlotDataMapRef* datamap)
  {
    _datamap = datamap;
  }

signals:
  void connectionClosed();

protected:
  QMenu* _menu;
  QAction* _action;
  const PlotDataMapRef* _datamap;
};

QT_BEGIN_NAMESPACE

#define StatePublisher_iid "com.icarustechnology.PlotJuggler.StatePublisher"

Q_DECLARE_INTERFACE(StatePublisher, StatePublisher_iid)

QT_END_NAMESPACE

#endif
