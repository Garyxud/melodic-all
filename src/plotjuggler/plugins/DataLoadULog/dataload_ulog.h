#pragma once

#include <QObject>
#include <QtPlugin>
#include <QWidget>
#include "PlotJuggler/dataloader_base.h"

class DataLoadULog : public DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader"
                        "../dataloader.json")
  Q_INTERFACES(DataLoader)

public:
  DataLoadULog();

  const std::vector<const char*>& compatibleFileExtensions() const override;

  bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

  ~DataLoadULog() override;

  const char* name() const override
  {
    return "DataLoad ULog";
  }

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

private:
  std::string _default_time_axis;
  QWidget* _main_win;
};

