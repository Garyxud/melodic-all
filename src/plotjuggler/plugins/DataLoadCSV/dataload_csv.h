#pragma once

#include <QObject>
#include <QtPlugin>
#include "PlotJuggler/dataloader_base.h"

class DataLoadCSV : public DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader"
                        "../dataloader.json")
  Q_INTERFACES(DataLoader)

public:
  DataLoadCSV();
  virtual const std::vector<const char*>& compatibleFileExtensions() const override;

  virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

  virtual ~DataLoadCSV();

  virtual const char* name() const override
  {
    return "DataLoad CSV";
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

protected:
  QSize parseHeader(QFile* file, std::vector<std::string>& ordered_names);

private:
  std::vector<const char*> _extensions;

  std::string _default_time_axis;
};


