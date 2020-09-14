#pragma once

#include <QtPlugin>
#include <thread>
#include "PlotJuggler/datastreamer_base.h"

class DataStreamSample : public DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataStreamer"
                        "../datastreamer.json")
  Q_INTERFACES(DataStreamer)

public:
  DataStreamSample();

  virtual bool start(QStringList*) override;

  virtual void shutdown() override;

  virtual bool isRunning() const override;

  virtual ~DataStreamSample();

  virtual const char* name() const override
  {
    return "Dummy Streamer";
  }

  virtual bool isDebugPlugin() override
  {
    return true;
  }

  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  virtual bool xmlLoadState(const QDomElement& parent_element) override;

private:
  struct Parameters
  {
    double A, B, C, D;
  };

  void loop();

  std::thread _thread;

  bool _running;

  std::map<std::string, Parameters> _parameters;

  void pushSingleCycle();
};

