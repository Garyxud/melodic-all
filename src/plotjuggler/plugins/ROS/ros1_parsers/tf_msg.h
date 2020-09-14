#pragma once

#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>
#include "fmt/format.h"
#include "ros1_parser.h"

template <typename TfMsgType>
class TfMsgParserImpl : public BuiltinMessageParser<TfMsgType>
{
public:
  using BaseParser = BuiltinMessageParser<TfMsgType>;

  TfMsgParserImpl(const std::string& topic_name, PlotDataMapRef& plot_data) : BaseParser(topic_name, plot_data)
  {
  }

  void parseMessageImpl(const TfMsgType& msg, double timestamp) override
  {
    auto GetSeries = [&](const std::string& name) { return &BaseParser::getSeries(BaseParser::_plot_data, name); };

    for (const auto& trans : msg.transforms)
    {
      double header_stamp = trans.header.stamp.toSec();
      timestamp = (BaseParser::_use_header_stamp && header_stamp > 0) ? header_stamp : timestamp;

      std::string prefix;
      if (trans.header.frame_id.empty())
      {
        prefix = fmt::format("{}/{}", BaseParser::_topic_name, trans.child_frame_id);
      }
      else
      {
        prefix = fmt::format("{}/{}/{}", BaseParser::_topic_name, trans.header.frame_id, trans.child_frame_id);
      }

      PlotData* series = GetSeries(prefix + "/header/stamp");
      series->pushBack({ timestamp, header_stamp });

      series = GetSeries(prefix + "/header/seq");
      series->pushBack({ timestamp, double(trans.header.seq) });

      series = GetSeries(prefix + "/translation/x");
      series->pushBack({ timestamp, trans.transform.translation.x });

      series = GetSeries(prefix + "/translation/y");
      series->pushBack({ timestamp, trans.transform.translation.y });

      series = GetSeries(prefix + "/translation/z");
      series->pushBack({ timestamp, trans.transform.translation.z });

      series = GetSeries(prefix + "/rotation/x");
      series->pushBack({ timestamp, trans.transform.rotation.x });

      series = GetSeries(prefix + "/rotation/y");
      series->pushBack({ timestamp, trans.transform.rotation.y });

      series = GetSeries(prefix + "/rotation/z");
      series->pushBack({ timestamp, trans.transform.rotation.z });

      series = GetSeries(prefix + "/rotation/w");
      series->pushBack({ timestamp, trans.transform.rotation.w });
    }
  }
};

using TfMsgParser = TfMsgParserImpl<tf::tfMessage>;
using Tf2MsgParser = TfMsgParserImpl<tf2_msgs::TFMessage>;
