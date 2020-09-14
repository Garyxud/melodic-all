#pragma once

#include "ros_type_introspection/ros_introspection.hpp"
#include "PlotJuggler/plotdata.h"

//----------------------------------

enum LargeArrayPolicy : bool
{
  DISCARD_LARGE_ARRAYS = true,
  KEEP_LARGE_ARRAYS = false
};

using SerializedMessage = RosIntrospection::Span<uint8_t>;

class MessageParserBase
{
public:
  MessageParserBase(const std::string& topic_name, PlotDataMapRef& plot_data)
    : _use_header_stamp(false), _topic_name(topic_name), _plot_data(plot_data)
  {
  }

  virtual ~MessageParserBase() = default;

  virtual void setUseHeaderStamp(bool use);

  virtual void setMaxArrayPolicy(LargeArrayPolicy policy, size_t max_size)
  {
  }

  virtual bool parseMessage(const SerializedMessage serialized_msg, double timestamp) = 0;

  static PlotData& getSeries(PlotDataMapRef& plot_data, const std::string key);

protected:
  bool _use_header_stamp;
  const std::string _topic_name;
  PlotDataMapRef& _plot_data;
};

template <typename T>
class BuiltinMessageParser : public MessageParserBase
{
public:
  BuiltinMessageParser(const std::string& topic_name, PlotDataMapRef& plot_data)
    : MessageParserBase(topic_name, plot_data)
  {
  }

  virtual bool parseMessage(SerializedMessage serialized_msg, double timestamp) override
  {
    T msg;
    ros::serialization::IStream is(const_cast<uint8_t*>(serialized_msg.data()), serialized_msg.size());
    ros::serialization::deserialize(is, msg);
    parseMessageImpl(msg, timestamp);
    return true;
  }

  virtual void parseMessageImpl(const T& msg, double timestamp) = 0;

protected:
};

class IntrospectionParser : public MessageParserBase
{
public:
  IntrospectionParser(const std::string& topic_name, const std::string& topic_type, const std::string& definition,
                      PlotDataMapRef& plot_data)
    : MessageParserBase(topic_name, plot_data), _max_size(999)
  {
    auto type = RosIntrospection::ROSType(topic_type);
    _parser.registerMessageDefinition(topic_name, type, definition);
  }

  void setMaxArrayPolicy(LargeArrayPolicy policy, size_t max_size) override;

  virtual bool parseMessage(SerializedMessage serialized_msg, double timestamp) override;

private:
  RosIntrospection::Parser _parser;
  RosIntrospection::FlatMessage _flat_msg;
  RosIntrospection::RenamedValues _renamed;
  size_t _max_size;
};

class CompositeParser
{
public:
  CompositeParser(PlotDataMapRef& plot_data);

  virtual void setUseHeaderStamp(bool use);

  virtual void setMaxArrayPolicy(LargeArrayPolicy policy, size_t max_size);

  void registerMessageType(const std::string& topic_name, const std::string& topic_type, const std::string& definition);

  bool parseMessage(const std::string& topic_name, SerializedMessage serialized_msg, double timestamp);

private:
  std::unordered_map<std::string, std::shared_ptr<MessageParserBase>> _parsers;

  LargeArrayPolicy _discard_policy;

  size_t _max_array_size;

  bool _use_header_stamp;

  PlotDataMapRef& _plot_data;
};
