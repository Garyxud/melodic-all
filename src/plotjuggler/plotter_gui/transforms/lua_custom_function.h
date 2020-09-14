#ifndef LUA_CUSTOM_FUNCTION_H
#define LUA_CUSTOM_FUNCTION_H

#include "custom_function.h"
#include "sol.hpp"

class LuaCustomFunction : public CustomFunction
{
public:
  LuaCustomFunction(const std::string& linkedPlot, const SnippetData& snippet);

  void initEngine() override;

  PlotData::Point calculatePoint(const PlotData& src_data, const std::vector<const PlotData*>& channels_data,
                                 size_t point_index) override;

  QString language() const override
  {
    return "LUA";
  }

private:
  std::unique_ptr<sol::state> _lua_engine;
  sol::function _lua_function;
  std::vector<double> _chan_values;
};

#endif  // LUA_CUSTOM_FUNCTION_H
