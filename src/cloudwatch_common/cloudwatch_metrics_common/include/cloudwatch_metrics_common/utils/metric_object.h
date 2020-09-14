/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <list>
#include <map>
#include <string>

#include <aws/monitoring/model/PutMetricDataRequest.h>
#include <aws/monitoring/model/StandardUnit.h>
#include <cloudwatch_metrics_common/definitions/definitions.h>

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

static std::unordered_map<std::string, Aws::CloudWatch::Model ::StandardUnit> units_mapper = {
  {"sec", Aws::CloudWatch::Model::StandardUnit::Seconds},
  {"msec", Aws::CloudWatch::Model::StandardUnit::Milliseconds},
  {"usec", Aws::CloudWatch::Model::StandardUnit::Microseconds},
  {"percent", Aws::CloudWatch::Model::StandardUnit::Percent},
  {"count", Aws::CloudWatch::Model::StandardUnit::Count},
  {"count_per_sec", Aws::CloudWatch::Model::StandardUnit::Count_Second},
  {"bytes", Aws::CloudWatch::Model::StandardUnit::Bytes},
  {"kilobytes", Aws::CloudWatch::Model::StandardUnit::Kilobytes},
  {"megabytes", Aws::CloudWatch::Model::StandardUnit::Megabytes},
  {"gigabytes", Aws::CloudWatch::Model::StandardUnit::Gigabytes},
  {"terabytes", Aws::CloudWatch::Model::StandardUnit::Terabytes},
  {"bytes_per_sec", Aws::CloudWatch::Model::StandardUnit::Bytes_Second},
  {"kilobytes_per_sec", Aws::CloudWatch::Model::StandardUnit::Kilobytes_Second},
  {"megabytes_per_sec", Aws::CloudWatch::Model::StandardUnit::Megabytes_Second},
  {"gigabytes_per_sec", Aws::CloudWatch::Model::StandardUnit::Gigabytes_Second},
  {"terabytes_per_sec", Aws::CloudWatch::Model::StandardUnit::Terabytes_Second},
  {"none", Aws::CloudWatch::Model::StandardUnit::None},
  {"", Aws::CloudWatch::Model::StandardUnit::None},
};

enum class StatisticValuesType
{
  MINIMUM,
  MAXIMUM,
  SUM,
  SAMPLE_COUNT
};

/**
 * Wrapper object for the AWS specific Aws::CloudWatch::Model::MetricDatum. This object is meant to be constructed from
 * userland provided metric data instead of using the AWS SDK specific object.
 */
struct MetricObject {
  /**
   * Construct a new instance of MetricObject that contains a scalar value.
   *
   * @param _name metric name
   * @param _value metric scalar value
   * @param _unit metric unit
   * @param _timestamp metric time stamp
   * @param _dimensions dimensions delineating extra details about the metric
   * @param _storage_resolution
   */
  MetricObject(
    const std::string & _name,
    const double _value,
    const std::string & _unit,
    const int64_t _timestamp,
    const std::map<std::string, std::string> & _dimensions,
    const int _storage_resolution)
    : metric_name(_name), unit(_unit), timestamp(_timestamp), value(_value),
      dimensions(_dimensions), storage_resolution(_storage_resolution) {}

  /**
   * Construct a new instance of MetricObject that contains a statistical values.
   *
   * @param _name metric name
   * @param _statistic_values statistical values of the metric
   * @param _unit metric unit
   * @param _timestamp metric time stamp
   * @param _dimensions dimensions delineating extra details about the metric
   * @param _storage_resolution
   */
  MetricObject(
    const std::string & _name,
    const std::map<StatisticValuesType, double> & _statistic_values,
    const std::string & _unit,
    const int64_t _timestamp,
    const std::map<std::string, std::string> & _dimensions,
    const int _storage_resolution)
    : metric_name(_name), unit(_unit), timestamp(_timestamp), statistic_values(_statistic_values),
      dimensions(_dimensions), storage_resolution(_storage_resolution) {}

  std::string metric_name;
  std::string unit;
  int64_t timestamp;
  double value{}; // mutually exclusive with statistic_values
  std::map<StatisticValuesType, double> statistic_values; // mutually exclusive with value
  std::map<std::string, std::string> dimensions;
  int storage_resolution;
};

/**
 * Helper method to constructor an Aws::CloudWatch::Model::MetricDatum from a MetricObject.
 *
 * @param metrics input MetricObject
 * @param timestamp
 * @return Aws::CloudWatch::Model::MetricDatum
 */
static MetricDatum metricObjectToDatum(const MetricObject &metrics, const int64_t timestamp) {

  MetricDatum datum;
  Aws::String aws_metric_name(metrics.metric_name.c_str());
  Aws::Utils::DateTime date_time(timestamp);

  datum.WithMetricName(aws_metric_name).WithTimestamp(date_time);

  if (metrics.statistic_values.empty()) {
    datum.SetValue(metrics.value);
  } else {
    Aws::CloudWatch::Model::StatisticSet stats;
    for (const auto & keyval : metrics.statistic_values) {
      if (keyval.first == StatisticValuesType::MINIMUM) {
        stats.SetMinimum(keyval.second);
      } else if (keyval.first == StatisticValuesType::MAXIMUM) {
        stats.SetMaximum(keyval.second);
      } else if (keyval.first == StatisticValuesType::SUM) {
        stats.SetSum(keyval.second);
      } else if (keyval.first == StatisticValuesType::SAMPLE_COUNT) {
        stats.SetSampleCount(keyval.second);
      }
    }
    datum.SetStatisticValues(stats);
  }

  auto mapped_unit = units_mapper.find(metrics.unit);
  if (units_mapper.end() != mapped_unit) {
    datum.SetUnit(mapped_unit->second);
  } else {
    Aws::String unit_name(metrics.unit.c_str());
    datum.SetUnit(Aws::CloudWatch::Model::StandardUnitMapper::GetStandardUnitForName(unit_name));
  }

  for (const auto & it : metrics.dimensions) {
    Aws::CloudWatch::Model::Dimension dimension;
    Aws::String name(it.first.c_str());
    Aws::String d_value(it.second.c_str());
    dimension.WithName(name).WithValue(d_value);
    datum.AddDimensions(dimension);
  }

  datum.SetStorageResolution(metrics.storage_resolution);

  return datum;
}

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
