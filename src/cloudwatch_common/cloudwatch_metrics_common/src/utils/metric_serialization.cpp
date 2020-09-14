/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <cloudwatch_metrics_common/utils/metric_serialization.hpp>
#include <aws/core/utils/Array.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws/monitoring/model/MetricDatum.h>
#include <aws/monitoring/model/StandardUnit.h>
#include <iterator>
#include <string>
#include <cloudwatch_metrics_common/definitions/definitions.h>

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

using JsonValue = Aws::Utils::Json::JsonValue;

static constexpr const char* kTimestampKey = "timestamp";
static constexpr const char* kMetricNameKey = "metric_name";
static constexpr const char* kDimensionsKey = "dimensions";
static constexpr const char* kDimensionsNameKey = "name";
static constexpr const char* kDimensionsValueKey = "value";
static constexpr const char* kValueKey = "value";
static constexpr const char* kStorageResolutionKey = "storage_resolution";
static constexpr const char* kUnitKey = "unit";

static const std::vector<Aws::String> required_properties = {
    kMetricNameKey,
    kTimestampKey
};

/**
 * Take a JSON string and turn it into a MetricDatum object.
 * @throws invalid_argument if the JSON is invalid or is missing required parameters.
 * @param basic_string - a reference to a JSON string. This should be a single object.
 * @return datum - a MetricDatum created from this string.
 */
MetricDatum deserializeMetricDatum(const Aws::String &basic_string) {
  Aws::String aws_str(basic_string.c_str());
  JsonValue json_value(aws_str);
  if (!json_value.WasParseSuccessful()) {
    throw std::invalid_argument("Failed to parse metric JSON string");
  }
  auto view = json_value.View();
  for (const auto& property : required_properties) {
    if (!view.KeyExists(property)) {
      std::string property_name(property.c_str());
      throw std::invalid_argument("Could not find required property " + property_name + " in JSON string");
    }
  }

  MetricDatum datum;
  datum.SetMetricName(view.GetString(kMetricNameKey));
  datum.SetTimestamp(view.GetInt64(kTimestampKey));

  if (view.KeyExists(kDimensionsKey)) {
    auto array = view.GetArray(kDimensionsKey) ;
    Aws::Vector<Aws::CloudWatch::Model::Dimension> dimensions(array.GetLength());
    for (size_t i = 0; i < array.GetLength(); ++i) {
      Aws::CloudWatch::Model::Dimension dimension;
      dimension.SetName(array[i].GetString(kDimensionsNameKey));
      dimension.SetValue(array[i].GetString(kDimensionsValueKey));
      dimensions[i] = dimension;
    }
    datum.SetDimensions(dimensions);
  }

  if (view.KeyExists(kStorageResolutionKey)) {
    datum.SetStorageResolution(view.GetInteger(kStorageResolutionKey));
  }
  if (view.KeyExists(kUnitKey)) {
    datum.SetUnit(Aws::CloudWatch::Model::StandardUnit(view.GetInteger(kUnitKey)));
  }
  if (view.KeyExists(kValueKey)) {
    datum.SetValue(view.GetDouble(kValueKey));
  }

  return datum;
}

/**
 * Take a MetricDatum object and turn it into a JSON string
 * @param datum - A single MetricDatum object
 * @return json_string - a JSON representation of the passed in MetricDatum object.
 */
Aws::String serializeMetricDatum(const MetricDatum &datum) {
  Aws::Utils::Json::JsonValue json_value;

  const Aws::Vector<Aws::CloudWatch::Model::Dimension>& dimensions = datum.GetDimensions();
  Aws::Utils::Array<JsonValue> dimensions_array = Aws::Utils::Array<JsonValue>(dimensions.size());
  for (size_t i = 0; i < dimensions.size(); ++i) {
    JsonValue dimension_json;
    dimensions_array[i] = dimension_json
        .WithString(kDimensionsNameKey, dimensions[i].GetName())
        .WithString(kDimensionsValueKey, dimensions[i].GetValue());
  }

  const Aws::Vector<double>& values = datum.GetValues();
  Aws::Utils::Array<JsonValue> values_array = Aws::Utils::Array<JsonValue>(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    JsonValue val;
    values_array[i] = val.AsDouble(values[i]);
  }

  json_value
    .WithInt64(kTimestampKey, datum.GetTimestamp().Millis())
    .WithString(kMetricNameKey, datum.GetMetricName())
    .WithInteger(kStorageResolutionKey, datum.GetStorageResolution())
    .WithInteger(kUnitKey, static_cast<int>(datum.GetUnit()))
    .WithArray(kDimensionsKey, dimensions_array)
    .WithDouble(kValueKey, datum.GetValue());
  return json_value.View().WriteCompact();
}

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws