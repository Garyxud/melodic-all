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

#pragma once

#include <aws/core/Aws.h>
#include <aws/monitoring/model/PutMetricDataRequest.h>

#include <file_management/file_upload/file_upload_streamer.h>
#include <file_management/file_upload/file_manager.h>

#include <dataflow_lite/utils/data_batcher.h>

#include <cloudwatch_metrics_common/definitions/definitions.h>

#include <chrono>
#include <list>
#include <memory>

namespace Aws {
namespace CloudWatchMetrics {

class MetricBatcher :
        public DataBatcher<MetricDatum>,
        public Aws::DataFlow::OutputStage<Aws::FileManagement::TaskPtr<MetricDatumCollection>>
{
public:

    /**
     *  @brief Creates a new MetricBatcher
     *  Creates a new MetricBatcher that will group/buffer metrics. Note: metrics are only automatically published if the
     *  size is set, otherwise the publishBatchedData is necessary to push data to be published.
     *
     *  @throws invalid argument if publish_trigger_size is strictly greater than max_allowable_batch_size
     *  @param size of the batched data that will trigger a publish
     */
    explicit MetricBatcher(size_t max_allowable_batch_size = DataBatcher::kDefaultMaxBatchSize,
                           size_t publish_trigger_size = DataBatcher::kDefaultTriggerSize);

    MetricBatcher(const MetricBatcher & other) = delete;

    MetricBatcher & operator=(const MetricBatcher & other) = delete;

    /**
     *  @brief Tears down a MetricBatcher object
     */
    ~MetricBatcher() override;

    /**
     * Queue the batched data to be given to the publisher and sent to CloudWatch. Attempts to write to disk (through
     * the FileManager) if unable to publish.
     *
     * @return true if the batched data could be queued, false otherwise
     */
    bool publishBatchedData() override;
    /**
     * Override default behavior to attempt to write to file to disk when emptying the collection.
     */
    void emptyCollection() override;
    /**
     * Start this service
     * @return
     */
    bool start() override;

    /**
     * Set the log file manager, used for task publishing failures (write to disk if unable to send to CloudWatch).
     *
     * @throws invalid argument if the input is null
     * @param log_file_manager
     */
    virtual void setMetricFileManager(std::shared_ptr<Aws::FileManagement::FileManager<MetricDatumCollection>> file_manager);

private:
    std::shared_ptr<Aws::FileManagement::FileManager<MetricDatumCollection>> metric_file_manager_;
};

}  // namespace CloudWatchMetrics
}  // namespace Aws