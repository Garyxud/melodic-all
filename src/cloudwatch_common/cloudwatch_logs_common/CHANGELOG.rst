^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_logs_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2019-03-20)
------------------
* adding unit tests for cloudwatch facade
* Merge pull request `#4 <https://github.com/aws-robotics/cloudwatch-common/issues/4>`_ from juanrh/improve-coverage-cloudwatch_logger
  Improve coverage cloudwatch logger
* Make LogManagerFactory mockeable
* Make cloudwatch_logs_common shared lib to use it in other libs
* Merge pull request `#1 <https://github.com/aws-robotics/cloudwatch-common/issues/1>`_ from xabxx/master
  [Bug Fix] Resolved false-positive error log messages
* Resolved false-positive error log messages
* Contributors: Abby Xu, Ross Desmond, Ryan Newell, Yuan "Forrest" Yu, hortala

1.1.3 (2020-03-27)
------------------
* Bumping version to match bloom release (`#51 <https://github.com/aws-robotics/cloudwatch-common/issues/51>`_)
  Bumping version to 1.1.3
* Fix linting issues found by clang-tidy 6.0 (`#50 <https://github.com/aws-robotics/cloudwatch-common/issues/50>`_)
  * clang-tidy fixes
  * revert explicit constructor declarations to maintain API compatbility
  * clang-tidy linting issues fixed manually
  * fix unit tests build break
* Increase package version numbers to 1.1.2 (`#44 <https://github.com/aws-robotics/cloudwatch-common/issues/44>`_)
* Contributors: Miaofei Mei, Nick Burek, Ragha Prasad

1.1.1 (2019-09-10)
------------------
* Disable error on cast-align warning to support ARMhf builds (`#41 <https://github.com/aws-robotics/cloudwatch-common/issues/41>`_)
  * Remove -Wcast-align flag to support ARMhf builds
  *  - bumped versions to 1.1.1
  - restored cast-align, but added as a warning
  - removed unused headers
  * Removed duplicate Werror
* modify changelist to be compatible with catkin_generate_changelog (`#39 <https://github.com/aws-robotics/cloudwatch-common/issues/39>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Merge pull request `#36 <https://github.com/aws-robotics/cloudwatch-common/issues/36>`_ from aws-robotics/guard_test_libs
  Added guards for test libraries
* Added guards for test libraries
* Offline logs feature (`#34 <https://github.com/aws-robotics/cloudwatch-common/issues/34>`_)
  * Add file manager and upload complete callback
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fixed merge conflict
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add file manager test
  Still deciding if the write should just go in the FileManagerStrategy.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add FileManager to log manager factory
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add generic file upload logic
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add reading, discovery and rotation to FileManagerStrategy
  - Add function getFileToRead that returns the best file to start reading
  from when sending data from the file to CloudWatch.
  - Add function discoverStoredFiles which finds all files in the
  storage directory and keeps them, in case there were offline logs still
  hanging around from a previous run.
  - Add rotateActiveFile which creates a new file to write to. This is
  used when we want to read from the current active file, or when the file
  size limit has been reached.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add file management system.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Link file management with publisher
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add test for file management system
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add reading and writing to FileManagerStrategy
  - This way the file manager strategy can handle all the core file data
  stuff, keeping track of the size of the active file and rotating it when
  neccessary.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * cleanup
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Don't initialize file_manager_strategy yet
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add file upload statistics
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Move file management files to isolated package
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Moved tests to file_manager folder
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Update offline test location
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add comments to file management interfaces
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add more comments to file uploading
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add futures/promises to file upload tasks
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add blocking and observed queue interface
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add comments to observed_queue.h
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Make batch size configurable for file upload manager
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add subscribe method to file upload manager
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Move queues and status monitor to dataflow folder
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Atomatize status monitor
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add working dataflow pipeline
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add template argument for file management factory
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add priority between queues functionality
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix test and rename namespace for dataflow
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Cleanup test
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Organize priority options
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Rename FileUploadManager to FileUploadStreamer
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Convert LogManager to sink and ILogManager
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Amend UploadFunction status type
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add milliseconds to dequeue
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add timeout to dequeue()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add operator >> overload for source to input stage
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Adding tests for file manager strategy
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Use Tokens with FileManagerStrategy
  - FileManagerStrategy now only has basic read and write functions. When
  reading data you're given a token. When done sending this to CloudWatch
  you can call resolve() on the token which marks that section of the file
  as complete. After all sections of a file have uploaded the file is
  deleted.
  - FileManagerStrategy also now inherits from DataManagerStrategy so that
  in the future people can build new systems for managing their offline
  logs and easily hook them into the existing framework.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix operator overload for InputStage
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Remove unnecessary external class declaration
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add tests for file upload streamer
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Reformat file management
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Reformat log file names and discovery
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Cleanup, add Tests
  - Rename file_name to file_path to better describe what it represents
  - Add more tests for FileManagerStrategy
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Delete old files when storage limit is reached
  - Add a storage_limit option. When this is reached the oldest file on
  disk will be deleted to clear space.
  - The storage limit is checked before data is written so that the size
  of files on disk can never be over the storage limit.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Task factory (`#6 <https://github.com/aws-robotics/cloudwatch-common/issues/6>`_)
  * Don't use atomic memory operations
  * generic observer definition
  * init connection monitor
  * Added publisher interface
  Refactored log publisher to use interface
  * Added IPublisher Interface definition and as Task member
  Added Publisher implementation and templated for data type
  * Added LogBatcher (renamed LogManager)
  Added DataStreamer interface
  * Rename DataStreamer interface to DataBatcher to reduce confusion
  * Added TaskFactory
  Added Publisher Interface
  Added factory to LogBatcher
  * BasicTask uses a shared pointer for templated data
  * Added publishing when the DataBatcher reaches a specific size
  * Use generic FileManager in the TaskFactory
  Add uploadCompleteStatus as an abstract FileManager method
  * Added Log Service
  - worker thread to dequeue tasks
  - common owner of various actors (file streaming, log batching)
  - templated to  be a generic interface
  * Added Task cancel method
  * minor change to push
  * Fix file permissions
  * Added dequeue with timeout for LogService
  * Finished publisher base class implementation
  Added new states for LogPublisher
  * Removed shared object from logs
  * Moved AWS SDK init and shutdown into publisher
  * uncomment file streamer code - does not compile!
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fixed build, still todo fix file streamer
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Simple pipeline tests (`#9 <https://github.com/aws-robotics/cloudwatch-common/issues/9>`_)
  * initial commit for simple test
  * Added test for batched size
  * minor comments
  * Added ObservableObject class
  Added simple ObservableObject tests
  Integrated ObservableObject into base publisher class
  File Streamer uses ObservableObject registration on publisher owned
  state
  * Added basic service interface for generic init, start, and shutdown
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Remove task factory
  * Task change proposal
  * Remove the task factory
  * Working file management tests
  * Fix tests and minor logic due to merge
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Address comments for task change proposal
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Delete task_factory.h
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add cloudwatch options
  * Use explict set/add sink and source functions
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Cleanups (`#12 <https://github.com/aws-robotics/cloudwatch-common/issues/12>`_)
  * Added cancel flag
  * Added thread handling (same as log service) to file upload streamer
  * Added RunnableService
  * Added ObservableObject tests
  Added documentation
  * Added RunnableService test
  Added documentation
  Converted streamer and log service to runnables
  * Merge Conflict Fixes
  Added sanity (empty) tests
  * Fixed pipeline tests
  * Addressed review comments
  * fix for merge conflict
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Thorough testing of token system
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Clear file streamer queue on failure to upload
  * Add locks around dequeue
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add basic mutex synchronization for ObservedQueue
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Remove uploadStatusComplete from FileManager
  Remove the uploadStatusComplete function from FileManager as it is not the responsibility of the file manager to determine if data should be written. Instead, a lambda should be used to first check for upload failure then write to the file manager.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add construct from backup for TokenStore
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix synchronized queue and address comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Enable build flags (`#16 <https://github.com/aws-robotics/cloudwatch-common/issues/16>`_)
  * Added build flags per team process
  * Addressed some build fixes found by flags
  * Fix build issues with new build flags
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix publishing (`#15 <https://github.com/aws-robotics/cloudwatch-common/issues/15>`_)
  * Removed initialize method (not needed) for service
  Fixed publishing
  Reinit AWS SDK each time we configure (needed if gone offline)
  * Addressed some ToDos
  Added publisher diagnostics
  Minor cleanups
  Added documentation
  * Fix issue with constant
  * Propgated no network connection state in publisher
  * fix pipeline test teardown
  * Addressed review comments
  * merge fixes
  * Added input checking for CloudWatchService
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Don't clear sink on successful upload
  - Add test and fix bug so that the file upload sink is only cleared when
  an upload fails.
  cr https://code.amazon.com/reviews/CR-9559033
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2000: [Test] Full pipeline when there is no internet
  - added input checking for various constructors
  cr https://code.amazon.com/reviews/CR-9552279
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2136: Address migrating core classes to service interface
  - Define Defaults for File Strategy
  - Deleted files are deleted on a new thread
  - Removed code from destructors that may fail
  - CloudWatchService handles start / shutdown of all services
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2001: [Test] Full pipeline when there is intermittent internet
  ROS-2002: [Test] Case when batched data is queued at an untenable rate
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Addressed review comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Move dataflow to separate library
  cr https://code.amazon.com/reviews/CR-9586163
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Move file management to separate package directory
  * Modified onPublishStatusChanged in file streamer to remove dependency on cloudwatch
  cr https://code.amazon.com/reviews/CR-9596692
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2147: Move DataBatcher to utils
  cr https://code.amazon.com/reviews/CR-9640987
  Signed-off-by: Miaofei <miaofei@amazon.com>
  *  - addressed review comments
  - added documentation
  - moved waiter test utility to separate implementation
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2166: I can check the state of the CloudWatch publishing service
  cr https://code.amazon.com/reviews/CR-9763677
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add Metric File Manager to Cloudwatch Metrics Common
  cr https://code.amazon.com/reviews/CR-9607921
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Improve metric serialization, add tests.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add Serialization of StatisticValues
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add serializing of Dimensions, Value and Values
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Doc and coding style improvements
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Squashed commit of the following:
  cr https://code.amazon.com/reviews/CR-9769267
  commit 41bc857bd30853f80a439bfec0ba389fd4253dc0
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Fri Jun 21 13:52:29 2019 -0700
  Various fixes from rebasing
  commit 0e6149b0733323d80390567c11d65e013318d3f2
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Thu Jun 20 16:39:58 2019 -0700
  - addressed review comments
  - added metrics definition file
  - removed configure from publisher interface
  commit 206880df0198d6fba4299f0ebd25fbc23831bc8b
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Mon Jun 17 11:43:57 2019 -0700
  ROS-2055: Implement DataBatcher for Metrics
  ROS-2056: Implement MetricService
  cr https://code.amazon.com/reviews/CR-9769267
  commit c2ad314521b17a34c7f481d5ea5c5ca008918ac2
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Fri Jun 14 23:55:23 2019 -0700
  ROS-2057: Create immutable metric container
  commit b2df9419963a67b60b87df9e5aee34d55111d92c
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Fri Jun 14 16:50:48 2019 -0700
  Moved CloudwatchService to utils
  commit e07e35e04ed3a4c2d2803a3daf3261dc8f3c2e4b
  Author: Devin Bonnie <dbbonnie@amazon.com>
  Date:   Fri Jun 14 11:08:40 2019 -0700
  ROS-2055: Implement Metric Publisher
  - moved Publisher to utilities
  - moved CloudWatchService to utilities
  - cleaned up headers
  - fixed namespace issues
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2226: [Bug] Metrics Facade Class does not properly set network disconnected state
  cr https://code.amazon.com/reviews/CR-10089409
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Backup TokenStore to disk
  - Add TokenStoreOptions so the user can configure the directory the token store is backed up to.
  - On shutdown save the token store and all active tokens out to disk in
  JSON format.
  - On startup load the tokenstore from the file saved on disk.
  - Tests for shutdown/startup
  cr https://code.amazon.com/reviews/CR-9736297
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Improve serialize function, catch invalid JSON
  - Add a new serialize function instead of overloading << in TokenStore
  - Catch and continue if we have trouble parsing the TokenStore backup
  file.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Improve naming and initialization of variables
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add better random number generator
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Code style fixes
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2051: Add FileManagement Pipeline to CW Metrics
  cr https://code.amazon.com/reviews/CR-10100452
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Moving options around
  - Moving TokenStore and FileManagerStrategy options to a separate file
  so that it can be included and set by the upstream packages.
  - Renaming the Dataflow options to UploaderOptions
  - Creating one main CloudwatchOptions in both logs and metrics that has FileManagerOptions and
  UploaderOptions inside it.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Change storage limits to kb instead of bytes
  cr https://code.amazon.com/reviews/CR-10144739
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * File upload streamer integration and unit tested
  *Summary*
  File upload and token cache manages failed and in flight tokens. Files are uploaded when the streamer is notified of an available file and network access.
  Files that are on the system are after FileStreamer shutdown are uploaded on restart.
  * Tested with cloudwatch logs
  * Tested with unit tests
  cr https://code.amazon.com/reviews/CR-10173529
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Capitalize W in kDefaultCloudWatchOptions
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Pass options correctly, fixing bugs
  - Pass options to the FileManager for logs and metrics
  - Add additional params to handle this option passing.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add different file storage options for metrics by default
  - Metrics files now go in a metrics directory with metric prefix by
  default, so that they don't get mixed up with offline logs.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * DRY'ify, remove magic numbers, fix tests
  - Consolidate duplicate path processing code into one area.
  - Fix magic numbers, move into defines.
  - Fix tests.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2249: [Bug] Log Publisher implementation does not properly handle token init
  ROS-2250: Restore CloudWatch Logs Facade Unit Test
  cr https://code.amazon.com/reviews/CR-10253526
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Added relevant unit tests
  Minor fixes and cleanup
  Signed-off-by: Miaofei <miaofei@amazon.com>
  *  - CloudWatchClients are now shared pointers instead of unique
  - addressed spacing issues
  - updated CloudWatchLogs facade naming to be consistent with Metrics
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Include <random> in header file
  cr https://code.amazon.com/reviews/CR-10531916
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Rename variables and error to match config
  - Rename the batch size variables to match the config file names.
  - Update error message so the end user knows what config options are
  wrong.
  cr https://code.amazon.com/reviews/CR-10481115
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Allow batch_trigger_publish_size and batch_max_queue_size to be the same
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Changing back ot publish size must be less than max queue size
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Check batch trigger publish size against kDefaultTriggerSize
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2231: [Bug] Potential locking issue with DataBatcher child classes
  - batcher attempt to flush batched data when shutting down
  - added documentation
  cr https://code.amazon.com/reviews/CR-10543019
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Addressed review comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix up param values
  - Remove stream_max_queue_size as it's no longer used.
  - Remove kDefaultUploaderOptions because it's not used as it's always
  replaced by the default values specified in uploader_options struct.
  - Pass batch_max_queue_size and batch_trigger_publish_size to the
  DataBatcher's so they're actually used
  cr https://code.amazon.com/reviews/CR-10571067
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2338: I can configure the amount of streamed data to hold in memory
  cr https://code.amazon.com/reviews/CR-10578133
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2240: Restore existing unit tests
  - added definitions header to logs
  cr https://code.amazon.com/reviews/CR-10569452
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Removed extra definitions file
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2341: Publisher state refactor
  cr https://code.amazon.com/reviews/CR-10584550
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fixes bug with trying to upload to cloudwatch in batches that aren't chronologically sorted. https://sim.amazon.com/issues/7cbe72f2-28c6-4771-a202-ab0d72587031
  cr https://code.amazon.com/reviews/CR-10621402
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2346: [Bug] Don't set stats values in metric datums
  cr https://code.amazon.com/reviews/CR-10623123
  Signed-off-by: Miaofei <miaofei@amazon.com>
  *  - doc additions
  Signed-off-by: Miaofei <miaofei@amazon.com>
  *  - removed other unsupported types via review
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2263: [Bug] Storage and retry behavior for failed requests
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Addressed review comments
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Added invalid data handling to metrics
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2368: [Bug] Data is not attempted to be uploaded without an active input
  cr https://code.amazon.com/reviews/CR-10664962
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2369: [Bug] Fix Metrics Serialization Unit Tests
  cr https://code.amazon.com/reviews/CR-10665643
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Revert "ROS-2368: [Bug] Data is not attempted to be uploaded without an active input"
  This reverts commit 67129f977446079a28539833c0d3d7967306f0c2.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2368: [Bug] Data is not attempted to be uploaded without an active input
  cr https://code.amazon.com/reviews/CR-10664962
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2380: [Bug] CloudWatch Service Shutdown
  cr https://code.amazon.com/reviews/CR-10804863
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Fix bug - logs not being uploaded from disk after reconnecting
  - If all files on disk were added to the queue the status was set to
  UNAVAILABLE. Then if they failed to upload the status was never
  restored. This ensures that if a file fails to upload the status is set
  back to AVAILABLE so they can attempt to be uploaded again.
  - Add more DEBUG logs to file management.
  cr https://code.amazon.com/reviews/CR-10806493
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Read the newest file in storage instead of the oldest, lock when
  deleting file
  - Read the newest file from storage instead of reading the oldest.
  - When deleting a file to free up storage space, add a lock to ensure
  we're not reading from that same file. If we are then stop reading from
  that file.
  cr https://code.amazon.com/reviews/CR-10886255
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add lock to active write file
  - When checking if the active file should be rotated first lock it to ensure it's not being written to as it's rotated.
  - Add new log to delete oldest file.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Add docs for FileManagerStrategy, cleanup unused code
  - Add documentation to all FileManagerStrategy functions
  - Remove some un-useful code for the FileManagerStrategy
  - Function renaming / cleanup to make more sense.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Remove todo and unused variable
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Remove unneccessary initialization and commented out code
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2381: [Bug] Items in memory lost on shutdown
  cr https://code.amazon.com/reviews/CR-10942302
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * ROS-2421: [Bug] Ensure FileManager thrown exceptions are handled
  cr https://code.amazon.com/reviews/CR-11029944
  Signed-off-by: Miaofei <miaofei@amazon.com>
  *  - addressed review comments
  - changed file upload streamer wait timeout from 1 minute to 5 minutes
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Addressed terse variable names
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * increment minor version
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * fix compilation errors in unit tests
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * fix more compilation errors found in dashing
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * fix unit test failures
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Modify cloudwatch common to depend on gtest, gmock (`#19 <https://github.com/aws-robotics/cloudwatch-common/issues/19>`_)
  * Modify cloudwatch common to depend on gtest, gmock
  Use the macro in aws_common to find test dependencies for ROS1 or ROS2.
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * more CMakeLists.txt cleanup
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml test matrix
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update PACKAGE_NAMES
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Adding old release to change log
* Update package.xml for 1.0.2 release
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Increase test timeout to fix flakiness (`#21 <https://github.com/aws-robotics/cloudwatch-common/issues/21>`_)
  - Increase the test timeout to 5000ms because sometimes
  SendLogsToCloudWatch isn't being called fast enough and so the tests
  fail.
* Fixes issue with DescribeLogStreams being called to get the next token
  - Call DescribeLogStreams only one on startup
  - Implements a state machine to manage the core run loop instead of checking if the log stream exists every tick.
* Update success logs' severity to DEBUG in cloudwatch_facade.cpp
* Release 1.0.1 (`#14 <https://github.com/aws-robotics/cloudwatch-common/issues/14>`_)
  * Release 1.0.1
  * 1.0.1
* adding unit tests for cloudwatch facade
* Merge pull request `#4 <https://github.com/aws-robotics/cloudwatch-common/issues/4>`_ from juanrh/improve-coverage-cloudwatch_logger
  Improve coverage cloudwatch logger
* Make LogManagerFactory mockeable
* Make cloudwatch_logs_common shared lib to use it in other libs
* Merge pull request `#1 <https://github.com/aws-robotics/cloudwatch-common/issues/1>`_ from xabxx/master
  [Bug Fix] Resolved false-positive error log messages
* Resolved false-positive error log messages
* Contributors: AAlon, Abby Xu, Devin Bonnie, M. M, Nick Burek, Ross Desmond, Ryan Newell, Tim Robinson, Yuan "Forrest" Yu, hortala, ryanewel

1.0.0 (2019-03-20)
------------------
