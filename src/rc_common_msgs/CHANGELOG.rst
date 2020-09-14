0.5.0 (2020-04-03)
------------------

* Added array key/value list for extra data to CameraParam message and removed 'noise' field (will be provided in extra_data)

0.4.1 (2020-03-06)
------------------

* add CAPACITY_EXCEEDED and CAPACITY_REACHED to ReturnCodeConstants

0.4.0 (2019-12-04)
------------------

* Renamed CameraParams to CameraParam for consistency with internal message

0.3.0 (2019-09-11)
------------------

* add CameraParam msg

0.2.1 (2019-04-26)
------------------

* add KeyValue msg

0.2.0 (2019-03-14)
------------------

* add INTERNAL_ERROR and IO_ERROR as constants to return codes
* improve constants description

0.1.0 (2019-03-11)
------------------

* add ReturnCode with separate constants definitions
* add Trigger service using ReturnCode
* Initial release
