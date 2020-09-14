Changelog
=========

0.7.12 (2015-07-09)
-------------------

0.7.11 (2015-05-27)
-------------------
* [rocon_app_manager_msgs] published interface types.
* Contributors: Daniel Stonier

0.7.10 (2015-04-06)
-------------------

0.7.9 (2015-02-09)
------------------

0.7.8 (2014-11-21)
------------------
* new error code for stopping a rapp that is not listed.
* Contributors: Daniel Stonier

0.7.6 (2014-08-25)
------------------
* Merge pull request `#97 <https://github.com/robotics-in-concert/rocon_msgs/issues/97>`_ from robotics-in-concert/multiple_impl
  add selected fieled to indicate which rapp virtual uses
* rename as preferred
* Merge pull request `#96 <https://github.com/robotics-in-concert/rocon_msgs/issues/96>`_ from robotics-in-concert/multiple_impl
* remove required_capabilities and add implementations in rapp.msg
* explanation for the rapp message.
* Export architecture_independent flag in package.xml
* add parameters in list rapp and start rapp
* rocon_app_manager_msgs: error: unconfigured build_depend on 'rocon_service_pair_msg
* Contributors: Daniel Stonier, Jihoon Lee, Scott K Logan

0.7.4 (2014-05-05)
------------------
* GetStatus removed in favour of a newly created Status latched publisher.
* App -> Rapp refactoring.
* Contributors: Daniel Stonier

0.7.0 (2014-03-29)
------------------
* major redesign focusing on the orchestration platform prototype.

0.6.3 (2013-10-30)
------------------
* rocon_std_msgs centralises some app manager messages for sharing.

0.6.2 (2013-09-11)
------------------
* report details of currently running app

0.6.1 (2013-08-30)
------------------
* simple invite service for relaying invitations by intelligent lurkers (e.g. pairing master).

0.6.0 (2013-08-07 18:58)
------------------------
* 0.5.2
* Merge pull request `#20 <https://github.com/robotics-in-concert/rocon_msgs/issues/20>`_ from robotics-in-concert/pairing
  Pairing
* PairingApp->PairingClient, more accurate terminology.
* 0.5.1
* added an icon to the platform info message.
* 0.5.0
* fix typo in constant assignation.
* multi-rapp error message.
* updated messages for pairing mode.
* for latched app list publisher
* backwards compatible app list and appdescription -> app changes.

0.5.3 (2013-08-07 19:04)
------------------------

0.5.2 (2013-07-17)
------------------
* Merge pull request `#20 <https://github.com/robotics-in-concert/rocon_msgs/issues/20>`_ from robotics-in-concert/pairing
  Pairing
* PairingApp->PairingClient, more accurate terminology.
* added an icon to the platform info message.
* fix typo in constant assignation.
* multi-rapp error message.
* updated messages for pairing mode.
* for latched app list publisher
* backwards compatible app list and appdescription -> app changes.

0.5.1 (2013-06-10 16:28:55 +0900)
---------------------------------
* 0.5.0
* Merge pull request `#18 <https://github.com/robotics-in-concert/rocon_msgs/issues/18>`_ from robotics-in-concert/hydro-devel
  back port from hydro to groovy
* proper dependency exporting for rocon msgs.

0.5.0 (2013-05-27)
------------------
* local_client_name->application_namespace
* Invite with client local name
* Invite with client local name
* updates for rapp manager's stop app functionality.
* more clear on what the namespace is here.
* 0.4.0

0.3.0 (2013-02-05)
------------------
* fix broken cmake for reshuffled messages.
* update jihoon email
* concert status -> app manager status, part of first redesign.
* status message brought back into app manager.
* remote_control -> invite, start on general app design
* rocon_app_manager messages now fully realised in rocon_app_manager_msgs.
* platform info constants -> platform info msg.

0.2.1 (2013-01-31)
------------------
* PlatformInfo msg GetPlatformInfo srv moved from concert to app manager. RemoteGatewayInfo srv typo
* platform info to rocon_app_manager_msgs
* refactoring app->rapp.

0.2.0 (2012-12-23 14:15:44)
---------------------------

0.1.4 (2012-12-23 14:15:23)
---------------------------

0.1.3 (2012-12-07)
------------------

0.1.2 (2012-11-22)
------------------

0.1.1 (2012-11-21)
------------------

0.1.0 (2012-03-29)
------------------
