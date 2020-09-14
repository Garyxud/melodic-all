#!/usr/bin/env python
# -*- python -*-
#
#  @file profile_gen.py
#  @brief RTC profile generator
#  @date $Date$
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2008
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
#

import gen_base

profile_yaml = """rtcProfile: 
  version: "1.0"
  id: [id]

  basicInfo:
    name: [basicInfo.name]

    description: [basicInfo.description]

    version: [basicInfo.version]

    vendor: [basicInfo.vendor]

    category: [basicInfo.category]

    componentType: [basicInfo.componentType]

    activityType: [basicInfo.activityType]

    componentKind: [basicInfo.componentKind]

    maxInstances: [basicInfo.maxInstances]

    abstract: [basicInfo.abstract]

    executionRate: [basicInfo.executionRate]

    executionType: [basicInfo.executionType]

    creationDate:
      year: [basicInfo.creationDate.year]

      month: [basicInfo.creationDate.month]

      day: [basicInfo.creationDate.day]

      hour: [basicInfo.creationDate.hour]

      minute: [basicInfo.creationDate.minute]

      second: [basicInfo.creationDate.second]

    updateDate:
      year: [basicInfo.updateDate.year]

      month: [basicInfo.updateDate.month]

      day: [basicInfo.updateDate.day]

      hour: [basicInfo.updateDate.hour]

      minute: [basicInfo.updateDate.minute]

      second: [basicInfo.updateDate.second]

    "rtcDoc::doc":
      algorithm: [basicInfo.rtcDoc::doc.algorithm]

      creator: [basicInfo.rtcDoc::doc.creator]

      description: [basicInfo.rtcDoc::doc.description]

      inout: [basicInfo.rtcDoc::doc.inout]

      license: [basicInfo.rtcDoc::doc.license]

      reference: [basicInfo.rtcDoc::doc.reference]

    "rtcExt::versionUpLog": 
[for log in basicInfo.rtcExt::versionUpLog]
      - [log]
[endfor]
  language: 
[if-any language.java]
    java: 
      library: 
[for javalib in language.java.library]
        - [javalib]
[endfor]
[endif]
  actions:  
    onInitialize:
      implemented: [actions.onInitialize.implemented]

      "rtcDoc::doc":
        description: [actions.onInitialize.rtcDoc::doc.description]

        postCondition: [actions.onInitialize.rtcDoc::doc.postCondition]

        preCondition: [actions.onInitialize.rtcDoc::doc.preCondition]

    onActivated:
      implemented: [actions.onActivated.implemented]

      "rtcDoc::doc":
        description: [actions.onActivated.rtcDoc::doc.description]

        postCondition: [actions.onActivated.rtcDoc::doc.postCondition]

        preCondition: [actions.onActivated.rtcDoc::doc.preCondition]

    onDeactivated:
      implemented: [actions.onDeactivated.implemented]

      "rtcDoc::doc":
        description: [actions.onDeactivated.rtcDoc::doc.description]

        postCondition: [actions.onDeactivated.rtcDoc::doc.postCondition]

        preCondition: [actions.onDeactivated.rtcDoc::doc.preCondition]

    onAborting:
      implemented: [actions.onAborting.implemented]

      "rtcDoc::doc":
        description: [actions.onAborting.rtcDoc::doc.description]

        postCondition: [actions.onAborting.rtcDoc::doc.postCondition]

        preCondition: [actions.onAborting.rtcDoc::doc.preCondition]

    onError:
      implemented: [actions.onError.implemented]

      "rtcDoc::doc":
        description: [actions.onError.rtcDoc::doc.description]

        postCondition: [actions.onError.rtcDoc::doc.postCondition]

        preCondition: [actions.onError.rtcDoc::doc.preCondition]

    onReset:
      implemented: [actions.onReset.implemented]

      "rtcDoc::doc":
        description: [actions.onReset.rtcDoc::doc.description]

        postCondition: [actions.onReset.rtcDoc::doc.postCondition]

        preCondition: [actions.onReset.rtcDoc::doc.preCondition]

    onFinalize:
      implemented: [actions.onFinalize.implemented]

      "rtcDoc::doc":
        description: [actions.onFinalize.rtcDoc::doc.description]

        postCondition: [actions.onFinalize.rtcDoc::doc.postCondition]

        preCondition: [actions.onFinalize.rtcDoc::doc.preCondition]

    onStartup:
      implemented: [actions.onStartup.implemented]

      "rtcDoc::doc":
        description: [actions.onStartup.rtcDoc::doc.description]

        postCondition: [actions.onStartup.rtcDoc::doc.postCondition]

        preCondition: [actions.onStartup.rtcDoc::doc.preCondition]

    onRateChanged:
      implemented: [actions.onRateChanged.implemented]

      "rtcDoc::doc":
        description: [actions.onRateChanged.rtcDoc::doc.description]

        postCondition: [actions.onRateChanged.rtcDoc::doc.postCondition]

        preCondition: [actions.onRateChanged.rtcDoc::doc.preCondition]

    onShutdown:
      implemented: [actions.onShutdown.implemented]

      "rtcDoc::doc":
        description: [actions.onShutdown.rtcDoc::doc.description]

        postCondition: [actions.onShutdown.rtcDoc::doc.postCondition]

        preCondition: [actions.onShutdown.rtcDoc::doc.preCondition]

    onExecute:
      implemented: [actions.onExecute.implemented]

      "rtcDoc::doc":
        description: [actions.onExecute.rtcDoc::doc.description]

        postCondition: [actions.onExecute.rtcDoc::doc.postCondition]

        preCondition: [actions.onExecute.rtcDoc::doc.preCondition]

    onStateUpdate:
      implemented: [actions.onStateUpdate.implemented]

      "rtcDoc::doc":
        description: [actions.onStateUpdate.rtcDoc::doc.description]

        postCondition: [actions.onStateUpdate.rtcDoc::doc.postCondition]

        preCondition: [actions.onStateUpdate.rtcDoc::doc.preCondition]

  dataPorts: 
[for dport in dataPorts]
    -
      portType: [dport.portType]

      name: [dport.name]

      type: [dport.type]

      interfaceType: [dport.interfaceType]

      dataflowType: [dport.dataflowType]

      subscriptionType: [dport.subscriptionType]

      idlFile: [dport.idlFile]

      "rtcDoc::doc":
        type: [dport.rtcDoc::doc.type]

        description: [dport.rtcDoc::doc.description]

        number: [dport.rtcDoc::doc.number]

        occerrence: [dport.rtcDoc::doc.occerrence]

        operation: [dport.rtcDoc::doc.operation]

        semantics: [dport.rtcDoc::doc.semantics]

        unit: [dport.rtcDoc::doc.unit]

      "rtcExt::position": [dport.rtcExt::position]

      "rtcExt::varname": [dport.rtcExt::varname]

[endfor]
  servicePorts: 
[for sport in servicePorts]
    -
      name: [sport.name]

      "rtcDoc::doc":
        description: [sport.rtcDoc::doc.description]

        ifdescription: [sport.rtcDoc::doc.ifdescription]

      "rtcExt::position": [sport.rtcExt::position]

      serviceInterface: 
[for sif in sport.serviceInterface]
        -
          direction: [sif.direction]

          name: [sif.name]

          type: [sif.type]

          varname: [sif.varname]

          instanceName: [sif.instanceName]

          idlFile: [sif.idlFile]

          path: [sif.path]

          "rtcDoc::doc":
            description: [sif.rtcDoc::doc.description]

            docArgument: [sif.rtcDoc::doc.docArgument]

            docException: [sif.rtcDoc::doc.docException]

            docPostCondition: [sif.rtcDoc::doc.docPostCondition]

            docPreCondition: [sif.rtcDoc::doc.docPreCondition]

            docReturn: [sif.rtcDoc::doc.docReturn]

[endfor]
[endfor]
  configurationSet: 
    configuration: 
[for conf in configurationSet.configuration]
      - 
        name: [conf.name]

        type: [conf.type]

        varname: [conf.varname]

        defaultValue: [conf.defaultValue]

        "rtcDoc::doc":
          constraint: [conf.rtcDoc::doc.constraint]

          dataname: [conf.rtcDoc::doc.dataname]

          defaultValue: [conf.rtcDoc::doc.defaultValue]

          description: [conf.rtcDoc::doc.description]

          range: [conf.rtcDoc::doc.range]

          unit: [conf.rtcDoc::doc.unit]

[endfor]
  parameters: 
[if-any parameters]
[for param in parameters]
    -
      name: [param.name]

      defaultValue: [param.defaultValue]

[endfor]
[endif]
"""


class profile_gen(gen_base.gen_base):
    def __init__(self, data):
        self.data = data
        self.data["fname"] = data["basicInfo"]["name"] + ".yaml"
        self.tags = {}

    def print_yamlprofile(self):
        self.gen(self.data["fname"], profile_yaml, self.data, self.tags)

    def print_all(self):
        self.print_yamlprofile()
