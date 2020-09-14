#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file SdoServiceConsumerBase.py
# @brief SDO service consumer base class and its factory
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2011
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import SDOPackage
import OpenRTM_aist


##
# @if jp
#
# SdoServiceConsumerFactory&
#                     factory(SdoServiceConsumerFactory.instance());
#
# factory.addFactory(toRepositoryId<IDL Type>(),
#                   Creator< SdoServiceConsumerBase,
#                            your_sdo_service_consumer_subclass>,
#                   Destructor< SdoServiceConsumerBase,
#                            your_sdo_service_consumer_subclass>);
#
# @else
#
#
#
# @endif
class SdoServiceConsumerBase:
  """
  """

  def __init__(self):
    pass

  def __del__(self):
    pass

  # virtual bool init(RTObject_impl& rtobj,
  #                   const SDOPackage::ServiceProfile& profile) = 0;
  def init(self, rtobj, profile):
    pass


  # virtual bool reinit(const SDOPackage::ServiceProfile& profile) = 0;
  def reinit(self, profile):
    pass


  # virtual const SDOPackage::ServiceProfile& getProfile() const = 0;
  def getProfile(self):
    pass

  # virtual void finalize() = 0;
  def finalize(self):
    pass

sdoserviceconsumerfactory = None

class SdoServiceConsumerFactory(OpenRTM_aist.Factory,SdoServiceConsumerBase):
  def __init__(self):
    OpenRTM_aist.Factory.__init__(self)
    return

  def __del__(self):
    pass

  def instance():
    global sdoserviceconsumerfactory

    if sdoserviceconsumerfactory is None:
      sdoserviceconsumerfactory = SdoServiceConsumerFactory()

    return sdoserviceconsumerfactory

  instance = staticmethod(instance)
