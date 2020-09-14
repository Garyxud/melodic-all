/*
 * EvologicsStream.h
 *
 *  Created on: 21 nov. 2016
 *      Author: centelld
 */

#ifndef MERBOTS_LIB_INCLUDE_MERBOTS_EVOLOGICSSTREAM_H_
#define MERBOTS_LIB_INCLUDE_MERBOTS_EVOLOGICSSTREAM_H_

#include <cpplogging/Logger.h>
#include <dccomms_utils/Constants.h>
#include <functional>
#include <queue>
#include <string>

namespace dccomms_utils {
using namespace cpplogging;

// virtual inheritance:
// http://www.cprogramming.com/tutorial/virtual_inheritance.html
class EvologicsStream : public virtual Logger {
public:
  EvologicsStream();
  virtual ~EvologicsStream();
  virtual int ReadData(void *dbuf, int n, bool block = true);

  typedef std::function<void(const std::string &notification)> f_notification;

  virtual void SetNotificationReceivedCallback(f_notification);

protected:
  f_notification notificationReceivedCallback;
  void _InitNotificationsFilter();
  virtual int _Recv(void *, int n, bool block = true) = 0;
  // To filter notifications from input:
  char buffer[EVO_BUFFER_SIZE];
  char *notifHeader, *notifQueue, *notifHeaderCurPtr, *notifHeaderEndPtr,
      *notifQueueCurPtr, *notifQueueEndPtr;
  uint8_t *data, *cdata;
  int ndata;
  char lastNotification[EVO_BUFFER_SIZE];
  int maxNotifLength, notifLength;
  char bes[BES_LENGTH + 1] = "+++";
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_EVOLOGICSSTREAM_H_ */
