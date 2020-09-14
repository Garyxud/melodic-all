/*
 * EvologicsStream.cpp
 *
 *  Created on: 21 nov. 2016
 *      Author: centelld
 */

#include <cstring>
#include <dccomms_utils/EvologicsStream.h>

namespace dccomms_utils {

void defaultNotificationCallback(const std::string &notification) {
  // do nothing
}

EvologicsStream::EvologicsStream() {
  // TODO Auto-generated constructor stub
  notificationReceivedCallback = &defaultNotificationCallback;
  _InitNotificationsFilter();
}

EvologicsStream::~EvologicsStream() {
  // TODO Auto-generated destructor stub
}

void EvologicsStream::SetNotificationReceivedCallback(f_notification func) {
  notificationReceivedCallback = func;
}

void EvologicsStream::_InitNotificationsFilter() {
  // to filter notifications:
  memcpy(buffer, bes, BES_LENGTH);
  notifHeader = buffer;
  notifHeaderCurPtr = notifHeader;
  notifHeaderEndPtr = notifHeader + BES_LENGTH;

  notifQueue = notifHeaderEndPtr;

  strcpy(notifQueue, "\r\n");
  notifQueueCurPtr = notifQueue;
  notifQueueEndPtr = notifQueueCurPtr + strlen(notifQueue);

  data = (uint8_t *)notifQueueEndPtr;
  cdata = data;

  ndata = 0;

  notifLength = 0;
  maxNotifLength = 200;
}

// This method filters and logs whatever received notification from the modem
int EvologicsStream::ReadData(void *dbuf, int n, bool block) {
  int reqn;
  if (ndata < n) {
    reqn = n - ndata;
  } else {
    reqn = 1;
  }
  int res = _Recv(cdata, reqn, block);

  int nreturn; // the number of valid bytes that will be returned (nreturn <=
               // ret)

  uint8_t *firstNoValid = data;
  uint8_t *noData = cdata + res;
  while (cdata < noData) {
    // if we have not read a notification header...
    if (notifHeaderCurPtr < notifHeaderEndPtr) {
      if (*cdata != *notifHeaderCurPtr) {
        notifHeaderCurPtr = notifHeader;
        firstNoValid = cdata + 1;
      } else {
        notifHeaderCurPtr++;
      }
      cdata++;
    }
    // else, a notification has been found, and we will discard
    // all bytes until the notification's queue reception
    else if (notifQueueCurPtr < notifQueueEndPtr) {
      if (notifLength <= maxNotifLength) {
        if (*cdata != *notifQueueCurPtr) {
          notifQueueCurPtr = notifQueue;
        } else {
          notifQueueCurPtr++;
        }
        notifLength++;
      } else {
        notifLength = 0;
        notifHeaderCurPtr = notifHeader;
        notifQueueCurPtr = notifQueue;

        firstNoValid = cdata + 1;
      }
      cdata++;
    } else {
      notifLength = 0;
      notifHeaderCurPtr = notifHeader;
      notifQueueCurPtr = notifQueue;

      // all data from firstNoValid to cdata is a notification.
      int notifsize = cdata - firstNoValid;

      // save and log the nofification, and call callback
      memcpy(lastNotification, firstNoValid, notifsize);
      lastNotification[notifsize - 2] = 0;
      Log->info("notification received from modem: {}", lastNotification);
      notificationReceivedCallback(std::string(lastNotification));

      // copy the following bytes upon the notif ptr
      int endData = noData - cdata;
      memcpy(firstNoValid, cdata, endData);

      // reset cdata to notif ptr (firstNoValid)
      cdata = firstNoValid;
      noData = cdata + endData;
    }
  }
  ndata = firstNoValid > data ? firstNoValid - data : 0;
  nreturn = n <= ndata ? n : ndata;

  if (nreturn > 0) {
    memcpy(dbuf, data, nreturn);
    uint8_t *end = data + nreturn;

    if (firstNoValid == end) {
      cdata = data;
      ndata = 0;
    } else if (firstNoValid > end) // there are valid bytes that will not be
                                   // returned yet (n < ret)
    {
      memcpy(data, end, firstNoValid - end);
      cdata = end;
      ndata = cdata - data;
    } else // cdata < end (Impossible)
    {
      Log->critical("this message should not be shown, else there is a bug");
    }
  }

  // return number of valid bytes (after filter) copied in dbuf
  return nreturn;
}
} /* namespace merbots */
