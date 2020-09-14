/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ftdi.h>
#include <stdio.h>


int main(int argc, char **argv)
{

  // Initialize FTDI context
  struct ftdi_context ftdic;
  ftdi_init(&ftdic);

  // List devices
  int ret = 0;
  struct ftdi_device_list *devlist;
  if ((ret = ftdi_usb_find_all(&ftdic, &devlist, 0x0403, 0x6001)) >= 0) {
    printf("Found %i FTDI devices\n", ret);
    struct ftdi_device_list *curdev;
    char manufacturer[128], description[128], serial[128];
    int i = 0;
    for (curdev = devlist; curdev != NULL; i++) {
      printf("Checking device: %d\n", i);
      if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128, serial, 128)) < 0) {
        printf("ftdi_usb_get_strings failed: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        break;
      }
      printf("Manufacturer: %s, Description: %s, Serial: %s\n", manufacturer, description, serial);
      curdev = curdev->next;
    }
  } else {
    printf("ftdi_usb_find_all failed: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
  }
  ftdi_list_free(&devlist);

  // De-initialize FTDI context
  ftdi_deinit(&ftdic);

  return 0;
}

