#!/usr/bin/env python
# -*- coding: euc-jp -*-
# @file codegen.py
# @brief simple code template generator
# @date $Date: 2008-02-27 11:31:04 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2006
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#
# [usage]
# codegen.py [class_name]
#

import sys
import os
import yat

src_cpp = """// -*- C++ -*-
/*!
 * @file  [class_name].cpp
 * @brief [class_name] class
 * @date  [dollar]Date[dollar]
 
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * [dollar]Id[dollar]

 *
 */

#include <rtm/[class_name].h>

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  [class_name]::[class_name]()
  {
    ;
  }

}; // namespace RTC
"""

src_h = """// -*- C++ -*-
/*!
 * @file  [class_name].h
 * @brief [class_name] class
 * @date  [dollar]Date[dollar]

 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * [dollar]Id[dollar]

 *
 */

#ifndef [class_name]_h
#define [class_name]_h

namespace RTC
{
  /*!
   * @if jp
   * @class [class_name]

   * @brief [class_name] クラス
   * @else
   * @class [class_name]

   * @brief [class_name] class
   * @endif
   */
  class [class_name]

  {
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    [class_name]();

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~[class_name](){};

  protected:
  
  private:

  };
 
};     // namespace RTC
#endif // [class_name]_h

"""

def make_dict(classname):
    data = {}
    data["dollar"] = "$"
    data["class_name"] = classname
    data["src_cpp"]    = classname + ".cpp"
    data["src_h"]      = classname + ".h"
    return data

class src_gen:
    def __init__(self, data):
        self.data = data
        return

    def check_overwrite(self, fname):
        """
        Check file exist or not.
        """
        msg = " already exists. Overwrite or merge? (y/n)"
        if (os.access(fname, os.F_OK)):
            ans = raw_input("\"" + fname + "\"" + msg)
            if (ans == "y" or ans == "Y"):
                return file(fname, "w")
            else:
                return None
        else:
            return file(fname, "w")
    
    def gen(self, fname, temp_txt, data):
        fd = self.check_overwrite(fname)
        if not fd:
            return
        t = yat.Template(temp_txt)
        text = t.generate(data)
	fd.write(text)
        fd.close()
        print "\"" + fname + "\"" " was generated."
        return

    def gen_all(self):
        self.write_src_cpp()
        self.write_src_h()
        return

    def write_src_cpp(self):
        self.gen(self.data["src_cpp"], src_cpp, self.data)
        return

    def write_src_h(self):
        self.gen(self.data["src_h"], src_h, self.data)
        return


if len(sys.argv) < 2:
    sys.exit(1)

class_name = sys.argv[1]

gen  = src_gen(make_dict(class_name))
gen.gen_all()
