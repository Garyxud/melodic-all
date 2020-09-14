/*
**  Copyright (C) 2012 Aldebaran Robotics
**  See COPYING for the license
*/

#pragma once

#ifndef QICORE_API_H_
#define QICORE_API_H_

#include <qi/macro.hpp>

#ifndef SWIG
#define QICORE_API QI_LIB_API(qicore)
#else
#define QICORE_API
#endif

#endif /* !QICORE_API_H_ */
