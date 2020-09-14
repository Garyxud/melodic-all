// -*- C++ -*-
/*!
 * @file  Process.h
 * @brief Process handling functions
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef COIL_PROCESS_H
#define COIL_PROCESS_H

#include <stdlib.h>
#include <unistd.h>
#include <libgen.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <coil/stringutil.h>

namespace coil
{

  /*!
   * @if jp
   * @brief プロセスを起動する
   * @else
   * @brief Launching a process
   * @endif
   */
  int launch_shell(std::string command)
  {
    signal(SIGCHLD, SIG_IGN);

    pid_t pid;
    if((pid = fork()) < 0 )
      { // fork failed
        return -1; 
      }
    
    if (pid == 0) // I'm child process
      {
        //        signal(SIGCHLD, SIG_IGN);
        //        signal(SIGALRM, SIG_IGN);
        //        signal(SIGHUP , SIG_IGN);
        //        signal(SIGPIPE, SIG_IGN);
        //        signal(SIGTERM, SIG_IGN);
        setsid();
        //        close(0);
        //        close(1);
        //        close(2);
        //        open("/dev/null", O_RDWR);
        //        dup2(0, 1);
        //        dup2(0, 2);
        //        umask(0);

        coil::vstring vstr(::coil::split(command, " "));
        char* const * argv = ::coil::toArgv(vstr);

        execvp(vstr.front().c_str(), argv);
        
        return -1;
      }
    return 0;
  }

  int daemon(int nochdir, int noclose)
  {
    return daemon(nochdir, noclose);
  }


}; // namespace coil
#endif // COIL_PROCESS_H
