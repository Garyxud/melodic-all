// -*- C++ -*-
/*!
 * @file  Process.cpp
 * @brief coil process management functions
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

#include <coil/Process.h>

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
#ifdef UNICODE
	// std::string -> LPTSTR
    std::wstring wcommand = string2wstring(command);
    LPTSTR lpcommand = new TCHAR[wcommand.size() + 1];
    _tcscpy(lpcommand, wcommand.c_str());
#else
	// std::string -> LPTSTR
    LPTSTR lpcommand = new TCHAR[command.size() + 1];
    _tcscpy(lpcommand, command.c_str());
#endif // UNICODE

    STARTUPINFO si;
    ZeroMemory( &si, sizeof(si) );
    si.cb = sizeof(si);

    PROCESS_INFORMATION pi;
    ZeroMemory( &pi, sizeof(pi) );

    if(!CreateProcess(NULL, lpcommand, NULL, NULL, FALSE, 0,
                      NULL, NULL, &si, &pi) )
      {
        delete lpcommand;
	return -1;
      }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    delete lpcommand;
    return 0;
  }

  int daemon(int nochdir, int noclose)
  {
    // not implemented
    return 0;
  }
}; // namespace coil

