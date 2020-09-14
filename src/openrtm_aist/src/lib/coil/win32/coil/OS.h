// -*- C++ -*-
/*!
 * @file OS_win32.h
 * @brief OS class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef COIL_OS_H
#define COIL_OS_H


#ifdef WINVER
#undef WINVER
#endif
#define WINVER 0x0500

#ifdef _WIN32_WINNT
#undef _WIN32_WINNT
#endif
#define _WIN32_WINNT 0x0500

#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <process.h>
#include <stdlib.h>
#include <winbase.h>

extern "C"
{
  extern char *optarg;
};

namespace coil
{

#define COIL_UTSNAME_LENGTH 65
  struct utsname
  {
    char sysname[COIL_UTSNAME_LENGTH];
    char nodename[COIL_UTSNAME_LENGTH];
    char release[COIL_UTSNAME_LENGTH];
    char version[COIL_UTSNAME_LENGTH];
    char machine[COIL_UTSNAME_LENGTH];
  };

  /*!
   * @if jp
   *
   * @brief システム情報取得
   *
   * システム情報を構造体に設定して返す。
   *
   * @param name 構造体名称
   *
   * @return 0: 成功, -1: 失敗
   *
   * @else
   *
   * @brief Get System information
   *
   * Return a system information to a structure.
   *
   * @param name Name of structure
   *
   * @return 0: Successful, -1: failed
   *
   * @endif
   */
  inline int uname(utsname* name)
  {
    if (name == NULL) return 0;
    int ret(0);

    // name.sysname
    ::strcpy(name->sysname, "Win32");

    // name.release, name.version
    OSVERSIONINFO version_info;
    version_info.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
    if (::GetVersionEx(&version_info) == false)
      ret = -1;

    const char *os;
    if (version_info.dwPlatformId == VER_PLATFORM_WIN32_NT)
      {
        os = "Windows NT %d.%d";
      }
    else
      {
        os = "Windows CE %d.%d";
      }
    
    sprintf(name->release, os,
            (int) version_info.dwMajorVersion,
            (int) version_info.dwMinorVersion);

    sprintf(name->version, "Build %d %s",
            (int) version_info.dwBuildNumber,
            version_info.szCSDVersion);

    // name.machine
    SYSTEM_INFO sys_info;
    GetSystemInfo(&sys_info);
    WORD arch = sys_info.wProcessorArchitecture;
    char cputype[COIL_UTSNAME_LENGTH/2];
    char subtype[COIL_UTSNAME_LENGTH/2];

    switch (arch)
      {
        case PROCESSOR_ARCHITECTURE_INTEL:
          strcpy(cputype, "Intel");
          if (sys_info.wProcessorLevel == 3)
            strcpy(subtype, "80386");
          else if (sys_info.wProcessorLevel == 4)
            strcpy(subtype, "80486");
          else if (sys_info.wProcessorLevel == 5)
            strcpy(subtype, "Pentium");
          else if (sys_info.wProcessorLevel == 6)
            strcpy(subtype, "Pentium Pro");
          else if (sys_info.wProcessorLevel == 7)
            strcpy(subtype, "Pentium II");
          else
            strcpy(subtype, "Pentium Family");
          break;
      default:
        strcpy(cputype, "Unknown");
        strcpy(subtype, "Unknown");
      }
    sprintf(name->machine, "%s %s", cputype, subtype);

    // name.nodename
    DWORD len = COIL_UTSNAME_LENGTH;
    if (GetComputerNameExA(ComputerNameDnsHostname,
							name->nodename,
                            &len) == false)
    ret = -1;
    return ret;
  }

  /*!
   * @if jp
   *
   * @brief 呼び出し元プロセスのプロセスID取得
   *
   * 呼び出し元プロセスのプロセスIDを返す。
   *
   * @return プロセスID
   *
   * @else
   *
   * @brief Get process ID of the caller process
   *
   * Return a process ID of the caller process.
   *
   * @return Process ID
   *
   * @endif
   */
  typedef int pid_t;
  inline pid_t getpid()
  {
    return ::_getpid();
  }

  /*!
   * @if jp
   *
   * @brief 呼び出し元プロセスの親プロセスのプロセスID取得
   *
   * 呼び出し元プロセスの親プロセスのプロセスIDを返す。
   *
   * @return 0
   *
   * @else
   *
   * @brief Get process ID of the parent process
   *
   * Return a process ID of the parent process.
   *
   * @return 0
   *
   * @endif
   */
  inline pid_t getppid()
  {
    return 0;
  }

  /*!
   * @if jp
   *
   * @brief 環境変数取得
   *
   * 環境変数を返す。
   *
   * @param name 環境変数名称
   *
   * @return 環境変数の値(NULL: 該当なし)
   *
   * @else
   *
   * @brief Get environment variable
   *
   * Return a environment variable.
   *
   * @param name Name of environment variable
   *
   * @return Value of environment variable(NULL: nonexistent)
   *
   * @endif
   */
  inline char* getenv(const char* name)
  {
    return ::getenv(name);
  }

  static int	opterr = 1,		/* if error message should be printed */
                optind = 1,		/* index into parent argv vector */
                optopt,			/* character checked for validity */
                optreset;		/* reset getopt */
  static char	*optarg;		/* argument associated with option */

#define	BADCH	(int)'?'
#define	BADARG	(int)':'
#define	EMSG	""
  
  /*!
   * @if jp
   *
   * @brief コマンドライン引数解析関数
   *
   * コマンドライン引数を解析する。
   *
   * @return 解析結果
   *
   * @else
   *
   * @brief Function of parses the command line arguments
   *
   * Parses the command line arguments.
   *
   * @return Result of parses.
   *
   * @endif
   */
  static int getopt(int nargc, char * const *nargv, const char *ostr)
  {
    static char *place = EMSG;		/* option letter processing */
    const char *oli;			/* option letter list index */
    char *p;
    
    if (optreset || !*place) {		/* update scanning pointer */
      optreset = 0;
      /* Check that the scanning reached the last element. */
      if (optind >= nargc)
      { 
        place = EMSG;
        return(-1);
      }
      for(;;)
      {
        /* Check that the scanning reached the last element. */
        if (optind >= nargc)
        {                
          place = EMSG;
          return(-1);
        }
        place = nargv[optind];
        /* Check that the first character of the element is '-'. */
        if (*place != '-')
        {                  
          /* When the head of the element is not '-',        */
          /* Scan the next element disregarding the element. */
          optind++;
                   
        }
        else
        {
          break;
        }
      }
      if (place[1] && *++place == '-') {	/* found "--" */
        ++optind;
        place = EMSG;
        return(-1);
      }
    }					/* option letter okay? */
    if ((optopt = (int)*place++) == (int)':' ||
        !(oli = strchr(ostr, optopt))) {
      /*
       * if the user didn't specify '-' as an option,
       * assume it means -1 (EOF).
       */
      if (optopt == (int)'-')
        return(-1);
      if (!*place)
        ++optind;
      if (opterr && *ostr != ':') {
        if (!(p = strrchr(*nargv, '/')))
          p = *nargv;
        else
          ++p;
        fprintf(stderr, "%s: illegal option -- %c\n",
                p, optopt);
      }
      return(BADCH);
    }
    if (*++oli != ':') {			/* don't need argument */
      optarg = NULL;
      if (!*place)
        ++optind;
    }
    else {					/* need an argument */
      if (*place)			/* no white space */
        optarg = place;
      else if (nargc <= ++optind) {	/* no arg */
        place = EMSG;
        if (!(p = strrchr(*nargv, '/')))
          p = *nargv;
        else
          ++p;
        if (*ostr == ':')
          return(BADARG);
        if (opterr)
          fprintf(stderr,
                  "%s: option requires an argument -- %c\n",
                  p, optopt);
        return(BADCH);
      }
      else				/* white space */
        optarg = nargv[optind];
      place = EMSG;
      ++optind;
    }
    return(optopt);				/* dump back option letter */
  }

  /*!
   * @if jp
   *
   * @class GetOpt
   * @brief GetOpt クラス
   *
   * @else
   *
   * @class GetOpt
   * @brief GetOpt class
   *
   * @endif
   */
  class GetOpt
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。
     *
     * @param name オブジェクト名
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @param name Object name
     *
     * @endif
     */
    GetOpt(int argc, char* const argv[], const char* opt, int flag)
      : m_argc(argc), m_argv(argv), m_opt(opt), m_flag(flag), optind(1), opterr(1), optopt(0)

    {
      this->optarg = coil::optarg;
      coil::optind = 1;
    }

    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * デストラクタ。
     *
     * @else
     *
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    ~GetOpt()
    {
      coil::optind = 1;
    }

    /*!
     * @if jp
     *
     * @brief コマンドライン引数解析
     *
     * コマンドライン引数を解析する。
     *
     * @return 解析結果
     *
     * @else
     *
     * @brief Parses the command line arguments
     *
     * Parses the command line arguments.
     *
     * @return Result of parses.
     *
     * @endif
     */
    int operator()()
    {
      coil::opterr = opterr;
      coil::optind = optind;

      int result = getopt(m_argc, m_argv, m_opt);

      optarg = coil::optarg;
      optind = coil::optind;
      optopt = coil::optopt;

      return result;
    }

    char* optarg;     //! オプション引数
    int optind;       //! 処理対象引数
    int opterr;       //! エラー表示 0:抑止、1:表示
    int optopt;       //! オプション文字が足りない時、多い時にセットされる
 

  private:
    int m_argc;
    char* const * m_argv;
    const char* m_opt;
    int m_flag;

  };
    
};

#endif // COIL_OS_H
