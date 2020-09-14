// -*- C++ -*-
/*!
 * @file OS_posix.h
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

#include <string>
#include <sys/utsname.h>
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h>

extern "C"
{
  extern char *optarg;
};

namespace coil
{
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
  typedef ::utsname utsname;
  inline int uname(utsname* name)
  {
    return ::uname(name);
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
  typedef ::pid_t pid_t;
  inline pid_t getpid()
  {
    return ::getpid();
  }

  /*!
   * @if jp
   *
   * @brief 呼び出し元プロセスの親プロセスのプロセスID取得
   *
   * 呼び出し元プロセスの親プロセスのプロセスIDを返す。
   *
   * @return プロセスID
   *
   * @else
   *
   * @brief Get process ID of the parent process
   *
   * Return a process ID of the parent process.
   *
   * @return Process ID
   *
   * @endif
   */
  inline pid_t getppid()
  {
    return ::getppid();
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
  inline char* getenv(const char *name)
  {
    return ::getenv(name);
  }


  /* Global Variables for getopt() */

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
      : optarg(::optarg), optind(1), opterr(1), optopt(0), m_argc(argc), m_argv(argv), m_opt(opt), m_flag(flag)
    {
      ::optind = 1;
#ifdef __QNX___
      optind_last = 1;
#endif
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
      ::optind = 1;
#ifdef __QNX__
      optind_last = 1;
#endif
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
      ::opterr = opterr;
#ifndef __QNX__
      ::optind = optind;
#else
      ::optind = optind_last;
      ::optarg = 0;
#endif
      int result = getopt(m_argc, m_argv, m_opt);
#ifdef __QNX__
        if(::optind == optind_last)
	  {
            ::optind++;
            result = getopt(m_argc, m_argv, m_opt);
            optind_last = ::optind;
	  }
#endif
      optarg = ::optarg;
      optind = ::optind;
      optopt = ::optopt;
#if __QNX__
      if(optind_last < m_argc) { ++optind_last; }
#endif
      return result;
    }

    char* optarg;     //! オプション引数
    int optind;       //! 処理対象引数
    int opterr;       //! エラー表示 0:抑止、1:表示
    int optopt;       //! オプション文字が足りない時、多い時にセットされる
#ifdef __QNX__
    int optind_last;
#endif

  private:
    int m_argc;
    char* const * m_argv;
    const char* m_opt;
    int m_flag;
  };
    
};

#endif // COIL_OS_H
