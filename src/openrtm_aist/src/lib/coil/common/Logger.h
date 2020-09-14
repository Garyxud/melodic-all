// -*- C++ -*-
/*!
 * @file Logger.h
 * @brief log_streambuf and log_stream class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009
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

#ifndef LOGGER_H
#define LOGGER_H

#include <limits.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <coil/Mutex.h>
#include <coil/Guard.h>

#ifndef LINE_MAX
#define LINE_MAX  1024
#endif

#define BUFFER_LEN LINE_MAX

namespace coil
{
  /*!
   * @if jp
   *
   * @class log_streambuf
   * @brief log_streambuf テンプレートクラス
   *
   * @else
   *
   * @class log_streambuf
   * @brief log_streambuf template class
   *
   * @endif
   */
  template <typename _CharT, typename _Traits=std::char_traits<_CharT> >
  class log_streambuf
    : public ::std::basic_streambuf<_CharT, _Traits>
  {
  public:
    typedef _CharT                                       char_type;
    typedef _Traits                                      traits_type;
    typedef std::basic_streambuf<char_type, traits_type> streambuf_type;
    typedef coil::Mutex Mutex;
    typedef coil::Guard<coil::Mutex> Guard;

    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @endif
     */
    log_streambuf()
      : streambuf_type()
    {
      char *pStart = m_buf;
      char *pEnd = m_buf + (BUFFER_LEN - 1);
      this->setp(pStart, pEnd);
      this->setg(pStart, pStart, pEnd);
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
    virtual ~log_streambuf()
    {
    }

    /*!
     * @if jp
     *
     * @brief ストリームを追加する
     *
     * log_streambuf に実際の出力先であるストリームを追加する。
     * ここで追加されたストリームの解体責任はユーザにあり、
     * log_streambuf 解体時には解体されることはない。
     * また追加されているストリームを log_streambuf の解体前に
     * 解体してはならない。ストリームの解体は log_streambuf の解体後に、
     * ユーザが解体しなければならない。
     *
     * @param stream std::basic_streambuf 型のストリームへのポインタ
     *
     * @else
     *
     * @brief Destructor
     *
     * This operation adds a stream that is actual output stream.
     * User has responsibility to destruct the stream object, since
     * this object never destructs the stream objects.  The added
     * stream objects should not be destructed before the destruction
     * of this object.  The stream destruction should be done by user
     * after removing it from this object or destructing this object.
     *
     * @param stream a pointer to std::basic_streambuf type stream object
     *
     * @endif
     */
    void addStream(streambuf_type* stream, bool cleanup = false)
    {
      m_streams.push_back(Stream(stream, cleanup));
    }

    /*!
     * @if jp
     *
     * @brief ストリームを削除する
     *
     * log_streambuf から出力先であるストリームを削除する。
     * ここで削除されたストリームの解体責任はユーザにある。
     *
     * @param stream std::basic_streambuf 型のストリームへのポインタ
     *
     * @else
     *
     * @brief Destructor
     *
     * This operation remove a stream that is actual output stream.
     * User has responsibility to destruct the stream object.
     *
     * @param stream a pointer to std::basic_streambuf type stream object
     * @return Whether removing the stream succeeded.
     *
     * @endif
     */
    bool removeStream(streambuf_type* stream)
    {
      std::vector<coil::log_streambuf<char>::Stream>::iterator it;
      for( it = m_streams.begin(); it != m_streams.end(); it++ )
      {
          if (it->stream_ == stream)
            {
              m_streams.erase(it);
              return true;
            }
      }
      return false;
    }

    /*!
     * @if jp
     *
     * @brief ストリームバッファ取得
     *
     * ストリームバッファを返す。
     *
     * @return streambuf_type リスト
     *
     * @else
     *
     * @brief Get stream buffer list
     *
     * Return a stream buffer list.
     *
     * @return streambuf_type list
     *
     * @endif
     */
    std::vector<streambuf_type*> getBuffers()
    {
      std::vector<streambuf_type*> buffs;
      for (int i(0), len(m_streams.size()); i < len; ++i)
        {
          buffs.push_back(m_streams[i].stream_);
        }
      return buffs;
    }
   
  protected:
    /*!
     * @if jp
     *
     * @brief basic_streambuf::xsputn のオーバーライド
     *
     * @param s 入力文字列へのポインタ
     * @param n 入力文字数
     * @return 入力文字列のサイズ
     *
     * @else
     *
     * @brief override of basic_streambuf::xsputn
     *
     * @param s a pointer to input characters
     * @param n number of input characters
     * @return input stream size
     *
     * @endif
     */
    virtual std::streamsize xsputn(const char_type* s, std::streamsize n)
    {
      stream_sputn();
      for (int i(0), len(m_streams.size()); i < len; ++i)
        {
          Guard gaurd(m_streams[i].mutex_);
          m_streams[i].stream_->sputn(s, n);
        }
      return n;
    }

    /*!
     * @if jp
     *
     * @brief ストリームへ出力する。
     *
     * @return 出力した文字数 
     *
     * @else
     *
     * @brief Write the stream buffer in stream.  
     *
     * @return The number of characters written.
     *
     * @endif
     */
    virtual std::streamsize stream_sputn()
    {
      int bytes_to_write;
      bytes_to_write = this->pptr() - this->gptr();
      if (bytes_to_write > 0)
        {
          for (int i(0), len(m_streams.size()); i < len; ++i)
            {
              Guard gaurd(m_streams[i].mutex_);
              m_streams[i].stream_->sputn(this->gptr(), bytes_to_write);
            }
          this->gbump(bytes_to_write);
          if (this->gptr() >= this->pptr())
            {
              this->pbump(this->pbase() - this->pptr());
              this->gbump(this->pbase() - this->gptr());
            }
        }
      return bytes_to_write;
    }

    /*!
     * @if jp
     *
     * @brief ストリームへ出力する。
     *
     * @param s 入力文字列へのポインタ
     * @param n 入力文字数
     * @return 入力文字列のサイズ
     *
     * @else
     *
     * @brief Writes up to n characters from the array pointed by s 
     *        to the output sequence controlled by the stream buffer.
     *
     * @param s a pointer to input characters
     * @param n number of input characters
     * @return The number of characters written.
     *
     * @endif
     */
    virtual std::streamsize stream_sputn(const char_type* s, std::streamsize n)
    {
      
      for (int i(0), len(m_streams.size()); i < len; ++i)
        {
          Guard gaurd(m_streams[i].mutex_);
          m_streams[i].stream_->sputn(s, n);
          m_streams[i].stream_->pubsync();
        }
      return n;
    }
    /*!
     * @if jp
     *
     * @brief basic_streambuf::overflow のオーバーライド
     *
     * @param c 入力文字
     * @return 返却値
     *
     * @else
     *
     * @brief override of basic_streambuf::overflow
     *
     * @param c input character
     * @return return value
     *
     * @endif
     */

    virtual int overflow (int c = traits_type::eof())
    {
      Guard guard(m_mutex);
//      if (traits_type::eq_int_type(c, traits_type::eof()))
//        return c;
//
//      char_type last_char = traits_type::to_char_type(c);
//      if (sputn(&last_char, 1) != 1)
//        return traits_type::eof();
//      else
//        return c;

      if (this->pbase())
        {
          if (this->pptr() > this->epptr() || this->pptr() < this->pbase())
            return traits_type::eof();
          // Add extra character to buffer if not EOF
          if (!traits_type::eq_int_type(c, traits_type::eof()))
            {
              this->pbump(-1);
              *(this->pptr()) = traits_type::to_char_type(c);
              this->pbump(1);
            }
          // Number of characters to write to file
          int bytes_to_write = this->pptr() - this->gptr();
          // Overflow doesn't fail if nothing is to be written
          if (bytes_to_write > 0)
            {
              if (stream_sputn(this->gptr(), bytes_to_write) != bytes_to_write)
                return traits_type::eof();
              // Reset next pointer to point to pbase on success
              this->pbump(this->pbase() - this->pptr());
              this->gbump(this->pbase() - this->gptr());
            }
        }
      // Write extra character to file if not EOF
      else if (!traits_type::eq_int_type(c, traits_type::eof()))
        {
          // Impromptu char buffer (allows "unbuffered" output)
          char_type last_char = traits_type::to_char_type(c);
          // If gzipped file won't accept this character, fail
          if (stream_sputn(&last_char, 1) != 1)
            return traits_type::eof();
        }
      // If you got here, you have succeeded (even if c was EOF)
      // The return value should therefore be non-EOF
      if (traits_type::eq_int_type(c, traits_type::eof()))
        return traits_type::not_eof(c);
      else
        return c;
    } 

    /*!
     * @if jp
     *
     * @brief basic_streambuf::sync のオーバーライド
     *
     * @return 返却値
     *
     * @else
     *
     * @brief override of basic_streambuf::sync
     *
     * @return return value
     *
     * @endif
     */
    virtual int sync()
    {
      if (this->pbase())
        {
          Guard guard(m_mutex);
          if (this->pptr() > this->epptr() || this->pptr() < this->pbase())
            return -1;

          int bytes_to_write;
          bytes_to_write = this->pptr() - this->gptr();
          if (bytes_to_write > 0)
            {
              if (stream_sputn(this->gptr(), bytes_to_write) != bytes_to_write)
                {
                  return -1;
                }
              this->gbump(bytes_to_write);
              if (this->gptr() >= this->pptr())
                {
                  this->pbump(this->pbase() - this->pptr());
                  this->gbump(this->pbase() - this->gptr());
                }
            }
        }
      else
        {
          this->overflow();
        }
      return 0;
    }

  public:

    /*!
     * @if jp
     *
     * @brief ストリーム管理用構造体
     *
     * @else
     *
     * @brief Structure for stream management
     *
     * @endif
     */
    struct Stream
    {
      Stream(streambuf_type* stream, bool cleanup = false)
        : stream_(stream), cleanup_(cleanup)
      {
      }

      virtual ~Stream()
      {
      }

      Stream(const Stream& x)
        : stream_(x.stream_)
      {
      }

      Stream& operator=(const Stream& x)
      {
        Stream tmp(x);
        tmp.swap(*this);
        return *this;
      }

      void swap(Stream& x)
      {
        streambuf_type* stream = x.stream_;
        bool cleanup = x.cleanup_;

        x.stream_ = this->stream_;
        x.cleanup_ = this->cleanup_; 

        this->stream_ = stream;
        this->cleanup_ = cleanup;
      }
      mutable Mutex mutex_;
      streambuf_type* stream_;
      bool cleanup_;
    };

  private:

    /*!
     * @if jp
     *
     * @brief コピーコンストラクタ
     *
     * コピーコンストラクタ
     *
     * @param x log_streambuf オブジェクト
     *
     * @else
     *
     * @brief Copy Constructor
     *
     * Copy Constructor
     *
     * @param x log_streambuf object
     *
     * @endif
     */
    log_streambuf(const log_streambuf& x);

    /*!
     * @if jp
     *
     * @brief 代入演算子
     *
     * log_streambufオブジェクトをコピーする。
     *
     * @param x log_streambuf オブジェクト
     *
     * @return 代入結果
     *
     * @else
     *
     * @brief Assignment operator
     *
     * Copy a log_streambuf object.
     *
     * @param x log_streambuf object.
     *
     * @return Assignment result.
     *
     * @endif
     */
    log_streambuf& operator=(const log_streambuf& x);

    std::vector<Stream> m_streams;
    Mutex m_mutex;
    char m_buf[BUFFER_LEN];
  };


  /*!
   * @if jp
   *
   * @class log_stream
   * @brief log_stream テンプレートクラス
   *
   * @else
   *
   * @class log_stream
   * @brief log_stream template class
   *
   * @endif
   */
  template <typename _CharT, typename _Traits=std::char_traits<_CharT> >
  class log_stream
    : public std::basic_ostream<_CharT, _Traits>
  {
  public:
    // Types:
    typedef _CharT                                       char_type;
    typedef _Traits                                      traits_type;
    typedef std::basic_ostream<char_type, traits_type>   ostream_type;
    typedef std::basic_streambuf<char_type, traits_type> streambuf_type;
    typedef coil::Mutex Mutex;
    typedef coil::Guard<Mutex> Guard;

    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @param streambuf basic_streambuf 型オブジェクト
     * @param levelmin ログレベルの最小値
     * @param levelmax ログレベルの最大値
     * @param デフォルトのログレベル
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @param streambuf basic_streambuf type object
     * @param levelmin minimum value for log level
     * @param levelmax maximum value for log level
     * @param level default log level
     *
     * @endif
     */
    log_stream(streambuf_type* sb, int levelmin, int levelmax, int level)
      : ostream_type(sb),
        m_minLevel(levelmin), m_maxLevel(levelmax), m_logLevel(level)
    {
      if (m_minLevel >= m_maxLevel) throw std::bad_alloc();
      this->init(sb);
    }

    /*!
     * @if jp
     *
     * @brief メッセージのヘッダ追加関数
     *
     * サブクラスにおいてこの関数をオーバーライドし、
     * ログメッセージに適当な時刻などのヘッダを追加する。
     *
     * @else
     *
     * @brief Message header appender function
     *
     * Subclasses of this class should override this operation, and
     * this function should be defined to append some header to the
     * log messages.
     *
     * @endif
     */
    virtual void header(int level)
    {
      return;
    }

    /*!
     * @if jp
     *
     * @brief ログレベル設定
     *
     * ログレベルを設定する。
     *
     * @param level ログレベル
     *
     * @else
     *
     * @brief Set the log level
     *
     * Set the log level.
     *
     * @param level Log level
     *
     * @endif
     */
    bool setLevel(int level)
    {
      if (m_minLevel <= level && level <= m_maxLevel)
        {
          m_logLevel = level;
          return true;
        }
      return false;
    }

    /*!
     * @if jp
     *
     * @brief ログレベル取得
     *
     * ログレベルを取得する。
     *
     * @return ログレベル
     *
     * @else
     *
     * @brief Get the log level
     *
     * Get the log level.
     *
     * @return Log level
     *
     * @endif
     */
    int getLevel() const
    {
      return m_logLevel;
    }
    
    /*!
     * @if jp
     *
     * @brief ロックモード設定
     *
     * ロックモードを有効にする。
     *
     * @else
     *
     * @brief Enable the lock mode
     *
     * Enable the lock mode.
     *
     * @endif
     */
    void enableLock()
    {
      m_lockEnable = true;
    }
    
    /*!
     * @if jp
     *
     * @brief ロックモード解除
     *
     * ロックモードを無効にする。
     *
     * @else
     *
     * @brief Disable the lock mode
     *
     * Disable the lock mode.
     *
     * @endif
     */
    void disableLock()
    {
      m_lockEnable = false;
    }
    
    /*!
     * @if jp
     *
     * @brief ログストリームの取得
     *
     * 指定されたログレベルを判断し、ログストリームを取得する。
     * 指定されたログレベルが設定されているログレベル以下の場合には、本クラスを
     * 返す。
     * 指定されたログレベルが設定されているログレベルを超えている場合には、
     * ダミーログクラスを返す。
     *
     * @param level 指定ログレベル
     *
     * @return 対象ログストリーム
     *
     * @else
     *
     * @brief Acquire log stream
     *
     * Investigate the specified log level and get its log stream.
     * If the specified log level is under the set log level, this class
     * will be returned.
     * If the specified log level exceeds the set log level, a dummy log class
     * will be returned.
     *
     * @param level The specified log level
     *
     * @return Target log stream
     *
     * @endif
     */
    ostream_type& level(int level)
    {
      if (m_minLevel <= level && level <= m_logLevel)
	{
          header(level);
	  return *this;
	}
      else
	{
	  return m_dummy;
	}
    }
    
    /*!
     * @if jp
     *
     * @brief ログレベル有効チェック
     *
     * 指定されたログレベルが有効範囲かチェックし、有効・無効を返す。
     *
     * @param level ログレベル
     *
     * @return true: 有効, false: 無効
     *
     * @else
     *
     * @brief Log level effective check
     *
     * Check it whether an appointed log level is an effective range 
     * and return effective or invalidity.
     *
     * @param level Log level
     *
     * @return true: Valid, false: Invalid
     *
     * @endif
     */
    bool isValid(int level) const
    {
      return m_minLevel <= level && level <= m_logLevel;
    }

    /*!
     * @if jp
     *
     * @brief ログロック取得
     * ロックモードが設定されている場合、ログのロックを取得する。
     *
     * @else
     *
     * @brief Acquire log lock
     * Acquire log lock when the lock mode is set.
     *
     * @endif
     */
    inline void lock()
    {
      if (m_lockEnable) m_mutex.lock();
    }
    
    /*!
     * @if jp
     *
     * @brief ログロック解放
     * ロックモードが設定されている場合に、ログのロックを解放する。
     *
     * @else
     *
     * @brief Release the log lock
     * Release the log lock when the lock mode is set.
     *
     * @endif
     */
    inline void unlock()
    {
      if (m_lockEnable) m_mutex.unlock();
    }

    
  protected:

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
    ~log_stream(){};

    /*!
     * @if jp
     *
     * @brief デフォルトコンストラクタ
     *
     * デフォルトコンストラクタ
     *
     * @else
     *
     * @brief Default constructor
     *
     * Default constructor
     *
     * @endif
     */
    log_stream();

    /*!
     * @if jp
     *
     * @brief コピーコンストラクタ
     *
     * コピーコンストラクタ
     *
     * @param x log_stream オブジェクト
     *
     * @else
     *
     * @brief Copy Constructor
     *
     * Copy Constructor
     *
     * @param x log_stream object
     *
     * @endif
     */
    log_stream(const log_stream& x);

    /*!
     * @if jp
     *
     * @brief 代入演算子
     *
     * log_streamオブジェクトをコピーする。
     *
     * @param x log_streamオブジェクト
     *
     * @return 代入結果
     *
     * @else
     *
     * @brief Assignment operator
     *
     * Copy a log_stream object.
     *
     * @param x log_stream object.
     *
     * @return Assignment result.
     *
     * @endif
     */
    log_stream& operator=(const log_stream& x);

  private:
    int m_minLevel, m_maxLevel;
    int m_logLevel;

    /*!
     * @if jp
     * @brief ダミーストリーム
     * @else
     * @brief Dummy log
     * @endif
     */
    std::ofstream m_dummy;
  public:

    /*!
     * @if jp
     * @brief ロック有効モード
     * @else
     * @brief Lock enable mode
     * @endif
     */
    static bool m_lockEnable;

    /*!
     * @if jp
     * @brief 排他制御オブジェクト
     * @else
     * @brief Mutual exclusion object
     * @endif
     */
    static Mutex m_mutex;
  };

  template <typename _CharT, typename _Traits >
  bool log_stream<_CharT,_Traits >::m_lockEnable = true;

  template <typename _CharT, typename _Traits >
  coil::Mutex log_stream<_CharT,_Traits>::m_mutex("Mutex for Logger.");

  typedef log_streambuf<char> LogStreamBuffer;
  typedef log_stream<char> LogStream;

};
#endif // LOGGER_H
