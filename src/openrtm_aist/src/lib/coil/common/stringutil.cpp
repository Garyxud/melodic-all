// -*- C++ -*-
/*!
 * @file StringUtil.cpp
 * @brief String operation utility
 * @date $Date: 2007-12-31 03:08:07 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: StringUtil.cpp 826 2008-08-26 08:13:39Z n-ando $
 *
 */

#include <algorithm>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include <coil/stringutil.h>
#include <string.h>
#include <cctype>
#include <cstdio>

#ifdef __QNX__
using std::toupper;
using std::isalpha;
#endif

namespace coil
{
  /*!
   * @if jp
   * @brief string から wstring への変換
   * @else
   * @brief string to wstring conversion
   * @endif
   */
  std::wstring string2wstring(std::string str)
  {
    std::wstring wstr(str.length(), L' ');
    std::copy(str.begin(),str.end(),wstr.begin());
    return wstr;
  }

  /*!
   * @if jp
   * @brief wstring から string への変換
   * @else
   * @brief wstring to string conversion
   * @endif
   */
  std::string wstring2string(std::wstring wstr)
  {
    std::string str(wstr.length(), ' ');
    std::copy(wstr.begin(), wstr.end(), str.begin());
    return str;
  }

  /*!
   * @if jp
   * @brief 大文字への変換
   * @else
   * @brief Uppercase String Transformation
   * @endif
   */
  void toUpper(std::string& str)
  {
    std::transform(str.begin(), str.end(), str.begin(),
                   (int (*)(int))std::toupper); 
  }

  /*!
   * @if jp
   * @brief 小文字への変換
   * @else
   * @brief Lowercase String Transformation
   * @endif
   */
  void toLower(std::string& str)
  {
    std::transform(str.begin(), str.end(), str.begin(),
                   (int (*)(int))std::tolower); 
  }

  /*!
   * @if jp
   * @brief 入力ストリームから1行読み込む
   * @else
   * @brief Read a line from input stream
   * @endif
   */
  int getlinePortable(std::istream& istr, std::string& line)
  {
    char c;
    std::stringstream s;
    
    while (istr.get(c))
      {
        if (c == '\n')
          {
            break;
          }
        else if (c == '\r')
          {
            if (istr.peek() == '\n')
              {
                istr.ignore();
              }
            break;
          }
        else
          {
            s << c;
          }
      }
    line = s.str();
    return static_cast<int>(line.size());
  }
  
  /*!
   * @if jp
   * @brief 文字列がエスケープされているか判断する
   * @else
   * @brief Check whether the character is escaped or not
   * @endif
   */
  bool isEscaped(const std::string& str, std::string::size_type pos)
  {
    --pos;
    unsigned int i;
    for (i = 0; (pos >= 0) && str[pos] == '\\'; --pos, ++i) ;
    // If the number of \ is odd, delimiter is escaped.
    return (i % 2) == 1;
  }
  
  /*!
   * @if jp
   * @brief 文字列をエスケープするためのFunctor
   * @else
   * @brief Functor to escape string
   * @endif
   */
  struct escape_functor
  {
    escape_functor() {};
    void operator()(const char c)
    {
      if      (c == '\t')  str += "\\t";
      else if (c == '\n')  str += "\\n";
      else if (c == '\f')  str += "\\f";
      else if (c == '\r')  str += "\\r";
      else if (c == '\\')  str += "\\\\";
      //    else if (c == '\"')  str += "\\\"";
      //    else if (c == '\'')  str += "\\\'";
      else str.push_back(c);
    }
    std::string str;
  };
  
  /*!
   * @if jp
   * @brief 文字列をエスケープする
   * @else
   * @brief Escape string
   * @endif
   */
  std::string escape(const std::string str)
  {
    return for_each(str.begin(), str.end(), escape_functor()).str;
  }
  
  /*!
   * @if jp
   * @brief 文字列をアンエスケープするためのFunctor
   * @else
   * @brief Functor to unescape string
   * @endif
   */
  struct unescape_functor
  {
    unescape_functor() : count(0) {};
    void operator()(char c)
    {
      if (c == '\\')
        {
          ++count;
          if (!(count % 2))
            {
              str.push_back(c);
            }
        }
      else
        {
          if (count > 0 && (count % 2))
            {
              count = 0;
              if      (c == 't')  str.push_back('\t');
              else if (c == 'n')  str.push_back('\n');
              else if (c == 'f')  str.push_back('\f');
              else if (c == 'r')  str.push_back('\r');
              else if (c == '\"') str.push_back('\"');
              else if (c == '\'') str.push_back('\'');
              else str.push_back(c);
            }
          else
            {
              count = 0;
              str.push_back(c);
            }
        }
    }
    std::string str;
    int count;
  };
  
  /*!
   * @if jp
   * @brief 文字列のエスケープを戻す
   * @else
   * @brief Unescape string
   * @endif
   */
  std::string unescape(const std::string str)
  {
    return for_each(str.begin(), str.end(), unescape_functor()).str;
  }

  /*!
   * @if jp
   * @brief 文字列の空白文字を削除する
   * @else
   * @brief Erase blank characters of string
   * @endif
   */
  void eraseBlank(std::string& str)
  {
    std::string::iterator it(str.begin());

    while (it != str.end())
      {
        if (*it == ' ' || *it == '\t')
          {
            it = str.erase(it);
          }
        else
          {
            ++it;
          }
      }

  }
  
  /*!
   * @if jp
   * @brief 文字列の先頭の空白文字を削除する
   * @else
   * @brief Erase the head blank characters of string
   * @endif
   */
  void eraseHeadBlank(std::string& str)
  {
    if (str.empty()) return;
    while (str[0] == ' ' || str[0] == '\t') str.erase(0, 1);
  }
  
  /*!
   * @if jp
   * @brief 文字列の末尾の空白文字を削除する
   * @else
   * @brief Erase the tail blank characters of string
   * @endif
   */
  void eraseTailBlank(std::string& str)
  {
    if (str.empty()) return;
    while ((str[str.size() - 1] == ' ' || str[str.size() - 1] == '\t') &&
           !isEscaped(str, str.size() - 1))
      str.erase(str.size() - 1, 1);
  }

  /*!
   * @if jp
   * @brief 文字列の先頭・末尾の空白文字を削除する
   * @else
   * @brief Erase the head blank and the tail blank characters of string
   * @endif
   */
  void eraseBothEndsBlank(std::string& str)
  {
    eraseHeadBlank(str);
    eraseTailBlank(str);
  }

  /*!
   * @if jp
   * @brief 文字列を正規化する
   * @else
   * @brief Erase the head/tail blank and replace upper case to lower case
   * @endif
   */
  std::string normalize(std::string& str)
  {
    eraseBothEndsBlank(str);
    toLower(str);
    return str;
  }
  
  /*!
   * @if jp
   * @brief 文字列を置き換える
   * @else
   * @brief Replace string
   * @endif
   */
  unsigned int replaceString(std::string& str, const std::string from,
                     const std::string to)
  {
    std::string::size_type pos(0);
    unsigned int cnt(0);

    while (pos != std::string::npos)
      {
        pos = str.find(from, pos);
        if (pos == std::string::npos) break;
        str.replace(pos, from.size(), to);
        pos += to.size();
        ++cnt;
      }
    return cnt;
  }
  
  /*!
   * @if jp
   * @brief 文字列を分割文字で分割する
   * @else
   * @brief Split string by delimiter
   * @endif
   */
  vstring split(const std::string& input,
                const std::string& delimiter,
                bool ignore_empty)
  {
    typedef std::string::size_type size;
    vstring results;
    size delim_size = delimiter.size();
    size found_pos(0), begin_pos(0), pre_pos(0), substr_size(0);
    
    if (input.empty()) return results;
    
    //  if (input.substr(0, delim_size) == delimiter)
    //    begin_pos = pre_pos = delim_size;
    
    while (1)
      {
        //    REFIND:
        found_pos = input.find(delimiter, begin_pos);
        if (found_pos == std::string::npos) 
          {
            std::string substr(input.substr(pre_pos));
            eraseHeadBlank(substr);
            eraseTailBlank(substr);
            if (!(substr.empty() && ignore_empty)) results.push_back(substr);
            break;
          }
        /*
          if (isEscaped(input, found_pos))
          {
          begin_pos = found_pos + delim_size;
          goto REFIND;
          }
        */
        substr_size = found_pos - pre_pos;
        if (substr_size >= 0)
          {
            std::string substr(input.substr(pre_pos, substr_size));
            eraseHeadBlank(substr);
            eraseTailBlank(substr);
            if (!(substr.empty() && ignore_empty)) results.push_back(substr);
          }
        begin_pos = found_pos + delim_size;
        pre_pos   = found_pos + delim_size;
      }
    return results;
  }
  
  /*!
   * @if jp
   * @brief 大文字に変換する Functor
   * @else
   * @brief Functor to convert to capital letters
   * @endif
   */
  struct Toupper
  {
    void operator()(char &c)
    {
      c = toupper(c);
    }
  };
  
  /*!
   * @if jp
   * @brief 与えられた文字列をbool値に変換する
   * @else
   * @brief Convert given string into bool value
   * @endif
   */
  bool toBool(std::string str, std::string yes, std::string no, 
              bool default_value)
  {
    std::for_each(str.begin(), str.end(), Toupper());
    std::for_each(yes.begin(), yes.end(), Toupper());
    std::for_each(no.begin(),  no.end(),  Toupper());
    
    if (str.find(yes) != std::string::npos)
      return true;
    else if (str.find(no) != std::string::npos)
      return false;
    else
      return default_value;
  }

  /*!
   * @if jp
   * @brief 文字列リスト中にある文字列が含まれるかどうかを判断する
   * @else
   * @brief Include if a string is included in string list
   * @endif
   */
  bool includes(const vstring& list, std::string value, bool ignore_case)
  {
    if (ignore_case) { toLower(value); }

    for (int i(0), len(static_cast<int>(list.size())); i < len; ++i)
      {
        std::string str(list[i]);
        if (ignore_case) { toLower(str); }
        if (str == value) return true;
      }
    return false;
  }

  /*!
   * @if jp
   * @brief 文字列リスト中にある文字列が含まれるかどうかを判断する
   * @else
   * @brief Include if a string is included in string list
   * @endif
   */
  bool includes(const std::string& list, std::string value, bool ignore_case)
  {
    vstring vlist(split(list, ","));
    return includes(vlist, value, ignore_case);
  }
  
  /*!
   * @if jp
   * @brief 与えられた文字列が絶対パスかどうかを判断する
   * @else
   * @brief Investigate whether the given string is absolute path or not
   * @endif
   */
  bool isAbsolutePath(const std::string& str)
  {
    // UNIX absolute path is begun from '/'
    if (str[0] == '/') return true;
    // Windows absolute path is begun from '[a-zA-Z]:\'
    if (isalpha(str[0]) && (str[1] == ':') && str[2] == '\\') return true;
    // Windows network file path is begun from '\\'
    if (str[0] == '\\' && str[1] == '\\') return true;
    
    return false;
  }
  
  /*!
   * @if jp
   * @brief 与えられた文字列がURLかどうかを判断する
   * @else
   * @brief Investigate whether the given string is URL or not
   * @endif
   */
  bool isURL(const std::string& str)
  {
    typedef std::string::size_type size;
    size pos;
    if (str.empty()) return false;
    pos = str.find(":");
    if ((pos != 0) &&
        (pos != std::string::npos) &&
        (str[pos + 1] == '/') &&
        (str[pos + 2] == '/'))
      return true;
    return false;
  }
  
  /*!
   * @if jp
   * @brief リスト内の文字列を検索する Functor
   * @else
   * @brief Functor to find string in a list
   * @endif
   */
  struct unique_strvec
  {
    void operator()(const std::string& s)
    {
      if (std::find(str.begin(), str.end(), s) == str.end())
        return str.push_back(s);
    }
    vstring str;
  };

  /*!
   * @if jp
   * @brief 与えられた文字列をstd::stringに変換
   * @else
   * @brief Convert the given string to std::string.
   * @endif
   */
  template<>
  bool stringTo<std::string>(std::string& val, const char* str)
  {
    if (str == 0) { return false; }
    val = str;
    return true;
  }
  
  /*!
   * @if jp
   * @brief 与えられた文字列リストから重複を削除
   * @else
   * @brief Eliminate duplication from the given string list
   * @endif
   */
  vstring unique_sv(vstring sv)
  {
    return std::for_each(sv.begin(), sv.end(), unique_strvec()).str;
  }
  
  /*!
   * @if jp
   * @brief 与えられた文字列リストからCSVを生成
   * @else
   * @brief Create CSV file from the given string list
   * @endif
   */
  std::string flatten(vstring sv)
  {
    if (sv.size() == 0) return "";
    
    std::string str;
    for (size_t i(0), len(sv.size() - 1); i < len; ++i)
      {
        str += sv[i] + ", ";
      }
    return str + sv.back();
  }
  
  /*!
   * @if jp
   * @brief 与えられた文字列リストを引数リストに変換
   * @else
   * @brief Convert the given string list into the argument list
   * @endif
   */
  char** toArgv(const vstring& args)
  {
    char** argv;
    size_t argc(args.size());
    
    argv = new char*[argc + 1];
    
    for (size_t i(0); i < argc; ++i)
      {
        size_t sz(args[i].size());
        argv[i] = new char[sz + 1];
        strncpy(argv[i], args[i].c_str(), sz);
        argv[i][sz] = '\0';
      }
    argv[argc] = NULL;
    return argv;
  }

  /*!
   * @if jp
   * @brief 指定された書式に変換
   * @else
   * @brief Convert it into a format given with an argumen
   * @endif
   */
  std::string sprintf(char const * __restrict fmt, ...)
  {
#ifndef LINE_MAX
#define LINE_MAX 1024
#endif
    char str[LINE_MAX];
    va_list ap;
    
    va_start(ap, fmt);
#ifdef WIN32
    _vsnprintf(str, LINE_MAX - 1, fmt, ap);
#else
    vsnprintf(str, LINE_MAX - 1, fmt, ap);
#endif
    va_end(ap);
    return std::string(str);
  }

}; // namespace coil
