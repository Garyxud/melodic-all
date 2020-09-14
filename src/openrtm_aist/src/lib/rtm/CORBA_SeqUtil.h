// -*- C++ -*-
/*!
 * @file CORBA_SeqUtil.h
 * @brief CORBA sequence utility template functions
 * @date $Date: 2007-12-31 03:06:24 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef CORBA_SEQUTIL_H
#define CORBA_SEQUTIL_H

#include <rtm/RTC.h>
#include <rtm/Manager.h>

/*!
 * @if jp
 * @namespace CORBA_SeqUtil
 *
 * @brief CORBA sequence ヘルパーテンプレート関数
 *
 * CORBA sequence に対して以下のユーティリティテンプレート関数を提供する。
 * 操作はスレッドセーフではないので、スレッドセーフに操作したい場合は、
 * 対象となるシーケンス値を適切にmutex等で保護する必要がある。
 *
 * - for_each()
 * - find()
 * - push_back()
 * - insert()
 * - front()
 * - back()
 * - erase()
 * - clear()
 *
 * @else
 *
 * @namespace CORBA_SeqUtil
 *
 * @brief CORBA sequence helper template functions
 *
 * This group provides the following utility function to CORBA sequence.
 * Since these functions are not thread-safe operations,
 * if the sequence would be operated in thread-safe,
 * the value should be protected by mutex properly.
 *
 * - for_each()
 * - find()
 * - push_back()
 * - insert()
 * - front()
 * - back()
 * - erase()
 * - clear()
 *
 * @endif
 */
namespace CORBA_SeqUtil
{
  /*!
   * @if jp
   *
   * @brief CORBA sequence に対して functor を適用する
   *
   * CORBA sequence 全ての要素に対して、与えられた functor を適用する。
   * functor は void functor(CORBA sequence の要素) の形式をとる必要がある。
   *
   * @param seq Functor を適用する CORBA sequence
   * @param f CORBA sequence の要素を処理する Functor
   *
   * @return 全ての要素を処理した Functor
   *
   * @since 0.4.0
   *
   * @else
   *
   * @brief Apply the functor to all CORBA sequence elements
   *
   * Apply the given functor to the given CORBA sequence.
   * functor should be void functor(CORBA sequence element).
   *
   * @param seq CORBA sequence to be applied the functor
   * @param f A functor to process CORBA sequence elements
   *
   * @return Functor that processed all CORBA sequence elements
   *
   * @endif
   */
  template <class CorbaSequence, class Functor>
  Functor for_each(CorbaSequence& seq, Functor f)
  {
    CORBA::ULong len;
    len = seq.length();
    for (CORBA::ULong i = 0; i < len; ++i)
      {
        f(seq[i]);
      }
    return f;
  }

  /*!
   * @if jp
   * @brief CORBA sequence の中から functor に適合する要素のインデックスを返す
   *
   * CORBA sequence 全ての要素に対して、与えられた functor を適用し、
   * functor が true を返すようそのインデックスを返す。
   * functor は bool functor(const CORBA sequence の要素) の形式をとり、
   * 適合する要素に対して true を返す必要がある。
   *
   * @param seq Functor を適用する CORBA sequence
   * @param f CORBA sequence から要素を見つける Functor
   *
   * @return Functor に適合する要素のインデックス。
   *         見つからないときは -1 を返す。
   *
   * @else
   *
   * @brief Return the index of CORBA sequence element that functor matches
   *
   * This operation applies the given functor to the given CORBA sequence,
   * and returns the index of the sequence element that the functor matches.
   * The functor should be bool functor(const CORBA sequence element) type,
   * and it would return true, if the element matched the functor.
   *
   * @param seq CORBA sequence to be applied the functor
   * @param f A functor to process CORBA sequence elements
   *
   * @return The index of the element that functor matches.
   *         If no element found, it would return -1.
   *
   * @endif
   */
  template <class CorbaSequence, class Functor>
  CORBA::Long find(const CorbaSequence& seq, Functor f)
  {
    CORBA::ULong len(seq.length());
    for (CORBA::ULong i = 0; i < len; ++i)
      {
        if (f(seq[i])) { return (CORBA::Long)i; }
      }
    return -1;
  }

  /*!
   * @if jp
   * @brief CORBA sequence の最後に要素を追加する
   *
   * CORBA sequence の最後に与えられた要素を追加する。
   * CORBA sequence の長さは自動的に拡張される。
   *
   * @param seq 要素を追加する CORBA sequence
   * @param elem 追加する要素
   *
   * @else
   *
   * @brief Push the new element back to the CORBA sequence
   *
   * Add the given element to the last of CORBA sequence.
   * The length of the CORBA sequence will be expanded automatically.
   *
   * @param seq CORBA sequence to be added a new element
   * @param elem The new element to be added to the CORBA sequence
   *
   * @endif
   */
  template <class CorbaSequence, class SequenceElement>
  void push_back(CorbaSequence& seq, SequenceElement elem)
  {
    CORBA::ULong len(seq.length());
    seq.length(len + 1);
    seq[len] = elem;
  }

  /*!
   * @if jp
   * @brief CORBA sequence をマージする
   *
   * 与えられた CORBA sequence をマージする。
   *
   * @param seq1 マージされる CORBA sequence
   * @param seq2 マージされる CORBA sequence
   *
   * @else
   *
   * @brief Merge the elements of the CORBA sequence
   *
   * Merge given CORBA sequences.
   *
   * @param seq1 merge target CORBA sequence
   * @param seq2 merge target CORBA sequence
   *
   * @endif
   */
  template <class CorbaSequence>
  void push_back_list(CorbaSequence& seq1, const CorbaSequence& seq2)
  {
    CORBA::ULong len1(seq1.length());
    CORBA::ULong len2(seq2.length());
    CORBA::ULong len(len1 + len2);
    seq1.length(len);

    for (CORBA::ULong i = 0; i < len2; ++i)
      {
        seq1[len1 + i] = seq2[i];
      }
  }

  /*!
   * @if jp
   * @brief CORBA sequence に要素を挿入する
   *
   * CORBA sequence の index の位置に要素を加える。
   * index が 与えられた　CORBA sequence の最大の index より大きい場合
   * 最後の要素として加えられる。
   * CORBA sequence の長さは自動的に拡張される。
   *
   * @param seq 要素を追加する CORBA sequence
   * @param elem 追加する要素
   * @param index 要素を追加する位置
   *
   * @else
   *
   * @brief Insert the element to the CORBA sequence
   *
   * Insert a new element in the given position to the CORBA sequence.
   * If the given index is greater than the length of the sequence,
   * the given element is pushed back to the last of the sequence.
   * The length of the CORBA sequence will be expanded automatically.
   *
   * @param seq The CORBA sequence to be inserted a new element
   * @param elem The new element to be inserted the sequence
   * @param index The inserting position
   *
   * @endif
   */
  template <class CorbaSequence, class SequenceElement>
  void insert(CorbaSequence& seq, SequenceElement& elem, CORBA::ULong index)
  {
    CORBA::ULong len(seq.length());
    if (index > len)
      {
        push_back(seq, elem);
        return;
      }

    seq.length(len + 1);
    for (CORBA::ULong i = len; i > index; --i)
      {
        seq[i] = seq[i - 1];
      }
    seq[index] = elem;
  }

  /*!
   * @if jp
   * @brief CORBA sequence の先頭要素を取得する
   *
   * CORBA sequence の先頭要素を取得する。
   * seq[0] と同じ。
   *
   * @param seq 要素を取得する CORBA sequence
   *
   * @return 取得した要素
   *
   * @else
   *
   * @brief Get the front element of the CORBA sequence
   *
   * This operation returns seq[0].
   *
   * @param seq CORBA sequence which acquires an element
   *
   * @return An acquisition element
   *
   * @param seq The CORBA sequence to be get the element
   *
   * @endif
   */
  template <class CorbaSequence, class SequenceElement>
  SequenceElement& front(CorbaSequence& seq)
  {
    return seq[0];
  }

  /*!
   * @if jp
   * @brief CORBA sequence の末尾要素を取得する
   *
   * CORBA sequence の末尾要素を取得する。
   * seq[seq.length() - 1] と同じ。
   *
   * @param seq 要素を取得する CORBA sequence
   *
   * @return 取得した要素
   *
   * @else
   *
   * @brief Get the last element of the CORBA sequence
   *
   * This operation returns seq[seq.length() - 1].
   *
   * @param seq The CORBA sequence to be get the element
   *
   * @return An acquisition element
   *
   * @endif
   */
  template <class CorbaSequence, class SequenceElement>
  SequenceElement& back(CorbaSequence& seq)
  {
    return seq[seq.length() - 1];
  }

  /*!
   * @if jp
   * @brief CORBA sequence の指定された位置の要素を削除する
   *
   * 指定されたインデックスの要素を削除する。
   * 削除された要素は詰められ、sequence の長さは1減る。
   *
   * @param seq 要素を削除する CORBA sequence
   * @param index 削除する要素のインデックス
   *
   * @else
   *
   * @brief Erase the element of the specified index
   *
   * This operation removes the element of the given index.
   * The other elements are closed up around the hole.
   *
   * @param seq The CORBA sequence to be get the element
   * @param index The index of the element to be removed
   *
   * @endif
   */
  template <class CorbaSequence>
  void erase(CorbaSequence& seq, CORBA::ULong index)
  {
    CORBA::ULong len(seq.length());
    if (index > len) return;

    for (CORBA::ULong i = index; i < len - 1; ++i)
      {
        seq[i] = seq[i + 1];
      }
    seq.length(len - 1);
  }

  /*!
   * @if jp
   *
   * @brief シーケンスの要素を述語にしたがって削除する
   *
   * このオペレーションは述語として与えられた関数オブジェクトの
   * 条件が真のとき、そのシーケンスの要素を削除する。
   *
   * @param seq 要素検索対象の CORBA sequence
   * @param f 削除するシーケンスを決定する術語
   *
   * @else
   *
   * @brief Remove an element of a sequence according to a predicate
   *
   * This operation removes the element from sequence when a condition of
   * a function object given as a predicate is True.
   *
   * @param seq target CORBA sequence
   * @param f predicate which decides a sequence to remove
   *
   * @endif
   */
  template <class CorbaSequence, class Functor>
  void erase_if(CorbaSequence& seq, Functor f)
  {
    CORBA::Long index;
    index = find(seq, f);
    if (index < 0) return;
    CORBA_SeqUtil::erase(seq, index);
  }

  /*!
   * @if jp
   * @brief CORBA sequence の全要素を削除
   *
   * CORBA sequence の全要素を削除する。
   * seq.length(0) と同じ。
   *
   * @else
   *
   * @brief Erase all the elements of the CORBA sequence
   *
   * same as seq.length(0).
   *
   * @endif
   */
  template <class CorbaSequence>
  void clear(CorbaSequence& seq)
  {
    seq.length(0);
  }
  // End of CORBA sequence helper functions


  template <class CorbaRefSequence>
  coil::vstring refToVstring(const CorbaRefSequence& objlist)
  {
    coil::vstring iorlist;
    CORBA::ORB_ptr orb = ::RTC::Manager::instance().getORB();

    for (CORBA::ULong i(0), len(objlist.length()); i < len; ++i)
      {
#ifndef ORB_IS_RTORB
        CORBA::String_var str_var = orb->object_to_string(objlist[i].in());
#else
        CORBA_Object obj = (objlist.cobj())->_buffer[i];
        CORBA::String_var str_var = orb->object_to_string2(obj);
#endif
        iorlist.push_back(str_var.in());
      }
    return iorlist;
  }
}; // namespace CORBA_SeqUtil
#endif // CORBA_SEQUTIL_H
