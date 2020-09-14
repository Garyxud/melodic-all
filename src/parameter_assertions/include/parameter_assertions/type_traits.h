#ifndef SRC_TYPE_TRAITS_H
#define SRC_TYPE_TRAITS_H

#include <type_traits>
#include <vector>

namespace assertions::type_traits
{
template <bool B, typename T = void>
using disable_if = std::enable_if<!B, T>;

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename T>
struct is_vector : public std::false_type
{
};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : public std::true_type
{
};

template <typename T, typename A>
struct is_vector<const std::vector<T, A>> : public std::true_type
{
};

template <typename T>
struct is_number : public std::false_type
{
};

template <>
struct is_number<double> : public std::true_type
{
};

template <>
struct is_number<float> : public std::true_type
{
};

template <>
struct is_number<int> : public std::true_type
{
};

}  // namespace assertions::type_traits

#endif  // SRC_TYPE_TRAITS_H
