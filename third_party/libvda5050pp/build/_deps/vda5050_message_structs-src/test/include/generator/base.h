// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#ifndef TEST_INCLUDE_GENERATOR_H_

#include <chrono>
#include <memory>
#include <optional>
#include <random>
#include <type_traits>
#include <vector>

namespace generator {

class RNG {
private:
  static std::random_device real;
  static std::unique_ptr<std::mt19937> pseudo;

public:
  static void seed(std::optional<std::random_device::result_type> seed = std::nullopt) {
    RNG::pseudo = std::make_unique<std::mt19937>(seed.value_or(real()));
  }
  static std::mt19937 &get() {
    if (!RNG::pseudo) {
      RNG::seed();
    }
    return *RNG::pseudo;
  }
};

std::random_device RNG::real;
std::unique_ptr<std::mt19937> RNG::pseudo;

/// SFINAE type traits /////////////////////////////////////////////////////////////////////////////

template <typename T> struct is_std_vector : std::false_type {};
template <typename T, typename Alloc>
struct is_std_vector<std::vector<T, Alloc>> : std::true_type {};
template <typename T> inline constexpr bool is_std_vector_v = is_std_vector<T>::value;

template <typename T> struct is_std_optional : std::false_type {};
template <typename T> struct is_std_optional<std::optional<T>> : std::true_type {};
template <typename T> inline constexpr bool is_std_optional_v = is_std_optional<T>::value;

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T> void generate_to(T &to);

template <typename Vector> typename std::enable_if_t<is_std_vector_v<Vector>, Vector> generate();

template <typename Optional>
typename std::enable_if_t<is_std_optional_v<Optional>, Optional> generate();

template <typename Timestamp>
typename std::enable_if_t<std::is_same_v<Timestamp, std::chrono::system_clock::time_point>,
                          Timestamp>
generate();

template <typename String>
typename std::enable_if_t<std::is_same_v<std::string, String>, String> generate();

template <typename Int8>
typename std::enable_if_t<std::is_same_v<Int8, int8_t> || std::is_same_v<Int8, uint8_t>, Int8>
generate();

template <typename Integer>
typename std::enable_if_t<std::is_integral_v<Integer> && !std::is_same_v<Integer, int8_t> &&
                              !std::is_same_v<Integer, uint8_t> && !std::is_same_v<Integer, bool>,
                          Integer>
generate();

template <typename Float>
typename std::enable_if_t<std::is_floating_point_v<Float>, Float> generate();

template <typename Bool> typename std::enable_if_t<std::is_same_v<Bool, bool>, Bool> generate();

template <typename Vector> typename std::enable_if_t<is_std_vector_v<Vector>, Vector> generate();

template <typename Optional>
typename std::enable_if_t<is_std_optional_v<Optional>, Optional> generate();

template <typename Timestamp>
typename std::enable_if_t<std::is_same_v<Timestamp, std::chrono::system_clock::time_point>,
                          Timestamp>
generate();
}  // namespace generator

#define TEST_INCLUDE_GENERATOR_H_

#endif  // TEST_INCLUDE_GENERATOR_H_
