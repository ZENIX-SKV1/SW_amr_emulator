// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#ifndef TEST_INCLUDE_GENERATOR_BASE_IMPL_H_
#define TEST_INCLUDE_GENERATOR_BASE_IMPL_H_

#include "generator/base.h"

namespace generator {
template <typename String>
typename std::enable_if_t<std::is_same_v<std::string, String>, String> generate() {
  static std::string_view charset = "abcdefghijklmnopqrstuvwxyz";
  static std::uniform_int_distribution dist(0, (int)charset.length() - 1);
  size_t len = 8;
  std::string gen(len, ' ');

  for (int i = 0; i < len; i++) {
    gen[i] = charset[dist(RNG::get())];
  }

  return gen;
}

template <typename Int8>
typename std::enable_if_t<std::is_same_v<Int8, int8_t> || std::is_same_v<Int8, uint8_t>, Int8>
generate() {
  static std::uniform_int_distribution dist((short)std::numeric_limits<Int8>::min(),
                                            (short)std::numeric_limits<Int8>::max());
  return static_cast<Int8>(dist(RNG::get()));
}

template <typename Integer>
typename std::enable_if_t<std::is_integral_v<Integer> && !std::is_same_v<Integer, int8_t> &&
                              !std::is_same_v<Integer, uint8_t> && !std::is_same_v<Integer, bool>,
                          Integer>
generate() {
  static std::uniform_int_distribution dist(std::numeric_limits<Integer>::min(),
                                            std::numeric_limits<Integer>::max());
  return dist(RNG::get());
}

template <typename Float>
typename std::enable_if_t<std::is_floating_point_v<Float>, Float> generate() {
  static std::uniform_real_distribution dist(std::numeric_limits<Float>::min(),
                                             std::numeric_limits<Float>::max());
  return dist(RNG::get());
}

template <typename Bool> typename std::enable_if_t<std::is_same_v<Bool, bool>, Bool> generate() {
  static std::bernoulli_distribution dist(0.5);
  return dist(RNG::get());
}

template <typename Vector> typename std::enable_if_t<is_std_vector_v<Vector>, Vector> generate() {
  static std::uniform_int_distribution dist(0, 5);
  auto len = dist(RNG::get());

  Vector gen(len);
  std::generate(gen.begin(), gen.end(), generate<typename Vector::value_type>);

  return gen;
}

template <typename Optional>
typename std::enable_if_t<is_std_optional_v<Optional>, Optional> generate() {
  static std::bernoulli_distribution dist(0.66);
  if (dist(RNG::get())) {
    return generate<typename Optional::value_type>();
  } else {
    return std::nullopt;
  }
}

template <typename Timestamp>
typename std::enable_if_t<std::is_same_v<Timestamp, std::chrono::system_clock::time_point>,
                          Timestamp>
generate() {
  return std::chrono::system_clock::time_point(std::chrono::milliseconds(generate<int64_t>()));
}

template <typename T> void generate_to(T &to) { to = generate<T>(); }

}  // namespace generator

#endif  // TEST_INCLUDE_GENERATOR_BASE_IMPL_H_
