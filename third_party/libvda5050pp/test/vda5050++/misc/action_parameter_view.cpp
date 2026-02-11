// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/misc/action_parameter_view.h"

#include <catch2/catch_all.hpp>

#include "vda5050++/exception.h"

TEST_CASE("misc::ActionParameterView", "[misc]") {
  vda5050::ActionParameter p_int{"int", vda5050::json(123)};
  vda5050::ActionParameter p_bool{"bool", vda5050::json(true)};
  vda5050::ActionParameter p_float{"float", vda5050::json(-128.125)};
  vda5050::ActionParameter p_string{"string", vda5050::json("this_is_a_string")};
  std::vector<vda5050::ActionParameter> action_parameters{
      p_int,
      p_bool,
      p_float,
      p_string,
  };

  vda5050pp::misc::ActionParameterView view(action_parameters);

  SECTION("Getting a unknown key") {
    REQUIRE_THROWS_AS(view.get("unknown"), vda5050pp::VDA5050PPInvalidActionParameterKey);
    REQUIRE_THROWS_AS(view.getString("unknown"), vda5050pp::VDA5050PPInvalidActionParameterKey);
    REQUIRE_THROWS_AS(view.getInt("unknown"), vda5050pp::VDA5050PPInvalidActionParameterKey);
    REQUIRE_THROWS_AS(view.getFloat("unknown"), vda5050pp::VDA5050PPInvalidActionParameterKey);
    REQUIRE_THROWS_AS(view.getBool("unknown"), vda5050pp::VDA5050PPInvalidActionParameterKey);
    REQUIRE(view.tryGet("unknown") == std::nullopt);
    REQUIRE(view.tryGetString("unknown") == std::nullopt);
    REQUIRE(view.tryGetInt("unknown") == std::nullopt);
    REQUIRE(view.tryGetBool("unknown") == std::nullopt);
    REQUIRE(view.tryGetFloat("unknown") == std::nullopt);
  }

  SECTION("Getting a string") {
    REQUIRE(view.getString(p_string.key) == "this_is_a_string");
    REQUIRE_THROWS_AS(view.getString(p_float.key), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getString(p_bool.key), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getString(p_int.key), vda5050pp::VDA5050PPInvalidActionParameterType);
  }

  SECTION("Getting an integer") {
    REQUIRE(view.getInt("int") == 123);
    REQUIRE(view.tryGetBool("int") == std::nullopt);
    REQUIRE(view.tryGetString("int") == std::nullopt);
    REQUIRE_THROWS_AS(view.getBool("int"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getString("int"), vda5050pp::VDA5050PPInvalidActionParameterType);
  }

  SECTION("Getting a float") {
    REQUIRE(view.getFloat("float") == -128.125);
    REQUIRE_THROWS_AS(view.getBool("float"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getInt("float"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getString("float"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE(view.tryGetBool("float") == std::nullopt);
    REQUIRE(view.tryGetInt("float") == std::nullopt);
    REQUIRE(view.tryGetString("float") == std::nullopt);
  }

  SECTION("Getting a bool") {
    REQUIRE(view.getBool("bool") == true);
    REQUIRE_THROWS_AS(view.getInt("bool"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getFloat("bool"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE_THROWS_AS(view.getString("bool"), vda5050pp::VDA5050PPInvalidActionParameterType);
    REQUIRE(view.tryGetInt("bool") == std::nullopt);
    REQUIRE(view.tryGetFloat("bool") == std::nullopt);
    REQUIRE(view.tryGetString("bool") == std::nullopt);
  }
}