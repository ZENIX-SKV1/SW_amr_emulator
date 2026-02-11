#pragma once

#include <memory>
#include <string>
#include "yaml_config.h"  

#include "idead_reckoning.h"
#include "dead_reckoning_euler.h"
#include "sd_dead_reckoning_euler.h"

class DeadReckoningModelFactory 
{
public:
    static std::shared_ptr<ideadReckoningModel> create(
        const std::string& model_type,
        const AmrConfig& config  // 전체 config 전달
    );
};