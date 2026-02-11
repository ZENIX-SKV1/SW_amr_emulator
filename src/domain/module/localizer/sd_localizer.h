#pragma once
#include <memory>
#include "ilocalizer.h"
#include "idead_reckoning.h"  

class SDLocalizer : public ILocalizer 
{
public:
    explicit SDLocalizer(std::shared_ptr<ideadReckoningModel> dr_model);

    void setInitialPose(double x, double y, double theta) override;
    void update(double left_rpm, double right_rpm, double dt) override;
    void getPose(double& x, double& y, double& theta) const override;

private:
    std::shared_ptr<ideadReckoningModel> dr_model_;
};