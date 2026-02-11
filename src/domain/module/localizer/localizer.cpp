#include "localizer.h"
#include <iostream>

Localizer::Localizer(std::shared_ptr<ideadReckoningModel> dr_model)
    : dr_model_(dr_model)
{
}

void Localizer::setInitialPose(double x, double y, double theta)
{
    if (dr_model_)
    {
        dr_model_->setInitialPose(x, y, theta);
    }
}

void Localizer::update(double left_rpm, double right_rpm, double dt)
{

    std::cout << "Localizer::update : " << left_rpm << " " << right_rpm << std::endl;

    if (dr_model_)
    {
        dr_model_->update(left_rpm, right_rpm, dt);
    }
}

void Localizer::getPose(double& x, double& y, double& theta) const
{
    if (dr_model_)
    {
        dr_model_->getPose(x, y, theta);
    }
    else
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }
}