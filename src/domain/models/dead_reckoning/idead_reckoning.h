#pragma once

class ideadReckoningModel 
{
public:
    virtual ~ideadReckoningModel() = default;

    // 초기 위치 설정
    virtual void setInitialPose(double x, double y, double theta) = 0;
    // 상태 갱신 : 입력 rpm과 경과시간
    virtual void update(double left_rpm, double right_rpm, double dt) = 0;
    // 현재 추정 위치/자세 반환
    virtual void getPose(double& x, double& y, double& theta) const = 0;
};