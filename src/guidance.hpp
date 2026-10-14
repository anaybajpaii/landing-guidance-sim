#pragma once
#include "lander.hpp"
#include "params.hpp"

class Guidance {
public:
    virtual ~Guidance() = default;
    virtual double desiredVy(const LanderState& s, double mass) const = 0;
};

class ConstantDecel : public Guidance {
public:
    explicit ConstantDecel(const Params& params);
    double desiredVy(const LanderState& s, double mass) const override;

private:
    Params params_;
};

class LateBurn : public Guidance {
public:
    explicit LateBurn(const Params& params);
    double desiredVy(const LanderState& s, double mass) const override;

private:
    Params params_;
};