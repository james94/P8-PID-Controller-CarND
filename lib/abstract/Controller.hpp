#pragma once

class Controller {
public:
    virtual void UpdateError(double value) = 0;
    virtual double GetControlValue() const = 0;
    virtual ~Controller() {}
};
