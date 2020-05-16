#include <whole_body_ik/LeakyIntegrator.h>

LeakyIntegrator::LeakyIntegrator()
{
  integral_ = 0.0;
  rate_ = 0.1;
  saturation_ = false;
}
void LeakyIntegrator::add(const double &value, double dt)
{

  double leak_ = (rate_ * dt) > 1.0 ? 1.0 : (rate_ * dt); //For numerical stability
  integral_ = (1.0 - leak_) * integral_ + dt * value;
  if (saturation_)
  {
    saturate();
  }
}

void LeakyIntegrator::saturate()
{
  if (integral_ < l_sat)
  {
    integral_ = l_sat;
  }
  else if (integral_ > u_sat)
  {
    integral_ = u_sat;
  }
}
