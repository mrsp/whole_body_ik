  #include <whole_body_ik/LeakyIntegrator.h>


  LeakyIntegrator::LeakyIntegrator()
  {
        integral_ = 0.0;
        rate_ = 0.1;
        saturation_ = -1.;
  }
  void LeakyIntegrator::add(const double & value, double dt)
  {
    integral_ = (1. - rate_ * dt) * integral_ + dt * value;
    if (saturation_ > 0.)
    {
      saturate();
    }
  }

 void LeakyIntegrator::saturate()
  {
      if (integral_ < -saturation_)
      {
        integral_ = -saturation_;
      }
      else if (integral_ > saturation_)
      {
        integral_ = saturation_;
      }
  }
  
