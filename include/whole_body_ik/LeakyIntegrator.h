#include <iostream>

class LeakyIntegrator
{
  public:
  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  void add(const double & value, double dt);

  /** Evaluate the output of the integrator.
   *
   */
  const double & eval() const
  {
    return integral_;
  }

  LeakyIntegrator();
  /** Get leak rate.
   *
   */
  inline double rate() const
  {
    return rate_;
  }

  /** Set the leak rate of the integrator.
   *
   * \param rate New leak rate.
   *
   */
  inline void rate(double rate)
  {
    rate_ = rate;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  inline void saturation(double s)
  {
    saturation_ = s;
  }

  /** Reset integral to zero.
   *
   */
  inline void setZero()
  {
    integral_=0.0;
  }
  inline void setInitialState(double value_)
  {
    integral_ = value_;
  }
private:
  void saturate();
  double integral_;
  double rate_;
  double saturation_;
};
