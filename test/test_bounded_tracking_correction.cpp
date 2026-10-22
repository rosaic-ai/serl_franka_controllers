#include <cassert>
#include <iostream>
#include <limits>
#include "serl_franka_controllers/bounded_tracking_correction.h"
using namespace serl_franka_controllers;
using V=Eigen::Vector3d;
int main() {
  // Disabled and invalid input cannot wind up; rate and magnitude are vector limits.
  BoundedTrackingCorrection c;
  for(int i=0;i<1000;++i) assert(c.update(.001,V(.01,0,0),0,0,1,false).isZero());
  V previous=V::Zero();
  for(int i=0;i<10000;++i) {
    V force=c.update(.001,V(.008,.008,0),0,0,1,true);
    assert(force.norm()<=15.+1e-9);
    assert((force-previous).norm()<=.003+1e-9); previous=force;
  }
  assert(c.force().norm()>14.99);
  // Saturated state contains no hidden accumulator: reversal changes sign promptly.
  for(int i=0;i<5500;++i)c.update(.001,V(-.01,0,0),0,0,1,true);
  assert(c.force().x()<0);
  previous=c.force();c.update(.1,V(.01,0,0),0,0,1,true);
  assert((c.force()-previous).isZero());
  // Force trip latches, ramps down and cannot rearm itself after contact disappears.
  c.update(.001,V(.01,0,0),0,0,12,true);assert(c.tripped());
  for(int i=0;i<6000;++i)c.update(.001,V(.01,0,0),0,0,0,true);
  assert(c.force().norm()<1e-9 && c.tripped());
  c.clearTrip();for(int i=0;i<1000;++i)c.update(.001,V(.01,0,0),0,0,1,true);
  assert(c.force().x()>2.);
  // A moving target smoothly releases correction instead of accumulating path lag.
  for(int i=0;i<6000;++i)c.update(.001,V(.01,0,0),.02,0,1,true);
  assert(c.force().norm()<1e-9);
  c.reset();c.update(.001,V(std::numeric_limits<double>::quiet_NaN(),0,0),0,0,1,true);
  assert(c.tripped() && c.force().isZero());
  // Integrator uses elapsed time, rather than number of controller updates.
  BoundedTrackingCorrection a,b;
  for(int i=0;i<1000;++i)a.update(.001,V(.001,0,0),0,0,1,true);
  for(int i=0;i<2000;++i)b.update(.0005,V(.001,0,0),0,0,1,true);
  assert((a.force()-b.force()).norm()<.001);
  // Simple quasi-static disturbance model: a 10mm P-only residual falls below 1mm.
  // This model is a regression test, NOT a hardware or contact-safety proof.
  c.reset();double error=.01;
  for(int i=0;i<15000;++i) {
    V f=c.update(.001,V(error,0,0),0,0,8,true);
    error=(15.-f.x())/1500.;
  }
  assert(error>=0 && error<.001);
  std::cout<<"All bounded-correction tests passed; model residual="<<error*1000<<"mm\n";
}
