#ifndef COLLISION_HANDLER_H
#define COLLISION_HANDLER_H

#include "casadi_kin_dyn.h"
#include <Eigen/Dense>

namespace casadi_kin_dyn {

class CasadiCollisionHandler {

public:
  typedef std::shared_ptr<CasadiCollisionHandler> Ptr;

  CasadiCollisionHandler(CasadiKinDyn::Ptr kd);

  CasadiCollisionHandler(const CasadiCollisionHandler &);

  size_t numPairs() const;

  bool distance(Eigen::Ref<const Eigen::VectorXd> q,
                Eigen::Ref<Eigen::VectorXd> d);

  bool distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                        Eigen::Ref<Eigen::MatrixXd> J);

  ~CasadiCollisionHandler();

private:
  class Impl;
  std::unique_ptr<Impl> _impl;
  Impl &impl();
  const Impl &impl() const;
};

} // namespace casadi_kin_dyn

#endif // COLLISION_HANDLER_H
