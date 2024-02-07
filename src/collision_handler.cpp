#include "casadi_kin_dyn/collision_handler.h"
#include "casadi_kin_dyn/casadi_kin_dyn.h"

#define PINOCCHIO_WITH_HPP_FCL
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <casadi/casadi.hpp>

namespace pin = pinocchio;

using namespace casadi_kin_dyn;

class CasadiCollisionHandler::Impl {

public:
  Impl(CasadiKinDyn::Ptr kd);

  size_t numPairs() const;

  bool distance(Eigen::Ref<const Eigen::VectorXd> q,
                Eigen::Ref<Eigen::VectorXd> d);

  bool distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                        Eigen::Ref<Eigen::MatrixXd> J);

  CasadiKinDyn::Ptr kd;

  ~Impl();

private:
  Eigen::VectorXd _last_q;
  Eigen::VectorXd _d_cached;

  pin::Model _mdl;
  pin::Data _data;
  pin::GeometryModel _geom_mdl;
  pin::GeometryData _geom_data;

  std::vector<Eigen::MatrixXd> _joint_J;
};

CasadiCollisionHandler::CasadiCollisionHandler(CasadiKinDyn::Ptr kd) {
  _impl = std::make_unique<Impl>(kd);
}

CasadiCollisionHandler::CasadiCollisionHandler(
    const CasadiCollisionHandler &other) {
  _impl = std::make_unique<Impl>(*other._impl);
  _impl->kd = std::make_shared<CasadiKinDyn>(*_impl->kd);
}

size_t CasadiCollisionHandler::numPairs() const { return impl().numPairs(); }

bool CasadiCollisionHandler::distance(Eigen::Ref<const Eigen::VectorXd> q,
                                      Eigen::Ref<Eigen::VectorXd> d) {
  return impl().distance(q, d);
}

bool CasadiCollisionHandler::distanceJacobian(
    Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<Eigen::MatrixXd> J) {
  return impl().distanceJacobian(q, J);
}

CasadiCollisionHandler::~CasadiCollisionHandler() {}

CasadiCollisionHandler::Impl &CasadiCollisionHandler::impl() { return *_impl; }

const CasadiCollisionHandler::Impl &CasadiCollisionHandler::impl() const {
  return *_impl;
}

CasadiCollisionHandler::Impl::Impl(CasadiKinDyn::Ptr _kd)
    : _mdl(std::any_cast<pin::Model>(_kd->modelHandle())), _data(_mdl),
      kd(_kd) {
  std::istringstream urdf_stream(_kd->urdf());

  _geom_mdl = pin::urdf::buildGeom(_mdl, urdf_stream,
                                   pin::GeometryType::COLLISION, _geom_mdl);

  _geom_mdl.addAllCollisionPairs();
  _geom_data = pin::GeometryData(_geom_mdl);

  _joint_J.assign(_mdl.njoints, Eigen::MatrixXd::Zero(6, _mdl.nv));
}

size_t CasadiCollisionHandler::Impl::numPairs() const {
  return _geom_mdl.collisionPairs.size();
}

bool CasadiCollisionHandler::Impl::distance(Eigen::Ref<const Eigen::VectorXd> q,
                                            Eigen::Ref<Eigen::VectorXd> d) {
  if (q.size() != _mdl.nq || d.size() != _geom_mdl.collisionPairs.size()) {
    std::cerr << __func__ << ": wrong input size: q.size()=" << q.size()
              << "!= " << _mdl.nq << ", d.size()=" << d.size()
              << "!= " << _geom_mdl.collisionPairs.size() << "\n";
    return false;
  }

  if (_last_q.size() == 0 || (_last_q.cwiseNotEqual(q)).any()) {

    // auto tic = std::chrono::high_resolution_clock::now();

    pin::computeDistances(_mdl, _data, _geom_mdl, _geom_data, q);

    _last_q = q;

    // auto toc = std::chrono::high_resolution_clock::now();

    // auto dur_sec = std::chrono::duration<double>(toc - tic);
  }

  for (size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k) {
    const auto &cp = _geom_mdl.collisionPairs[k];
    const auto &dr = _geom_data.distanceResults[k];
    auto name_1 = _geom_mdl.geometryObjects[cp.first].name;
    auto name_2 = _geom_mdl.geometryObjects[cp.second].name;

    d[k] = dr.min_distance;
  }

  return true;
}

bool CasadiCollisionHandler::Impl::distanceJacobian(
    Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<Eigen::MatrixXd> J) {
  if (q.size() != _mdl.nq || J.rows() != _geom_mdl.collisionPairs.size() ||
      J.cols() != _mdl.nv) {
    std::cerr << __func__ << ": wrong input size \n";
    return false;
  }

  _d_cached.setZero(numPairs());
  if (!distance(q, _d_cached)) {
    std::cerr << __func__ << ": distance computation failed \n";
    return false;
  }

  // auto tic = std::chrono::high_resolution_clock::now();

  pin::computeJointJacobians(_mdl, _data);

  for (size_t i = 0; i < _mdl.njoints; i++) {
    _joint_J[i].setZero(6, _mdl.nv);

    pin::getJointJacobian(
        _mdl, _data, i, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, _joint_J[i]);
  }

  Eigen::VectorXd Jrow_v(_mdl.nv);
  Eigen::VectorXd Jrow_q(_mdl.nq);

  for (size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k) {
    const auto &cp = _geom_mdl.collisionPairs[k];
    const auto &dr = _geom_data.distanceResults[k];

    auto &go_1 = _geom_mdl.geometryObjects[cp.first];
    auto &go_2 = _geom_mdl.geometryObjects[cp.second];

    auto name_1 = go_1.name;
    auto name_2 = go_2.name;

    size_t joint_1_id = _geom_mdl.geometryObjects[cp.first].parentJoint;
    size_t joint_2_id = _geom_mdl.geometryObjects[cp.second].parentJoint;

    if (joint_1_id > 0) {
      // joint 1 jacobian
      const auto &J_1 = _joint_J[joint_1_id];

      // witness point w.r.t. world
      Eigen::Vector3d w1 = dr.nearest_points[0];

      // translation
      Eigen::Vector3d r = w1 - _data.oMi[joint_1_id].translation();

      Jrow_v = -dr.normal.transpose() * J_1.topRows<3>();
      Jrow_v -= (r.cross(dr.normal)).transpose() * J_1.bottomRows<3>();
    } else {
      Jrow_v.setZero(_mdl.nv);
    }

    if (joint_2_id > 0) {
      // joint 2 jacobian
      const auto &J_2 = _joint_J[joint_2_id];

      // witness point w.r.t. world
      Eigen::Vector3d w2 = dr.nearest_points[1];

      // translation
      Eigen::Vector3d r = w2 - _data.oMi[joint_2_id].translation();

      Jrow_v += dr.normal.transpose() * J_2.topRows<3>();
      Jrow_v += (r.cross(dr.normal)).transpose() * J_2.bottomRows<3>();
    }

    kd->qdot(q, Jrow_v, Jrow_q);
    J.row(k) = Jrow_q;
  }

  // auto toc = std::chrono::high_resolution_clock::now();

  // auto dur_sec = std::chrono::duration<double>(toc - tic);

  return J.allFinite() && !J.hasNaN();

  // {
  //     std::cout << "bad values in distance jacobian: \n";
  //
  //     std::cout << "q = " << q.transpose().format(3) << "\n";
  //
  //     for (size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k)
  //     {
  //         if (!J.row(k).allFinite() || J.row(k).hasNaN())
  //         {
  //             std::cout << "at row " << k << ": " << J.row(k).format(3) <<
  //             "\n";
  //         }
  //     }
  // }
  //
  // return true;
}

CasadiCollisionHandler::Impl::~Impl() {}
