/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef SOLVERS__CERES_SOLVER_HPP_
#define SOLVERS__CERES_SOLVER_HPP_

#include <math.h>
#include <ceres/ceres.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <memory>
#include <mutex>
#include "karto_sdk/Mapper.h"
#include "ceres_utils.h"

namespace solver_plugins
{

enum class LinearSolver
{
  SPARSE_NORMAL_CHOLESKY = ceres::SPARSE_NORMAL_CHOLESKY,
  SPARSE_SCHUR = ceres::SPARSE_SCHUR,
  ITERATIVE_SCHUR = ceres::ITERATIVE_SCHUR,
  CGNR = ceres::CGNR,
};

enum class Preconditioner
{
  JACOBI = ceres::JACOBI,
  IDENTITY = ceres::IDENTITY,
  SCHUR_JACOBI = ceres::SCHUR_JACOBI,
};

enum class TrustStrategy
{
  LEVENBERG_MARQUARDT = ceres::LEVENBERG_MARQUARDT,
  DOGLEG = ceres::DOGLEG,
};

enum class DoglegType
{
  TRADITIONAL_DOGLEG = ceres::TRADITIONAL_DOGLEG,
  SUBSPACE_DOGLEG = ceres::SUBSPACE_DOGLEG,
};

enum class LossFunction
{
  None,
  HuberLoss,
  CauchyLoss,
};

struct CeresSolverConfig
{
  LinearSolver linear_solver = LinearSolver::SPARSE_NORMAL_CHOLESKY;
  Preconditioner preconditioner = Preconditioner::JACOBI;
  DoglegType dogleg_type = DoglegType::TRADITIONAL_DOGLEG;
  TrustStrategy trust_strategy = TrustStrategy::LEVENBERG_MARQUARDT;
  LossFunction loss_function = LossFunction::None;
  bool localization = false;
  bool debug_logging = false;
  int num_threads = 1;
};

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  // Get corrected poses after optimization
  virtual const karto::ScanSolver::IdPoseVector & GetCorrections() const;

  virtual void Compute();  // Solve
  virtual void Clear();  // Resets the corrections
  virtual void Reset();  // Resets the solver plugin clean

  void Configure(const CeresSolverConfig & config = {});

  // Adds a node to the solver
  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex);
  // Adds a constraint to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge);
  // Get graph stored
  virtual std::unordered_map<int, Eigen::Vector3d> * getGraph();
  // Removes a node from the solver correction table
  virtual void RemoveNode(kt_int32s id);
  // Removes constraints from the optimization problem
  virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId);

  // change a node's pose
  virtual void ModifyNode(const int & unique_id, Eigen::Vector3d pose);
  // get a node's current pose yaw
  virtual void GetNodeOrientation(const int & unique_id, double & pose);

private:
  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // ceres
  ceres::Solver::Options options_;
  ceres::Problem::Options options_problem_;
  ceres::LossFunction * loss_function_;
  ceres::Problem * problem_;
  ceres::Manifold * angle_manifold_;
  bool was_constant_set_, debug_logging_;

  // graph
  std::unordered_map<int, Eigen::Vector3d> * nodes_;
  std::unordered_map<size_t, ceres::ResidualBlockId> * blocks_;
  std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
  std::mutex nodes_mutex_;

};

}  // namespace solver_plugins

#endif  // SOLVERS__CERES_SOLVER_HPP_
