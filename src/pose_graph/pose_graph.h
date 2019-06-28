/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/


#ifndef ROBORTS_SLAM_POSE_GRAPH_POSE_GRAPH_H
#define ROBORTS_SLAM_POSE_GRAPH_POSE_GRAPH_H

#include "util/slam_util.h"
#include "util/transform.h"
#include "glog/logging.h"
#include <memory>
#include <queue>
#include <vector>
#include <set>

namespace roborts_slam {

template<typename T>
class Edge;

template<typename T>
class Vertex : public std::enable_shared_from_this<Vertex<T>> {
  friend class Edge<T>;
 public:
  using Ptr = std::shared_ptr<Vertex<T>>;
  using WeakPtr = std::weak_ptr<Vertex<T>>;
  using PtrVec = std::vector<Vertex<T>::Ptr>;
  using Vec = std::vector<Vertex<T>>;

  Vertex(std::shared_ptr<T> data_container_ptr) {
    data_container_ptr_ = data_container_ptr;
  }

  const std::vector<std::shared_ptr<Edge<T>>> &GetEdges() const {
    return edges_;
  }

  std::shared_ptr<T> GetDataContainer() const {
    return data_container_ptr_;
  }

  Vertex::PtrVec GetAdjacentVertices() const;

 private:

  void AddEdge(std::shared_ptr<Edge<T>> edge_ptr) {
    edges_.push_back(edge_ptr);
  }

  std::shared_ptr<T> data_container_ptr_;
  std::vector<std::shared_ptr<Edge<T>>> edges_;

};

class EdgeLinkInfo {
 public:
  using Ptr = std::shared_ptr<EdgeLinkInfo>;
 public:
  EdgeLinkInfo(const Pose2d &pose_1,
               const Pose2d &pose_2,
               const Matrix3d &covariance) {
    UpdateLinkInfo(pose_1, pose_2, covariance);
  };

  const Pose2d &GetPoseDifference() {
    return pose_diff_;
  };

  const Matrix3d &GetCovariance() {
    return covariance_;
  }

 private:
  void UpdateLinkInfo(const Pose2d &pose_1, const Pose2d &pose_2, const Matrix3d &covariance) {
    pose_1_ = pose_1;
    pose_2_ = pose_2;
//    Pose2d pose = Pose2d(0,0,0);
//    Transform2d transform(pose_1, pose);
//    pose_diff_ = transform.Transform(pose_2);

    roborts_slam::TransformByMidFrame transform(pose_1_, pose_2_);
    pose_diff_ = transform.Transform(Pose2d(0, 0, 0));

    if (std::abs(pose_diff_(0)) > 1 || std::abs(pose_diff_(1)) > 1) {
      DLOG(WARNING) << "pose_1: " << pose_1_.transpose();
      DLOG(WARNING) << "pose_2: " << pose_2_.transpose();
      DLOG(WARNING) << "pose_d: " << pose_diff_.transpose();
    }

    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(-pose_1(2), Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    covariance_ = rotation_matrix * covariance * rotation_matrix.transpose();
  };

 private:
  Pose2d pose_1_;
  Pose2d pose_2_;
  Pose2d pose_diff_;
  Matrix3d covariance_;
};

template<typename T>
class Edge : public std::enable_shared_from_this<Edge<T>> {
 public:
  using Ptr = std::shared_ptr<Edge>;
  using PtrVec = std::vector<std::shared_ptr<Edge>>;

  Edge() = default;

  void AddVertex(std::shared_ptr<Vertex<T>> source_vertex_ptr,
                 std::shared_ptr<Vertex<T>> target_vertex_ptr) {
    source_vertex_ptr_ = source_vertex_ptr;
    target_vertex_ptr_ = target_vertex_ptr;
    source_vertex_ptr_->AddEdge(std::enable_shared_from_this<Edge<T>>::shared_from_this());
    target_vertex_ptr_->AddEdge(std::enable_shared_from_this<Edge<T>>::shared_from_this());
  };

 public:
  std::shared_ptr<Vertex<T>> GetSource() const {
    return source_vertex_ptr_;
  }

  std::shared_ptr<Vertex<T>> GetTarget() const {
    return target_vertex_ptr_;
  }

  EdgeLinkInfo::Ptr GetLinkInfo() {
    return link_info_;
  }

  void SetLinkInfo(EdgeLinkInfo::Ptr edge_link_info) {
    link_info_ = edge_link_info;
  }

 private:
  typename Vertex<T>::Ptr source_vertex_ptr_;
  typename Vertex<T>::Ptr target_vertex_ptr_;
  EdgeLinkInfo::Ptr link_info_;
};

template<typename T>
std::vector<std::shared_ptr<Vertex<T>>> Vertex<T>::GetAdjacentVertices() const {
  Vertex::PtrVec vertices;

  for (const auto &it : edges_) {

    // check both source and target because we have a undirected graph
    if (it->GetSource() != std::enable_shared_from_this<Vertex<T>>::shared_from_this()) {
      CHECK_NE(it->GetSource()->GetDataContainer()->id(), this->GetDataContainer()->id());
      vertices.push_back(it->GetSource());
    }

    if (it->GetTarget() != std::enable_shared_from_this<Vertex<T>>::shared_from_this()) {
      CHECK_NE(it->GetTarget()->GetDataContainer()->id(), this->GetDataContainer()->id());
      vertices.push_back(it->GetTarget());
    }

  }

  return vertices;
}

template<typename T>
class PoseGraphSolver;

template<typename T>
class PoseGraph {
 public:
  PoseGraph() = default;

  virtual void AddVertex(typename Vertex<T>::Ptr vertex_ptr) {
    vertex_vec_.push_back(vertex_ptr);
    if (solver_ptr_ != nullptr) {
      solver_ptr_->AddNode(vertex_ptr);
    };
  };

  virtual void AddEdge(typename Edge<T>::Ptr edge_ptr) {
    edge_vec_.push_back(edge_ptr);
  };

  virtual const typename Vertex<T>::PtrVec &GetVertexVec() const {
    return vertex_vec_;
  }

  virtual const typename Edge<T>::PtrVec &GetEdgeVec() const {
    return edge_vec_;
  }

 protected:
  const typename Vertex<T>::Ptr &GetVertex(const int &data_container_id){
    return vertex_vec_.at(data_container_id);
  };


  using TraverseVisitorFunc = std::function<bool(typename Vertex<T>::Ptr)>;
  std::vector<std::shared_ptr<T>> BreadthFirstTraverse(typename Vertex<T>::Ptr start_vertex_ptr,
                                                       TraverseVisitorFunc visitor);

 protected:
  typename Vertex<T>::PtrVec vertex_vec_;
  typename Edge<T>::PtrVec edge_vec_;
  typename PoseGraphSolver<T>::Ptr solver_ptr_;

 private:
  TraverseVisitorFunc traverse_visitor_;

};

template<typename T>
std::vector<std::shared_ptr<T>> PoseGraph<T>::BreadthFirstTraverse(typename Vertex<T>::Ptr start_vertex_ptr,
                                                                   TraverseVisitorFunc visitor) {

  std::queue<typename Vertex<T>::Ptr> vertices_to_visit;
  std::set<typename Vertex<T>::Ptr> verteices_visited;
  std::vector<typename Vertex<T>::Ptr> valid_vertices;

  vertices_to_visit.push(start_vertex_ptr);
  verteices_visited.insert(start_vertex_ptr);

  do {
    typename Vertex<T>::Ptr next_ptr = vertices_to_visit.front();
    vertices_to_visit.pop();

    if (visitor(next_ptr)) {
      // vertex is valid, explore neighbors
      valid_vertices.push_back(next_ptr);

      std::vector<typename Vertex<T>::Ptr> adjacent_vertices = next_ptr->GetAdjacentVertices();

//      Debug for range scan pose graph
//      std::stringstream adjacent_vertices_id;
//      adjacent_vertices_id << "ID <" << next_ptr->GetDataContainer()->id();
//      adjacent_vertices_id << "> -> Adjacent Vertices: ";
//      for (auto it : adjacent_vertices) {
//        adjacent_vertices_id << " " << it->GetDataContainer()->id();
//      }
//      LOG(INFO) << adjacent_vertices_id.str();

      for (auto it : adjacent_vertices) {
        typename Vertex<T>::Ptr adjacent = it;
        if (verteices_visited.find(adjacent) == verteices_visited.end()) {
          vertices_to_visit.push(adjacent);
          verteices_visited.insert(adjacent);
        }
      }
    }
  } while (!vertices_to_visit.empty());

  std::vector<std::shared_ptr<T>> data_container_ptr_vec;
  for (typename Vertex<T>::Ptr &it : valid_vertices) {
    data_container_ptr_vec.push_back((it->GetDataContainer()));
  }
  return data_container_ptr_vec;
}

template<typename T>
class PoseGraphSolver {
 public:
  using Ptr = std::shared_ptr<PoseGraphSolver>;
  /**
   * Vector of id-pose pairs
   */
  typedef std::vector<std::pair<int, Pose2d> > Pose2DWithIdVector;

  PoseGraphSolver() {
  };

  virtual ~PoseGraphSolver() {
  };

 public:

  virtual void Compute() = 0;

  virtual const Pose2DWithIdVector &GetCorrections() const = 0;

  virtual void AddNode(typename Vertex<T>::Ptr vertex_ptr) = 0;

  virtual void RemoveNode(int id) {};

  virtual void AddConstraint(typename Edge<T>::Ptr edge_ptr) = 0;

  virtual void RemoveConstraint(int source_id, int target_id) {};

  virtual void GetGraph(std::vector<Eigen::Vector2d> &nodes,
                        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges) = 0;

  virtual void Clear() {};
};

} // namespace roborts_slam

#endif // ROBORTS_SLAM_POSE_GRAPH_POSE_GRAPH_H
