#pragma once

#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/surface_planning.h"
#include "process_planning/tool_access.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"

namespace gca {

  struct part_search_result {
    Nef_polyhedron part_nef;
  };

  bool is_centralized(const std::vector<surface>& corner_group);

  std::vector<shared_edge>
  edges_to_fillet(const std::vector<surface>& cg,
  		  const triangular_mesh& m,
  		  const point dir);

  bool all_corner_groups_millable(const std::vector<std::vector<surface> >& corner_groups);
  
  void vtk_debug_shared_edges(const std::vector<shared_edge>& edges,
  			      const triangular_mesh& m);
  bool
  solveable_by_filleting(const triangular_mesh& m,
  			 const std::vector<std::vector<surface> >& corner_groups);

  bool is_rectilinear(const triangular_mesh& m,
  		      const std::vector<std::vector<surface> >& corner_groups);

  bool simplified_corners(const std::vector<std::vector<surface> >& corner_groups,
  			  const std::vector<triangular_mesh>& pos_meshes);

  int count_planes(const std::vector<std::vector<surface> >& corner_groups);

  double distance(const polygon_3& l, const polygon_3& r);

  bool is_deep(const feature& f, const double depth_factor);

  bool is_deep_external(const feature& f, const double depth_factor);
  
  std::vector<feature*> check_deep_features(const triangular_mesh& m);

  struct part_split {
    Nef_polyhedron nef;
    std::vector<feature*> deep_features;
  };

  part_split build_part_split(const triangular_mesh& m);

  part_split build_part_split(const Nef_polyhedron& m);

  int total_deep_features(const std::vector<part_split>& meshes);
  
  void delete_duplicate_planes(std::vector<plane>& planes);

  std::vector<plane> possible_slice_planes(const triangular_mesh& m);

  std::vector<std::vector<part_split> >
  split_away_deep_features(const part_split& part_nef);

  bool deep_features_are_solved(const std::vector<part_split>& nefs);

  vector<vector<part_split> >
  splits_of_one_subpart(const std::vector<part_split>& nps);

  bool all_surfaces_are_millable_from(const point dir,
				      const std::vector<surface>& sfs);
  

  vtkSmartPointer<vtkActor>
  actor_for_fillet(const triangular_mesh& m,
		   const std::vector<shared_edge>& edges);
  
}
