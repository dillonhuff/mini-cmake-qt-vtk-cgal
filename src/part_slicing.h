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

  // bool is_centralized(const std::vector<surface>& corner_group);

  // std::vector<shared_edge>
  // edges_to_fillet(const std::vector<surface>& cg,
  // 		  const triangular_mesh& m,
  // 		  const point dir);

  // bool all_corner_groups_millable(const std::vector<std::vector<surface> >& corner_groups);
  
  // void vtk_debug_shared_edges(const std::vector<shared_edge>& edges,
  // 			      const triangular_mesh& m);
  // bool
  // solveable_by_filleting(const triangular_mesh& m,
  // 			 const std::vector<std::vector<surface> >& corner_groups);

  // bool is_rectilinear(const triangular_mesh& m,
  // 		      const std::vector<std::vector<surface> >& corner_groups);

  // bool simplified_corners(const std::vector<std::vector<surface> >& corner_groups,
  // 			  const std::vector<triangular_mesh>& pos_meshes);

  // int count_planes(const std::vector<std::vector<surface> >& corner_groups);

  // double distance(const polygon_3& l, const polygon_3& r);

  // bool is_deep(const feature& f, const double depth_factor);

  // bool is_deep_external(const feature& f, const double depth_factor);
  
  // std::vector<feature*> check_deep_features(const triangular_mesh& m);

  struct part_split {
    Nef_polyhedron nef;
    std::vector<feature*> deep_features;
  };

  // part_split build_part_split(const triangular_mesh& m);

  // part_split build_part_split(const Nef_polyhedron& m);

  // int total_deep_features(const std::vector<part_split>& meshes);
  
  // void delete_duplicate_planes(std::vector<plane>& planes);
  
//   std::vector<std::vector<part_split> >
//   split_away_deep_features(const part_split& part_nef) {

//     cout << "Entering split away deep features" << endl;

//     auto ms = nef_polyhedron_to_trimeshes(part_nef.nef);

//     cout << "# of meshes = " << ms.size() << endl;

//     if (ms.size() > 1) { return {}; }

//     cout << "One mesh" << endl;

//     auto m = ms.front();

//     cout << "Now building surface constraints" << endl;
    
//     auto sfc = build_surface_milling_constraints(m);
//     vector<vector<surface> > corner_groups =
//       sfc.hard_corner_groups();

//     cout << "Just before computing deep features" << endl;

//     int num_deep_features = part_nef.deep_features.size(); //check_deep_features(m).size();

//     cout << "# of deep features in initial part = " << num_deep_features << endl;

//     // This function should not be called unless the part
//     // has some deep features
//     DBG_ASSERT(num_deep_features > 0);

//     vector<plane> possible_slice_planes;
    
//     for (auto& r : corner_groups) {
//       //vtk_debug_highlight_inds(r);

//       //      if (!is_centralized(r)) {
// 	for (auto& s : r) {
// 	  plane p = surface_plane(s);
// 	  possible_slice_planes.push_back(p);
// 	}
//     }

//     cout << "# slice planes before deleting = " << possible_slice_planes.size() << endl;

//     delete_duplicate_plans(possible_slice_planes);

//     cout << "# of slice planes = " << possible_slice_planes.size() << endl;

//     vector<vector<part_split> > productive_splits;
//     for (auto p : possible_slice_planes) {
//       auto clipped_nef_pos = clip_nef(part_nef.nef, p.slide(0.0001));
//       auto clipped_nef_neg = clip_nef(part_nef.nef, p.flip().slide(0.0001));

//       cout << "Clipped both" << endl;

//       //auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
//       //vtk_debug_meshes(clipped_meshes);

//       //clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);

//       //cout << "Negative meshes" << endl;
//       //vtk_debug_meshes(clipped_meshes);

//       vector<part_split> new_split;
//       new_split.push_back(build_part_split(clipped_nef_pos));
//       new_split.push_back(build_part_split(clipped_nef_neg));

//       // vector<triangular_mesh> pos_meshes =
//       // 	nef_polyhedron_to_trimeshes(clipped_nef_pos);
//       // concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

//       cout << "Computing # of deep features" << endl;
//       int next_deep_feats = new_split[0].deep_features.size() +
// 	new_split[1].deep_features.size(); //total_deep_features(pos_meshes);

//       cout << "Computed # deep features = " << next_deep_feats << endl;

//       if (next_deep_feats < num_deep_features) {
// 	cout << "Reduced the number of features, returning" << endl;
// 	//return {clipped_nef_pos, clipped_nef_neg};
// 	//vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_pos));
// 	//vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_neg));
	
// 	productive_splits.push_back(new_split);
//       }

//       cout << "Done with iteration" << endl;

//     }
// 	//      }
//     //    }

//     return productive_splits;
//   }

//   bool deep_features_are_solved(const std::vector<part_split>& nefs) {
//     for (auto& n : nefs) {
//       //if (!deep_features_are_solved(n)) {
//       if (n.deep_features.size() > 0) {
// 	return false;
//       }
//     }
//     return true;
//   }

//   vector<vector<part_split> >
//   splits_of_one_subpart(const std::vector<part_split>& nps) {
//     vector<part_split> next_partial_solution = nps;
//     auto f = find_if(begin(next_partial_solution),
// 		     end(next_partial_solution),
// 		     [](const part_split& n) {
// 		       return n.deep_features.size() > 0; //!deep_features_are_solved(n);
// 		     });

//     DBG_ASSERT(f != end(next_partial_solution));

//     cout << "TRYING TO SPLIT" << endl;
//     //vtk_debug_meshes(nef_polyhedron_to_trimeshes(f->nef));

//     vector<vector<part_split> > next = split_away_deep_features(*f);

//     // Unsplittable part
//     if (next.size() == 0) { return {}; }

//     next_partial_solution.erase(f);

//     for (auto& n : next) {
//       cout << "# of deep features in split = " << total_deep_features(n) << endl;

//       for (auto& split : n) {
// 	vtk_debug_meshes(nef_polyhedron_to_trimeshes(split.nef));
// 	vtk_debug_features(split.deep_features);
//       }

//       concat(n, next_partial_solution);
//     }
    
//     return next;
//   }

//   vector<vector<part_split> >
//   solve_deep_features(const triangular_mesh& m) {

//     // int num_deep_features = check_deep_features(m).size();
//     // auto part_nef = trimesh_to_nef_polyhedron(m);

//     // if (num_deep_features == 0) {
//     //   return { {part_nef} };
//     // }

//     auto m_split = build_part_split(m);

//     if (m_split.deep_features.size() == 0) { return {{m_split}}; }

//     vector<vector<part_split> > parts{{m_split}};
//     vector<vector<part_split> > solved;

//     while (parts.size() > 0) {
//       cout << "# of possible solutions left = " << parts.size() << endl;

//       const auto& next_partial_solution = parts.back();

//       vector<vector<part_split> > splits =
// 	splits_of_one_subpart(next_partial_solution);

//       parts.pop_back();

//       // Partial solution could be simplified
//       if (splits.size() != 0) {

// 	for (auto& split : splits) {
// 	  if (deep_features_are_solved(split)) {
// 	    cout << "Found complete solution" << endl;
// 	    solved.push_back(split);
// 	  } else {
// 	    parts.push_back(split);
// 	  }
// 	}
//       } else {
// 	cout << "Removed unusable solution" << endl;
//       }

//     }

//     return solved;
//   }

}
