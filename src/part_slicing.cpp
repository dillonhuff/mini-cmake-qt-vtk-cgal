#include <vtkProperty.h>

#include "geometry/extrusion.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/surface_planning.h"
#include "process_planning/tool_access.h"
#include "system/parse_stl.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"

#include "part_slicing.h"

namespace gca {

  bool is_centralized(const std::vector<surface>& corner_group) {
    int num_surfaces_with_ortho_multiple_ortho_connections = 0;

    for (unsigned i = 0; i < corner_group.size(); i++) {
      int num_ortho_connections = 0;
      const surface& l = corner_group[i];
      for (unsigned j = 0; j < corner_group.size(); j++) {
	if (i != j) {
	  const surface& r = corner_group[j];
	  if (share_orthogonal_valley_edge(l, r)) {
	    num_ortho_connections++;
	  }
	}
      }

      if (num_ortho_connections > 1) {
	num_surfaces_with_ortho_multiple_ortho_connections++;
      }
    }

    return num_surfaces_with_ortho_multiple_ortho_connections <= 1;
  }

  std::vector<shared_edge>
  edges_to_fillet(const std::vector<surface>& cg,
		  const triangular_mesh& m,
		  const point dir) {
    vector<shared_edge> edges;

    for (auto& s : cg) {
      for (auto& other_s : cg ) {
	if (!s.contained_by(other_s)) {
	  auto new_edges = all_shared_edges(s.index_list(), other_s.index_list(), m);
	  delete_if(new_edges,
		    [m](const shared_edge e) {
		      return !is_valley_edge(e, m) || !angle_eps(e, m, 90, 0.5);
		    });

	  delete_if(new_edges,
		    [m, dir](const shared_edge e) {
		      point start = m.vertex(e.e.l);
		      point end = m.vertex(e.e.r);
		      point diff = end - start;
		      return !angle_eps(diff, dir, 180.0, 1.0) &&
			!angle_eps(diff, dir, 0.0, 1.0);
		    });

	  concat(edges, new_edges);
	}
      }
    }

    return edges;
    
  }

  bool all_corner_groups_millable(const std::vector<std::vector<surface> >& corner_groups) {
    for (auto& cg : corner_groups) {
      if (cg.size() > 2) {
	if (!is_centralized(cg)) {
	  return false;
	}
      }
    }
    return true;
  }

  std::vector<surface> find_access_surfaces(const std::vector<surface>& cg) {
    vector<point> norms;
    for (auto& s : cg) {
      norms.push_back(normal(s));
    }

    return find_access_surfaces(cg, norms);
  }

  vtkSmartPointer<vtkActor>
  actor_for_fillet(const triangular_mesh& m,
		   const std::vector<shared_edge>& edges) {

    vector<polyline> lines;
    for (auto s : edges) {
      point p1 = m.vertex(s.e.l);
      point p2 = m.vertex(s.e.r);
      lines.push_back(polyline({p1, p2}));
    }

    auto lines_pd = polydata_for_polylines(lines);
    color_polydata(lines_pd, 255, 0, 0);
    auto lines_act = polydata_actor(lines_pd);
    lines_act->GetProperty()->SetLineWidth(10);

    return lines_act;
  }

  void vtk_debug_shared_edges(const std::vector<shared_edge>& edges,
			      const triangular_mesh& m) {
    auto lines_act = actor_for_fillet(m, edges);

    auto mesh_pd = polydata_for_trimesh(m);
    color_polydata(mesh_pd, 255, 0, 0);
    auto mesh_act = polydata_actor(mesh_pd);
    mesh_act->GetProperty()->SetOpacity(0.4);

    visualize_actors({lines_act, mesh_act});
  }

  bool all_surfaces_are_millable_from(const point dir,
				      const std::vector<surface>& sfs) {
    if (sfs.size() == 0) { return true; }

    const triangular_mesh& m = sfs.front().get_parent_mesh();

    vector<index_t> all_inds;
    for (auto& s : sfs) {
      concat(all_inds, s.index_list());
    }
    all_inds = sort_unique(all_inds);

    auto millable_inds = millable_faces(dir, m);

    return intersection(millable_inds, all_inds).size() == all_inds.size();
  }
				      

  bool
  solveable_by_filleting(const triangular_mesh& m,
			 const std::vector<std::vector<surface> >& corner_groups) {
    vector<surface> sfs = outer_surfaces(m);
    DBG_ASSERT(sfs.size() > 0);
    vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    vector<point> dirs;
    for (auto& p : stock_planes) {
      dirs.push_back(p.normal());
      dirs.push_back(-1*p.normal());
    }

    for (auto cg : corner_groups) {

      for (auto access_dir : dirs) {

	if (!is_centralized(cg)) {

	  if (all_surfaces_are_millable_from(access_dir, cg)) {
	    vector<shared_edge> edges = edges_to_fillet(cg, m, access_dir);
	    vtk_debug_shared_edges(edges, m);
	  }
	}

      }
    }

    return true;
  }
  
  bool is_rectilinear(const triangular_mesh& m,
		      const std::vector<std::vector<surface> >& corner_groups) {


    if (all_corner_groups_millable(corner_groups)) {
      return true;
    }

    return false;
  }

  bool simplified_corners(const std::vector<std::vector<surface> >& corner_groups,
			  const std::vector<triangular_mesh>& pos_meshes) {
    int groups_after_cut = 0;

    for (auto& m : pos_meshes) {
      auto sfc = build_surface_milling_constraints(m);
      vector<vector<surface> > mcgs =
	sfc.hard_corner_groups();

      for (auto& cg : mcgs) {
	if (!is_centralized(cg)) {
	  groups_after_cut++;
	}
      }

    }

    int groups_before = 0;
    for (auto& cg : corner_groups) {
      if (!is_centralized(cg)) {
	groups_before++;
      }
    }

    cout << "Decentralized groups before cut = " << groups_before << endl;
    cout << "Decentralized groups after  cut = " << groups_after_cut << endl;

    return groups_after_cut < groups_before;
  }


  int count_planes(const std::vector<std::vector<surface> >& corner_groups) {
    int num_planes = 0;
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      if (!is_centralized(r)) {
	num_planes += r.size();
      }
    }

    return num_planes;
  }

  double distance(const polygon_3& l, const polygon_3& r) {
    double dist = 1e25;
    for (auto lpt : l.vertices()) {
      for (auto rpt : r.vertices()) {
	double d = (lpt - rpt).len();
	if (d < dist) {
	  dist = d;
	}
      }
    }

    return dist;
  }

  bool is_deep(const feature& f, const double depth_factor) {
    double area = base_area(f);
    double area_sq = sqrt(area);
    double depth = f.depth();

    return depth > 2.0*area_sq;
  }

  bool is_deep_external(const feature& f, const double depth_factor) {
    double depth = f.depth();

    polygon_3 base = f.base();
    auto base_holes = base.holes();

    if (base_holes.size() == 0) { return false; }

    for (unsigned i = 0; i < base_holes.size(); i++) {
      polygon_3 hi = build_clean_polygon_3(base_holes[i]);
      for (unsigned j = 0; j < base_holes.size(); j++) {
	if (i != j) {
	  polygon_3 hj = build_clean_polygon_3(base_holes[j]);

	  double d = distance(hi, hj);

	  if (depth > 2.0*d) {

	    // cout << "Depth = " << depth << endl;
	    // cout << "d     = " << d << endl;
	    // vtk_debug_polygons({hi, hj});


	    return true;
	  }
	}
      }
    }

    return false;
  }
  
  std::vector<feature*> check_deep_features(const triangular_mesh& m) {
    vector<surface> sfs = outer_surfaces(m);
    DBG_ASSERT(sfs.size() > 0);

    workpiece w(10, 10, 10, ALUMINUM);
    auto stock_mesh = align_workpiece(sfs, w);

    // vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    // vector<point> dirs;
    // for (auto& p : stock_planes) {
    //   dirs.push_back(p.normal());
    //   dirs.push_back(-1*p.normal());
    // }

    vector<surface> surfs = outer_surfaces(stock_mesh);

    DBG_ASSERT(surfs.size() == 6);

    vector<point> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
    }

    DBG_ASSERT(norms.size() == 6);

    vector<feature*> deep_features;
    
    for (auto& d : norms) {
      auto fd = build_min_feature_decomposition(stock_mesh, m, d);
      vector<feature*> deep_internal_features = collect_features(fd);
      delete_if(deep_internal_features,
		[](const feature* f) { return !(f->is_closed()) ||
		    !is_deep(*f, 5.0); });

      delete_if(deep_internal_features,
		[](const feature* f) {
		  auto diam = circle_diameter(f->base());
		  if (diam) { return true; }
		  return false;
		});

      //vtk_debug_features(deep_internal_features);

      concat(deep_features, deep_internal_features);

      vector<feature*> deep_external_features = collect_features(fd);
      delete_if(deep_external_features,
		[](const feature* f) { return f->is_closed() ||
		    !is_deep_external(*f, 5.0); });

      //vtk_debug_features(deep_internal_features);

      concat(deep_features, deep_external_features);

    }

    return deep_features;
  }

  part_split build_part_split(const triangular_mesh& m) {
    //auto fs = check_deep_features(m);
    return {trimesh_to_nef_polyhedron(m), {}}; //fs};
  }

  part_split build_part_split(const Nef_polyhedron& m) {
    // vector<feature*> fs;
    // for (auto& m : nef_polyhedron_to_trimeshes(m)) {
    //   concat(fs, check_deep_features(m));
    // }
    return {m, {}}; //fs};
  }

  int total_deep_features(const std::vector<part_split>& meshes) {
    int total = 0;

    for (auto& m : meshes) {
      total += m.deep_features.size();
    }

    return total;
  }
  
  void delete_duplicate_planes(std::vector<plane>& planes) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;
      for (unsigned i = 0; i < planes.size(); i++) {
	for (unsigned j = 0; j < planes.size(); j++) {
	  if (i != j) {
	    point in = planes[i].normal();
	    point jn = planes[j].normal();

	    if (angle_eps(in, jn, 0.0, 1.0)) {
	      point ipt = planes[i].pt();
	      point jpt = planes[j].pt();
	      point diff = ipt - jpt;

	      if (angle_eps(diff, in, 90.0, 1.0)) {
		planes.erase(begin(planes) + j);
		deleted_one = true;
		break;
	      }
	    }
	  }
	}

	if (deleted_one) {
	  break;
	}
      }
    }
  }

  std::vector<plane> possible_slice_planes(const triangular_mesh& m) {

    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    vector<plane> possible_slice_planes;
    
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      //      if (!is_centralized(r)) {
      for (auto& s : r) {
	plane p = surface_plane(s);
	possible_slice_planes.push_back(p);
      }
    }

    cout << "# slice planes before deleting = " << possible_slice_planes.size() << endl;

    delete_duplicate_planes(possible_slice_planes);

    return possible_slice_planes;
  }
  
  std::vector<std::vector<part_split> >
  split_away_deep_features(const part_split& part_nef) {

    cout << "Entering split away deep features" << endl;

    auto ms = nef_polyhedron_to_trimeshes(part_nef.nef);

    cout << "# of meshes = " << ms.size() << endl;

    if (ms.size() > 1) { return {}; }

    cout << "One mesh" << endl;

    auto m = ms.front();

    cout << "Now building surface constraints" << endl;
    
    auto sfc = build_surface_milling_constraints(m);
    vector<vector<surface> > corner_groups =
      sfc.hard_corner_groups();

    cout << "Just before computing deep features" << endl;

    int num_deep_features = part_nef.deep_features.size(); //check_deep_features(m).size();

    cout << "# of deep features in initial part = " << num_deep_features << endl;

    // This function should not be called unless the part
    // has some deep features
    DBG_ASSERT(num_deep_features > 0);

    vector<plane> possible_slice_planes;
    
    for (auto& r : corner_groups) {
      //vtk_debug_highlight_inds(r);

      //      if (!is_centralized(r)) {
      for (auto& s : r) {
	plane p = surface_plane(s);
	possible_slice_planes.push_back(p);
      }
    }

    cout << "# slice planes before deleting = " << possible_slice_planes.size() << endl;

    delete_duplicate_planes(possible_slice_planes);

    cout << "# of slice planes = " << possible_slice_planes.size() << endl;

    vector<vector<part_split> > productive_splits;
    for (auto p : possible_slice_planes) {
      auto clipped_nef_pos = clip_nef(part_nef.nef, p.slide(0.0001));
      auto clipped_nef_neg = clip_nef(part_nef.nef, p.flip().slide(0.0001));

      cout << "Clipped both" << endl;

      //auto clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);
      //vtk_debug_meshes(clipped_meshes);

      //clipped_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);

      //cout << "Negative meshes" << endl;
      //vtk_debug_meshes(clipped_meshes);

      vector<part_split> new_split;
      new_split.push_back(build_part_split(clipped_nef_pos));
      new_split.push_back(build_part_split(clipped_nef_neg));

      // vector<triangular_mesh> pos_meshes =
      // 	nef_polyhedron_to_trimeshes(clipped_nef_pos);
      // concat(pos_meshes, nef_polyhedron_to_trimeshes(clipped_nef_neg));

      cout << "Computing # of deep features" << endl;
      int next_deep_feats = new_split[0].deep_features.size() +
	new_split[1].deep_features.size(); //total_deep_features(pos_meshes);

      cout << "Computed # deep features = " << next_deep_feats << endl;

      if (next_deep_feats < num_deep_features) {
	cout << "Reduced the number of features, returning" << endl;
	//return {clipped_nef_pos, clipped_nef_neg};
	//vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_pos));
	//vtk_debug_meshes(nef_polyhedron_to_trimeshes(clipped_nef_neg));
	
	productive_splits.push_back(new_split);
      }

      cout << "Done with iteration" << endl;

    }
    //      }
    //    }

    return productive_splits;
  }

  bool deep_features_are_solved(const std::vector<part_split>& nefs) {
    for (auto& n : nefs) {
      //if (!deep_features_are_solved(n)) {
      if (n.deep_features.size() > 0) {
	return false;
      }
    }
    return true;
  }

  vector<vector<part_split> >
  splits_of_one_subpart(const std::vector<part_split>& nps) {
    vector<part_split> next_partial_solution = nps;
    auto f = find_if(begin(next_partial_solution),
		     end(next_partial_solution),
		     [](const part_split& n) {
		       return n.deep_features.size() > 0; //!deep_features_are_solved(n);
		     });

    DBG_ASSERT(f != end(next_partial_solution));

    cout << "TRYING TO SPLIT" << endl;
    //vtk_debug_meshes(nef_polyhedron_to_trimeshes(f->nef));

    vector<vector<part_split> > next = split_away_deep_features(*f);

    // Unsplittable part
    if (next.size() == 0) { return {}; }

    next_partial_solution.erase(f);

    for (auto& n : next) {
      cout << "# of deep features in split = " << total_deep_features(n) << endl;

      for (auto& split : n) {
	vtk_debug_meshes(nef_polyhedron_to_trimeshes(split.nef));
	vtk_debug_features(split.deep_features);
      }

      concat(n, next_partial_solution);
    }
    
    return next;
  }

  vector<vector<part_split> >
  solve_deep_features(const triangular_mesh& m) {

    // int num_deep_features = check_deep_features(m).size();
    // auto part_nef = trimesh_to_nef_polyhedron(m);

    // if (num_deep_features == 0) {
    //   return { {part_nef} };
    // }

    auto m_split = build_part_split(m);

    if (m_split.deep_features.size() == 0) { return {{m_split}}; }

    vector<vector<part_split> > parts{{m_split}};
    vector<vector<part_split> > solved;

    while (parts.size() > 0) {
      cout << "# of possible solutions left = " << parts.size() << endl;

      const auto& next_partial_solution = parts.back();

      vector<vector<part_split> > splits =
	splits_of_one_subpart(next_partial_solution);

      parts.pop_back();

      // Partial solution could be simplified
      if (splits.size() != 0) {

	for (auto& split : splits) {
	  if (deep_features_are_solved(split)) {
	    cout << "Found complete solution" << endl;
	    solved.push_back(split);
	  } else {
	    parts.push_back(split);
	  }
	}
      } else {
	cout << "Removed unusable solution" << endl;
      }

    }

    return solved;
  }

  polygon_3 build_2D_circle(const double radius) {
    // typedef boost::geometry::model::d2::point_xy<double> point;
    // typedef boost::geometry::model::polygon<point> polygon;

    // Declare the point_circle strategy
    boost::geometry::strategy::buffer::point_circle point_strategy(360);

    // Declare other strategies
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
    boost::geometry::strategy::buffer::join_round join_strategy;
    boost::geometry::strategy::buffer::end_round end_strategy;
    boost::geometry::strategy::buffer::side_straight side_strategy;

    // Declare/fill of a multi point
    boost::geometry::model::multi_point<boost_point_2> mp;
    boost::geometry::read_wkt("MULTIPOINT((0 0))", mp); //,(3 4),(4 4),(7 3))", mp);

    // Create the buffer of a multi point
    boost_multipoly_2 result;
    boost::geometry::buffer(mp, result,
			    distance_strategy, side_strategy,
			    join_strategy, end_strategy, point_strategy);

    DBG_ASSERT(result.size() == 1);

    boost_poly_2 front = result.front();

    return to_polygon_3(0, front);
  }

  polygon_3 build_3D_circle(const point center,
			    const double radius,
			    const point normal) {
    const rotation r = rotate_from_to(point(0, 0, 1), normal);

    polygon_3 circle = build_2D_circle(radius);
    circle = apply(r, circle);
    circle = shift(center, circle);

    return circle;
  }

  triangular_mesh build_hole_mesh(const point center,
				  const point normal,
				  const double depth,
				  const double radius) {
    polygon_3 circle = build_3D_circle(center, radius, normal);
    return extrude(circle, depth*normal);
  }

  void vtk_debug_nef(const Nef_polyhedron& n) {
    vtk_debug_meshes(nef_polyhedron_to_trimeshes(n));
  }

  bool is_coplanar(const plane p, const triangle t, const double tol) {
    point pt = t.v1;
    point diff = pt - p.pt();

    if (within_eps(diff.len(), 0.0, 0.001)) {
      diff = pt - t.v2;
    }

    DBG_ASSERT(!within_eps(diff.len(), 0.0, 0.001));

    if (angle_eps(p.normal(), diff, 90.0, tol) &&
	(angle_eps(normal(t), p.normal(), 0.0, tol) ||
	 angle_eps(normal(t), p.normal(), 180.0, tol))) {
      return true;
    }

    return false;
  }

  std::vector<index_t> coplanar_facets(const plane p,
				       const triangular_mesh& m) {
    vector<index_t> inds;
    for (auto& i : m.face_indexes()) {
      triangle t = m.face_triangle(i);
      //point pt = t.v1;
      //point diff = pt - p.pt();
      if (is_coplanar(p, t, 2.0)) {
	inds.push_back(i);
      }
      // NOTE: Tolerances are huge!
      // if (angle_eps(p.normal(), diff, 90.0, 2.0) &&
      // 	  (angle_eps(normal(t), p.normal(), 0.0, 2.0) ||
      // 	   angle_eps(normal(t), p.normal(), 180.0, 2.0))) {
      // 	inds.push_back(i);
      // }
    }
    return inds;
  }

  std::vector<surface> coplanar_surfaces(const plane p,
					 const triangular_mesh& m) {
    vector<index_t> pos_plane_tris =
      coplanar_facets(p, m);

    vector<vector<index_t> > inds =
      connect_regions(pos_plane_tris, m);

    vector<surface> surfs =
      inds_to_surfaces(inds, m);

    return surfs;
  }

std::vector<counterbore_params>
surface_hole_positions(const point dir, const surface& s) {
  vector<point> centroids;
  for (auto i : s.index_list()) {
    triangle t = s.face_triangle(i);
    centroids.push_back(t.centroid());
  }

  point c1 = centroids.front();
  point c2 = max_e(centroids, [c1](const point pt) {
      return (c1 - pt).len();
    });

  double offset = 0.01;
  return {{dir, c1, offset}, {dir, c2, offset}};
}

point
find_counterbore_side(const Nef_polyhedron& clipped_pos,
		      const Nef_polyhedron& clipped_neg,
		      const plane active_plane) {
  auto pos_meshes = nef_polyhedron_to_trimeshes(clipped_pos);
  double max_pos_diam = -1;
  for (auto& pos_mesh : pos_meshes) {
    double diam = diameter(active_plane.normal(), pos_mesh);
    if (diam > max_pos_diam) {
      max_pos_diam = diam;
    }
  }

  auto neg_meshes = nef_polyhedron_to_trimeshes(clipped_neg);
  double max_neg_diam = -1;
  for (auto& neg_mesh : neg_meshes) {
    double diam = diameter(active_plane.normal(), neg_mesh);
    if (diam > max_neg_diam) {
      max_neg_diam = diam;
    }
  }

  if (max_pos_diam > max_neg_diam) {
    return -1*active_plane.normal();
  }

  return active_plane.normal();
  
}

std::vector<counterbore_params>
match_polygons(const point counterbore_dir,
	       const std::vector<polygon_3>& pos_polys,
	       const std::vector<polygon_3>& neg_polys) {

  auto inters = polygon_intersection(pos_polys, neg_polys);

  double offset = 0.01;
  //vtk_debug_polygons(inters);

  vector<counterbore_params> ps;
  for (auto& poly : inters) {
    point pt = centroid(poly.vertices());
    ps.push_back({counterbore_dir, pt, offset});
  }
  return ps;
}

std::vector<counterbore_params>
hole_position(const Nef_polyhedron& clipped_pos,
	      const Nef_polyhedron& clipped_neg,
	      const plane active_plane) {
  point counterbore_dir =
    find_counterbore_side(clipped_pos, clipped_neg, active_plane);

  // cout << "COUNTERBORE MESH" << endl;
  // vtk_debug_nef(counterbore_mesh);

  auto pos_meshes = nef_polyhedron_to_trimeshes(clipped_pos);
  //auto neg_mesh = nef_to_single_trimesh(clipped_neg);

  // Q: How do you merge surfaces? Which surfaces need to
  // be joined by bolts?

  vector<polygon_3> pos_polys;
  for (auto& pos_mesh : pos_meshes) {
    vector<surface> surfs = coplanar_surfaces(active_plane, pos_mesh);

    for (auto& s : surfs) {
      cout << "POS SURFS" << endl;
      //vtk_debug_highlight_inds(s);

      vector<polygon_3> bound_polys = surface_boundary_polygons(s.index_list(),
								s.get_parent_mesh());
      DBG_ASSERT(bound_polys.size() == 1);
      pos_polys.push_back(bound_polys.front());
    }
  }

  auto neg_meshes = nef_polyhedron_to_trimeshes(clipped_neg);

  vector<polygon_3> neg_polys;
  for (auto& neg_mesh : neg_meshes) {
    
    vector<surface> surfs = coplanar_surfaces(active_plane, neg_mesh);
    for (auto& s : surfs) {
      cout << "NEG SURFS" << endl;
      //vtk_debug_highlight_inds(s);
      vector<polygon_3> bound_polys = surface_boundary_polygons(s.index_list(),
								s.get_parent_mesh());
      DBG_ASSERT(bound_polys.size() == 1);
      neg_polys.push_back(bound_polys.front());
    }
  }

  //vtk_debug_polygons(pos_polys);
  //vtk_debug_polygons(neg_polys);

  vector<counterbore_params> locs =
    match_polygons(counterbore_dir, pos_polys, neg_polys);

  return locs;

  // vector<counterbore_params> locs;
  
  // for (auto& pos_mesh : pos_meshes) {
  //   vector<surface> surfs = coplanar_surfaces(active_plane, pos_mesh);

  //   for (auto& s : surfs) {
  //     cout << "POSITIVE SURFACE" << endl;
  //     vtk_debug_highlight_inds(s);
  //     concat(locs, surface_hole_positions(counterbore_dir, s));
  //   }
  // }

  // vector<point> locs;
  // for (auto& s : surfs) {
  //   triangle t = s.face_triangle(s.front());
  //   locs.push_back(t.centroid());
  // }
  
  //return locs;

  // vector<surface> neg_surfs = coplanar_surfaces(active_plane, neg_mesh);

  // DBG_ASSERT(surfs.size() == 2);

  // for (auto& s : neg_surfs) {
  //   cout << "NEG SURFACE" << endl;
  //   vtk_debug_highlight_inds(s);
  // }
  
  //vtk_debug_highlight_inds(neg_plane_tris, neg_mesh);

  //return point(0, 0, 0);

  //DBG_ASSERT(false);
}

std::pair<Nef_polyhedron, Nef_polyhedron>
insert_attachment_holes(const Nef_polyhedron& clipped_pos,
			const Nef_polyhedron& clipped_neg,
			const plane active_plane) {
  vector<counterbore_params> positions =
    hole_position(clipped_pos, clipped_neg, active_plane);

  //vtk_debug_mesh(hole_mesh);

  Nef_polyhedron cp = clipped_pos;
  Nef_polyhedron cn = clipped_neg;

  double counter_diameter = 0.05;
  double hole_diameter = counter_diameter * (2.0 / 3.0);
  for (auto cb : positions) {
    triangular_mesh counterbore_mesh =
      build_hole_mesh(cb.counterbore_start(), cb.counter_dir, 10.0, counter_diameter);
    triangular_mesh hole_mesh =
      build_hole_mesh(cb.hole_start(), cb.counter_dir, 10.0, hole_diameter);

    auto cp_meshes = nef_polyhedron_to_trimeshes(cp);
    cp_meshes.push_back(counterbore_mesh);
    vtk_debug_meshes(cp_meshes);

    auto cn_meshes = nef_polyhedron_to_trimeshes(cn);
    cn_meshes.push_back(counterbore_mesh);
    vtk_debug_meshes(cn_meshes);
    
    auto counterbore_nef = trimesh_to_nef_polyhedron(counterbore_mesh);
    auto hole_nef = trimesh_to_nef_polyhedron(hole_mesh);
    cp = (cp - counterbore_nef) - hole_nef; //trimesh_to_nef_polyhedron(hole_mesh);
    cn = (cn - counterbore_nef) - hole_nef; //trimesh_to_nef_polyhedron(hole_mesh);

  }

  vtk_debug_nef(cp);
  vtk_debug_nef(cn);
  
  return make_pair(cp, cn);
}

sliced_part cut_part_with_plane(const plane active_plane,
				const Nef_polyhedron& part_nef) {
  auto clipped_pos = clip_nef(part_nef, active_plane.slide(0.0001));
  auto clipped_neg = clip_nef(part_nef, active_plane.flip().slide(0.0001));

  auto with_holes = insert_attachment_holes(clipped_pos, clipped_neg, active_plane);
  auto clipped_nef_pos = with_holes.first;
  auto clipped_nef_neg = with_holes.second;

  part_split pos_split = build_part_split(clipped_nef_pos);
  part_split neg_split = build_part_split(clipped_nef_neg);

  return sliced_part{pos_split, neg_split};
}
  
}
