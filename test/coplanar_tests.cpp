#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "part_slicing.h"

using namespace gca;

TEST_CASE("Slice CameraMount") {
  auto active_mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

  auto part_nef = trimesh_to_nef_polyhedron(active_mesh);

  plane p1(point(0, 0, -1), point(-0.25, -0.25, 0.1875));

  //vtk_debug(active_mesh, p1);

  plane p2(point(0, 0, 1), point(0.181102, -0.295276, 0.375));

  //auto clipped_pos = clip_nef(part_nef, p1.slide(0.0001));
  auto clipped_neg = clip_nef(part_nef, p1.flip().slide(0.0001));

  //vtk_debug_nef(clipped_pos);
  //vtk_debug_nef(clipped_neg);

  auto clipped_pos_2 = clip_nef(clipped_neg, p2.slide(0.0001));
  //auto clipped_neg_2 = clip_nef(clipped_neg, p2.flip().slide(0.0001));
  
  //vtk_debug_nef(clipped_pos_2);
  //vtk_debug_nef(clipped_neg_2);

  auto meshes = nef_polyhedron_to_trimeshes(clipped_pos_2);

  const auto& abnormal = meshes[0];
  auto ccs = const_orientation_regions(abnormal);
  for (auto cc : ccs) {
    if (cc.size() == 1) {
      cout << "SINGLE TRIANGLE GROUP!!" << endl;
      triangle t = abnormal.face_triangle(cc.front());
      cout << "t.v1   = " << t.v1 << endl;
      cout << "t.v2   = " << t.v2 << endl;
      cout << "t.v3   = " << t.v3 << endl;
      cout << "Normal = " << normal(t) << endl;
      vtk_debug_highlight_inds(cc, abnormal);
    }
  }

  vector<surface> surfs = coplanar_surfaces(p2, abnormal);
  for (auto s : surfs) {
    if (s.index_list().size() == 1) {
      cout << "ONE TRIANGLE COPLANAR!!" << endl;
      triangle t = abnormal.face_triangle(s.front());
      cout << "t.v1   = " << t.v1 << endl;
      cout << "t.v2   = " << t.v2 << endl;
      cout << "t.v3   = " << t.v3 << endl;
      cout << "Normal = " << normal(t) << endl;
      cout << "Coplanar = " << is_coplanar(p2, t, 2.0) << endl;

      vtk_debug_highlight_inds(s);

      for (auto& c : abnormal.face_face_neighbors(s.front())) {
	triangle t = abnormal.face_triangle(c);
	cout << "t.v1   = " << t.v1 << endl;
	cout << "t.v2   = " << t.v2 << endl;
	cout << "t.v3   = " << t.v3 << endl;
	cout << "Normal = " << normal(t) << endl;
	// NOTE: Adjust tolerance if changed in coplanar surfaces
	cout << "Coplanar = " << is_coplanar(p2, t, 2.0) << endl;

	point pt = t.v1;
	point diff = pt - p2.pt();
	cout << "Diff length = " << diff.len() << endl;

	cout << "diff = " << diff << endl;
	cout << "Angle between p2.normal() and diff = " << angle_between(p2.normal(), diff) << endl;
    // if (angle_eps(p.normal(), diff, 90.0, tol) &&
    // 	(angle_eps(normal(t), p.normal(), 0.0, tol) ||
    // 	 angle_eps(normal(t), p.normal(), 180.0, tol))) {
	
	vtk_debug_highlight_inds({c}, abnormal);
      }
    }

    
  }
}
