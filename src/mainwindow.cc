#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "geometry/extrusion.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QHBoxLayout>

using namespace gca;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
{

  current_mode = SLICE_MODE;

  vtk_window = new QVTKWidget(this, Qt::Widget);

  QHBoxLayout *layout = new QHBoxLayout;

  QPushButton* button2 = new QPushButton("Load STL");
  accept_button = new QPushButton("Accept slice", this);
  reject_button = new QPushButton("Reject slice", this);
  set_done_button = new QPushButton("Done with part", this);

  QVBoxLayout* update_buttons = new QVBoxLayout();
  update_buttons->addWidget(button2);
  update_buttons->addWidget(accept_button);
  update_buttons->addWidget(reject_button);
  update_buttons->addWidget(set_done_button);

  in_progress_heading = new QLabel();
  in_progress_heading->setText("In Progress");

  QVBoxLayout* in_progress = new QVBoxLayout();
  in_progress->addWidget(in_progress_heading);

  QLabel* completed_heading = new QLabel();
  completed_heading->setText("Completed");

  QVBoxLayout* completed = new QVBoxLayout();
  completed->addWidget(completed_heading);

  layout->addLayout(update_buttons);
  layout->addWidget(vtk_window);
  layout->addLayout(in_progress);
  layout->addLayout(completed);

  setCentralWidget(new QWidget);
  centralWidget()->setLayout(layout);

  connect(accept_button, SIGNAL (released()), this, SLOT (handle_accept()));
  connect(reject_button, SIGNAL (released()), this, SLOT (handle_reject()));
  connect(set_done_button, SIGNAL (released()), this, SLOT (handle_set_done()));

  // active_mesh =
  //   parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/artusitestp1 - Part 1.stl", 0.0001);
  
  // active_mesh =
  //   parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/caliperbedlevelingi3v2_fixed - Part 1.stl", 0.0001);

  active_mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

  slice_planes = possible_slice_planes(active_mesh);

  DBG_ASSERT(slice_planes.size() > 0);

  active_plane = slice_planes.back();
  slice_planes.pop_back();

  active_plane_actor = plane_actor(vtk_plane(active_plane));

  active_mesh_polydata = polydata_for_trimesh(active_mesh);
  color_polydata(active_mesh_polydata, 0, 255, 0);

  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(active_mesh_polydata);

  active_mesh_actor = 
    vtkSmartPointer<vtkActor>::New();
  active_mesh_actor->SetMapper(mapper);

  renderer = vtkSmartPointer<vtkRenderer>::New();

  renderer->SetBackground(1, 1, 1);
  renderer->AddActor(active_mesh_actor);
  renderer->AddActor(active_plane_actor);

  vtk_window->GetRenderWindow()->AddRenderer(renderer);

  vtk_window->show();

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

std::vector<index_t> coplanar_facets(const plane p,
				     const triangular_mesh& m) {
  vector<index_t> inds;
  for (auto& i : m.face_indexes()) {
    triangle t = m.face_triangle(i);
    point pt = t.v1;
    point diff = pt - p.pt();
    // NOTE: Tolerances are huge!
    if (angle_eps(p.normal(), diff, 90.0, 10.0) &&
	(angle_eps(normal(t), p.normal(), 0.0, 10.0) ||
	 angle_eps(normal(t), p.normal(), 180.0, 10.0))) {
      inds.push_back(i);
    }
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

struct counterbore_params {
  point counter_dir;
  point position;
  double counterbore_offset;

  point counterbore_start() const {
    return position + counterbore_offset*counter_dir;
  }

  point hole_start() const {
    return position - 5*counter_dir;
  }
};

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
  vtk_debug_polygons(inters);

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
      vtk_debug_highlight_inds(s);

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
      vtk_debug_highlight_inds(s);
      vector<polygon_3> bound_polys = surface_boundary_polygons(s.index_list(),
								s.get_parent_mesh());
      DBG_ASSERT(bound_polys.size() == 1);
      neg_polys.push_back(bound_polys.front());
    }
  }

  vtk_debug_polygons(pos_polys);
  vtk_debug_polygons(neg_polys);

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

    auto counterbore_nef = trimesh_to_nef_polyhedron(counterbore_mesh);
    auto hole_nef = trimesh_to_nef_polyhedron(hole_mesh);
    cp = (cp - counterbore_nef) - hole_nef; //trimesh_to_nef_polyhedron(hole_mesh);
    cn = (cn - counterbore_nef) - hole_nef; //trimesh_to_nef_polyhedron(hole_mesh);
  }

  // vtk_debug_nef(cp);
  // vtk_debug_nef(cn);
  
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

void MainWindow::handle_accept_slice() {
  auto part_nef = trimesh_to_nef_polyhedron(active_mesh);

  sliced_part sliced = cut_part_with_plane(active_plane, part_nef);

  part_split pos_split = sliced.pos_split;
  part_split neg_split = sliced.neg_split;

  add_to_queues(pos_split);
  add_to_queues(neg_split);
  
  slice_next_part();
}

void MainWindow::handle_reject_slice() {
  if (slice_planes.size() == 0) {
    in_progress_heading->setText("ERROR: No further slice planes!");
    return;
  } 

  active_plane = slice_planes.back();
  slice_planes.pop_back();

  update_active_plane(active_plane);

}

std::string to_string(const point pt) {
  return "(" + std::to_string(pt.x) + ", " + std::to_string(pt.y) + ", " + std::to_string(pt.z) + ")";
}

std::string to_string(const plane& p) {
  return "Normal = " + to_string(p.normal()) + "\nPt = " + to_string(p.pt());
}

void MainWindow::update_active_plane(const gca::plane p) {
  in_progress_heading->setText(to_string(p).c_str());
  renderer->RemoveActor(active_plane_actor);
  active_plane_actor = plane_actor(vtk_plane(p));
  renderer->AddActor(active_plane_actor);
  vtk_window->update();
}

void MainWindow::clear_active_plane() {
  renderer->RemoveActor(active_plane_actor);
}

void MainWindow::update_active_mesh(const gca::triangular_mesh& new_mesh) {

  active_mesh = new_mesh;

  slice_planes = possible_slice_planes(active_mesh);

  if (slice_planes.size() > 0) {
    active_plane = slice_planes.back();
    slice_planes.pop_back();

    update_active_plane(active_plane);
  } else {
    slice_planes = {};
    clear_active_plane();
  }

  active_mesh_polydata = polydata_for_trimesh(active_mesh);
  color_polydata(active_mesh_polydata, 0, 255, 0);

  renderer->RemoveActor(active_mesh_actor);
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(active_mesh_polydata);

  active_mesh_actor = 
    vtkSmartPointer<vtkActor>::New();
  active_mesh_actor->SetMapper(mapper);

  renderer->AddActor(active_mesh_actor);

  vtk_window->update();

}

void MainWindow::handle_set_done_slice() {

  auto part_nef = trimesh_to_nef_polyhedron(active_mesh);
  part_split p_split = build_part_split(part_nef);
  add_to_filletables(p_split);    
  
  if (in_progress.size() == 0) {
    switch_to_fillet_mode();
    fillet_next_part();
    return;
  }

  // auto next_mesh = nef_to_single_trimesh(in_progress.back().nef);
  // in_progress.pop_back();

  slice_next_part();
  //update_active_mesh(next_mesh);
}

void MainWindow::slice_next_part() {

  if (in_progress.size() == 0) {
    in_progress_heading->setText("All parts are finished");
    switch_to_fillet_mode();
    fillet_next_part();
    return;
  }

  part_split next_part = in_progress.back();
  in_progress.pop_back();

  auto mesh = nef_to_single_trimesh(next_part.nef);

  update_active_mesh(mesh);

}

template<typename T>
void delete_other_inds(std::vector<T>& elems, int ind) {
  T elem = elems[ind];
  elems.clear();
  elems.push_back(elem);
}

void MainWindow::handle_accept_fillet() {
  fillet_group& current_group =
    active_fillet_part.part.fillet_groups[active_fillet_part.fillet_group_index];

  delete_other_inds(current_group.possible_fillets, active_fillet_part.fillet_index);

  clear_active_fillet();

  fillet_next();
}

void MainWindow::handle_reject_fillet() {
  fillet_group& current_group =
    active_fillet_part.part.fillet_groups[active_fillet_part.fillet_group_index];

  current_group.possible_fillets.erase(begin(current_group.possible_fillets) + active_fillet_part.fillet_index);

  clear_active_fillet();

  fillet_next();
}

void MainWindow::handle_set_done_fillet() {
}

void MainWindow::handle_accept() {
  if (current_mode == SLICE_MODE) {
    handle_accept_slice();
  } else if (current_mode == FILLET_MODE) {
    handle_accept_fillet();
  } else {
    
  }

}

void MainWindow::handle_reject() {
  if (current_mode == SLICE_MODE) {
    handle_reject_slice();
  } else if (current_mode == FILLET_MODE) {
    handle_reject_fillet();
  } else {
    return;
  }
  
}

void MainWindow::handle_set_done() {
  if (current_mode == SLICE_MODE) {
    handle_set_done_slice();
  } else {
    DBG_ASSERT(false);
  }

}

void MainWindow::switch_to_fillet_mode() {
  current_mode = FILLET_MODE;
  clear_active_plane();
  accept_button->setText("Accept fillet");
  reject_button->setText("Reject fillet");
}

void MainWindow::clear_active_fillet() {
  renderer->RemoveActor(active_fillet_actor);
}

void MainWindow::clear_active_mesh() {
  renderer->RemoveActor(active_mesh_actor);
}

void MainWindow::set_active_fillet(const active_fillet& af) {

  update_active_mesh(af.part.part);
  clear_active_plane();

  clear_active_fillet();

  active_fillet_part = af;

  active_fillet_actor = actor_for_fillet(active_fillet_part.part.part,
					 active_fillet_part.current_fillet());

  renderer->AddActor(active_fillet_actor);
}

std::pair<int, int> find_next_fillet_choice(const active_fillet& af) {
  int fillet_group;
  for (fillet_group = 0;
       fillet_group < af.part.fillet_groups.size();
       fillet_group++) {

    int num_possible_fillets =
      af.part.fillet_groups[fillet_group].possible_fillets.size();
    if (num_possible_fillets > 1) {
      return std::make_pair(fillet_group, 0);
    }

  }


  return std::make_pair(-1, -1);
}

active_fillet MainWindow::next_active_fillet() {
  filletable_part next = in_progress_fillets.back();
  in_progress_fillets.pop_back();

  active_fillet af{next, -1, -1};

  std::pair<int, int> next_fillets = find_next_fillet_choice(af);

  DBG_ASSERT(next_fillets.first != -1);
  DBG_ASSERT(next_fillets.second != -1);

  af.fillet_group_index = next_fillets.first;
  af.fillet_index = next_fillets.second;

  return af;
}

void MainWindow::fillet_next() {
  std::pair<int, int> next_fillets = find_next_fillet_choice(active_fillet_part);

  if (next_fillets.first == -1) {
    finished_fillets.push_back(active_fillet_part.part);
    fillet_next_part();
    return;
  }

  active_fillet_part.fillet_group_index = next_fillets.first;
  active_fillet_part.fillet_index = next_fillets.second;
  set_active_fillet(active_fillet_part);
}

void MainWindow::fillet_next_part() {
  if (in_progress_fillets.size() == 0) {
    in_progress_heading->setText("NOTHING LEFT TO FILLET");
    set_complete_mode();
    return;
  }

  active_fillet af = next_active_fillet();
  set_active_fillet(af); //next.part, next.fillet_groups.front().possible_fillets.front());
}

std::vector<fillet_group> build_fillet_groups(const triangular_mesh& m) {
  auto sfc = build_surface_milling_constraints(m);
  vector<vector<surface> > corner_groups =
    sfc.hard_corner_groups();

  vector<surface> sfs = outer_surfaces(m);
  DBG_ASSERT(sfs.size() > 0);
  vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
  vector<point> dirs;
  for (auto& p : stock_planes) {
    dirs.push_back(p.normal());
    dirs.push_back(-1*p.normal());
  }

  vector<fillet_group> fillets;
  for (auto cg : corner_groups) {

    vector<vector<shared_edge> > possible_fillets;

    for (auto access_dir : dirs) {

      if (!is_centralized(cg)) {

	if (all_surfaces_are_millable_from(access_dir, cg)) {
	  vector<shared_edge> edges = edges_to_fillet(cg, m, access_dir);
	  possible_fillets.push_back(edges);
	  //vtk_debug_shared_edges(edges, m);
	}
      }

    }

    fillets.push_back({possible_fillets});

  }

  return fillets;
}

std::vector<filletable_part> build_filletables(const part_split& part) {
  vector<filletable_part> filletable;
  for (auto& m : nef_polyhedron_to_trimeshes(part.nef)) {
    std::vector<fillet_group> fillets = build_fillet_groups(m);
    filletable.push_back({m, fillets});
  }

  return filletable;
}

bool is_finished(const filletable_part& part) {
  if (part.fillet_groups.size() == 0) { return true; }

  for (auto& f : part.fillet_groups) {
    if (f.possible_fillets.size() > 1) {
      return false;
    }
  }

  return true;
}

void MainWindow::add_to_filletables(const part_split& part) {
  vector<filletable_part> filletables =
    build_filletables(part);

  for (auto& f : filletables) {
    if (is_finished(f)) {
      finished_fillets.push_back(f);
    } else {
      in_progress_fillets.push_back(f);
    }
  }
}

void MainWindow::add_to_queues(const part_split& part) {
  if (part.deep_features.size() > 0) {
    in_progress.push_back(part);
    return;
  }

  add_to_filletables(part);

}

void MainWindow::set_complete_mode() {
  current_mode = COMPLETE_MODE;

  clear_active_plane();
  clear_active_fillet();
  clear_active_mesh();

  in_progress_heading->setText("COMPLETE!!");

  color white(255, 255, 255);

  for (auto& p : finished_fillets) {
    auto p_data = polydata_for_trimesh(p.part);
    auto rc = random_color(white);
    color_polydata(p_data, rc.red(), rc.green(), rc.blue());
    auto p_act = polydata_actor(p_data);
    renderer->AddActor(p_act);
  }
}

MainWindow::~MainWindow() {
}
