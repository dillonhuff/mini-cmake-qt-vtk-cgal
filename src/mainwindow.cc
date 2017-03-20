#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "geometry/vtk_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QHBoxLayout>

using namespace gca;

void delete_duplicate_plans(std::vector<plane>& planes) {
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

  delete_duplicate_plans(possible_slice_planes);

  return possible_slice_planes;
}

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
{

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

  connect(accept_button, SIGNAL (released()), this, SLOT (handle_accept_slice()));
  connect(reject_button, SIGNAL (released()), this, SLOT (handle_reject_slice()));
  connect(set_done_button, SIGNAL (released()), this, SLOT (handle_set_done()));

  active_mesh =
    parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);
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

void MainWindow::handle_accept_slice() {
  in_progress_heading->setText("OH YEAH!!!");

  auto part_nef = trimesh_to_nef_polyhedron(active_mesh);

  auto clipped_nef_pos = clip_nef(part_nef, active_plane.slide(0.0001));
  auto clipped_nef_neg = clip_nef(part_nef, active_plane.flip().slide(0.0001));

  auto new_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);

  in_progress.push_back(clipped_nef_neg);

  update_active_mesh(new_meshes.front());
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

void MainWindow::update_active_plane(const gca::plane p) {
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

void MainWindow::handle_set_done() {

  finished.push_back(active_mesh);

  if (in_progress.size() == 0) {
    in_progress_heading->setText("ALL DONE");
    return;
  }

  auto next_mesh = nef_to_single_trimesh(in_progress.back());
  in_progress.pop_back();

  update_active_mesh(next_mesh);
}

MainWindow::~MainWindow()
{
}
