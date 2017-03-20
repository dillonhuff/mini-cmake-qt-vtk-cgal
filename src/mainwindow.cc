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

  part_split pos_split = build_part_split(clipped_nef_pos);
  part_split neg_split = build_part_split(clipped_nef_neg);

  // if (pos_split.deep_features.size() > 0) {
  //   auto new_meshes = nef_polyhedron_to_trimeshes(clipped_nef_pos);

  bool pos_finished = pos_split.deep_features.size() == 0;
  bool neg_finished = neg_split.deep_features.size() == 0;

  if (!pos_finished) {
    in_progress.push_back(pos_split);
  }

  if (!neg_finished) {
    in_progress.push_back(neg_split);
  }

  continue_with_next_in_progress_part();
  //   if (neg_split.deep_features.size() > 0) {
  //     in_progress.push_back(neg_split);
  //   }

  //   update_active_mesh(new_meshes.front());

  //   return;
  // }

  // if (neg_split.deep_features.size() > 0) {
  //   auto new_meshes = nef_polyhedron_to_trimeshes(clipped_nef_neg);

  //   update_active_mesh(new_meshes.front());

  //   return;
  // }

  //in_progress_heading->setText("COMPLETELY DONE");
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

  auto next_mesh = nef_to_single_trimesh(in_progress.back().nef);
  in_progress.pop_back();

  update_active_mesh(next_mesh);
}

void MainWindow::continue_with_next_in_progress_part() {

  if (in_progress.size() == 0) {
    in_progress_heading->setText("All parts are finished");
    return;
  }

  part_split next_part = in_progress.back();
  in_progress.pop_back();

  auto mesh = nef_to_single_trimesh(next_part.nef);

  update_active_mesh(mesh);

}

MainWindow::~MainWindow()
{
}
