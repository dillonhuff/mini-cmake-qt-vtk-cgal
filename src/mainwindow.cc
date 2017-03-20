#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include "geometry/vtk_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/surface_planning.h"
#include "system/parse_stl.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

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
  QPushButton* accept_button = new QPushButton("Accept slice");
  QPushButton* reject_button = new QPushButton("Reject slice");
  QPushButton* set_done_button = new QPushButton("Done with part");

  QVBoxLayout* update_buttons = new QVBoxLayout();
  update_buttons->addWidget(button2);
  update_buttons->addWidget(accept_button);
  update_buttons->addWidget(reject_button);
  update_buttons->addWidget(set_done_button);

  QLabel* in_progress_heading = new QLabel();
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

  auto m =
    parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);
  auto planes = possible_slice_planes(m);

  DBG_ASSERT(planes.size() > 0);

  auto plane_act = plane_actor(vtk_plane(planes.front()));

  auto mesh_pd = polydata_for_trimesh(m);

  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(mesh_pd);

  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();

  renderer->SetBackground(1, 1, 1);
  renderer->AddActor(actor);
  renderer->AddActor(plane_act);
  
  vtk_window->GetRenderWindow()->AddRenderer(renderer);

  vtk_window->show();
  
}

MainWindow::~MainWindow()
{
}
