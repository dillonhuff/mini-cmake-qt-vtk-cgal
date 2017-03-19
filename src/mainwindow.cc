#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include "geometry/vtk_utils.h"
#include "system/parse_stl.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace gca;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)//, ui(new Ui::MainWindow)
{
  //ui->setupUi(this);

  vtk_window = new QVTKWidget(this, Qt::Widget);
  setCentralWidget(vtk_window);

  auto m =
    parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

  auto mesh_pd = polydata_for_trimesh(m);

  // vtkSmartPointer<vtkSphereSource> sphereSource = 
  //   vtkSmartPointer<vtkSphereSource>::New();
  // sphereSource->SetCenter(0.0, 0.0, 0.0);
  // sphereSource->SetRadius(5.0);
 
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(mesh_pd); //sphereSource->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();

  renderer->SetBackground(1, 1, 1);
  renderer->AddActor(actor);
  
  vtk_window->GetRenderWindow()->AddRenderer(renderer);

  vtk_window->show();
  
}

MainWindow::~MainWindow()
{
}
