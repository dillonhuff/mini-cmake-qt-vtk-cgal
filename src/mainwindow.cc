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

#include <QHBoxLayout>
#include <QPushButton>

using namespace gca;

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
{

  vtk_window = new QVTKWidget(this, Qt::Widget);

  QHBoxLayout *layout = new QHBoxLayout;
  QPushButton *button2 = new QPushButton("button2");

  layout->addWidget(vtk_window);
  layout->addWidget(button2);

  setCentralWidget(new QWidget);
  centralWidget()->setLayout(layout);

  auto m =
    parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/SmallReverseCameraMount - Part 1.stl", 0.0001);

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
  
  vtk_window->GetRenderWindow()->AddRenderer(renderer);

  vtk_window->show();
  
}

MainWindow::~MainWindow()
{
}
