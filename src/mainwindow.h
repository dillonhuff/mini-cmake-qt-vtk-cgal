#ifndef mainwindow_h
#define mainwindow_h

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <QMainWindow>
#include <QScopedPointer>
#include <QVTKWidget.h>
#include <QLabel>
#include <QPushButton>

#include "geometry/plane.h"
#include "geometry/mesh_operations.h"
#include "geometry/triangular_mesh.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();


private slots:
  void handle_accept_slice();
  void handle_reject_slice();
  void handle_set_done();

private:

  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkActor> active_plane_actor;
  vtkSmartPointer<vtkActor> active_mesh_actor;
  vtkSmartPointer<vtkPolyData> active_mesh_polydata;

  std::vector<gca::plane> slice_planes;
  gca::plane active_plane;
  gca::triangular_mesh active_mesh;

  std::vector<gca::triangular_mesh> finished;
  std::vector<gca::Nef_polyhedron> in_progress;

  QVTKWidget* vtk_window;

  QLabel* in_progress_heading;


  QPushButton* accept_button;
  QPushButton* reject_button;
  QPushButton* set_done_button;

  void update_active_mesh(const gca::triangular_mesh& m);
  void update_active_plane(const gca::plane p);
  void clear_active_plane();

};

#endif
