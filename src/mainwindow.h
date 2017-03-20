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
#include "part_slicing.h"

enum edit_mode { FILLET_MODE, SLICE_MODE };

struct filletable_part {
  gca::triangular_mesh part;
  
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();


private slots:
  void handle_accept();
  void handle_reject();
  void handle_set_done();

private:

  edit_mode current_mode;

  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkActor> active_plane_actor;
  vtkSmartPointer<vtkActor> active_mesh_actor;
  vtkSmartPointer<vtkPolyData> active_mesh_polydata;

  std::vector<gca::plane> slice_planes;
  gca::plane active_plane;
  gca::triangular_mesh active_mesh;

  std::vector<gca::part_split> in_progress;

  std::vector<filletable_part> in_progress_fillets;
  std::vector<filletable_part> finished_fillets;

  QVTKWidget* vtk_window;

  QLabel* in_progress_heading;


  QPushButton* accept_button;
  QPushButton* reject_button;
  QPushButton* set_done_button;

  void update_active_mesh(const gca::triangular_mesh& m);
  void update_active_plane(const gca::plane p);
  void clear_active_plane();

  void continue_with_next_in_progress_part();

  // Commands for slice mode
  void handle_accept_slice();
  void handle_reject_slice();
  void handle_set_done_slice();

  // Commands for fillet mode
  void handle_accept_fillet();
  void handle_reject_fillet();
  void handle_set_done_fillet();
  
};

#endif
