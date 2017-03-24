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

enum edit_mode { FILLET_MODE, SLICE_MODE, COMPLETE_MODE };

struct fillet_group {
  std::vector<std::vector<gca::shared_edge> > possible_fillets;
};

struct filletable_part {
  gca::triangular_mesh part;
  std::vector<fillet_group> fillet_groups;
};

struct active_fillet {
  filletable_part part;
  int fillet_group_index, fillet_index;

  const std::vector<gca::shared_edge>& current_fillet() const {
    return part.fillet_groups[fillet_group_index].possible_fillets[fillet_index];
  }
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
  vtkSmartPointer<vtkActor> active_fillet_actor;
  vtkSmartPointer<vtkPolyData> active_mesh_polydata;

  std::vector<gca::plane> slice_planes;
  gca::plane active_plane;
  gca::triangular_mesh active_mesh;

  std::vector<gca::part_split> in_progress;

  active_fillet active_fillet_part;
  int active_fillet_group_index, active_fillet_index;
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
  void clear_active_mesh();
  void clear_active_fillet();

  void slice_next_part();

  // Commands for slice mode
  void handle_accept_slice();
  void handle_reject_slice();
  void handle_set_done_slice();

  void add_to_queues(const gca::part_split& part);

  void add_to_filletables(const gca::part_split& part);

  // Commands for fillet mode
  void handle_accept_fillet();
  void handle_reject_fillet();
  void handle_set_done_fillet();

  void switch_to_fillet_mode();
  void fillet_next_part();
  void set_active_fillet(const active_fillet& af);
  void fillet_next();

    // void set_active_fillet(const gca::triangular_mesh& part,
    // 			 const std::vector<gca::shared_edge>& fillet);

  active_fillet next_active_fillet();

  // Commands for complete mode
  void set_complete_mode();
  
};

#endif
