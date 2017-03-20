#ifndef mainwindow_h
#define mainwindow_h

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <QMainWindow>
#include <QScopedPointer>
#include <QVTKWidget.h>
#include <QLabel>
#include <QPushButton>

#include "geometry/plane.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();


private slots:
  void handle_accept_slice();
  void handle_reject_slice();

private:

  vtkSmartPointer<vtkRenderer> renderer;  

  std::vector<gca::plane> slice_planes;
  gca::plane active_plane;

  

  QVTKWidget* vtk_window;

  QLabel* in_progress_heading;


  QPushButton* accept_button;
  QPushButton* reject_button;
};

#endif
