#ifndef mainwindow_h
#define mainwindow_h

#include <QMainWindow>
#include <QScopedPointer>

#include <QVTKWidget.h>
#include <QLabel>
#include <QPushButton>


// namespace Ui
// {
//     class MainWindow;
// }

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();


private slots:
  void handle_accept_slice();

private:
  
  QVTKWidget* vtk_window;

  QLabel* in_progress_heading;


  QPushButton* accept_button;
};

#endif
