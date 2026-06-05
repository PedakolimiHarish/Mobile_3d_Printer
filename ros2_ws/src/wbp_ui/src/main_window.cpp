// start of file: ros2_ws/src/wbp_ui/src/main_window.cpp
#include "wbp_ui/main_window.hpp"
#include <QVBoxLayout>
#include <QFileDialog>

namespace wbp_ui
{

  MainWindow::MainWindow(QWidget *parent)
      : QWidget(parent)
  {
    start_btn_ = new QPushButton("Start");
    pause_btn_ = new QPushButton("Pause");
    resume_btn_ = new QPushButton("Resume");
    abort_btn_ = new QPushButton("Abort");
    QPushButton *load_btn_ = new QPushButton("Load G-code");

    auto *layout = new QVBoxLayout(this);
    layout->addWidget(load_btn_);
    layout->addWidget(start_btn_);
    layout->addWidget(pause_btn_);
    layout->addWidget(resume_btn_);
    layout->addWidget(abort_btn_);

    connect(start_btn_, &QPushButton::clicked, this, &MainWindow::startRequested);
    connect(pause_btn_, &QPushButton::clicked, this, &MainWindow::pauseRequested);
    connect(resume_btn_, &QPushButton::clicked, this, &MainWindow::resumeRequested);
    connect(abort_btn_, &QPushButton::clicked, this, &MainWindow::abortRequested);

    connect(load_btn_, &QPushButton::clicked, this, [this]()
            {
    QString file = QFileDialog::getOpenFileName(
        this,
        "Select G-code file",
        "",
        "G-code Files (*.gcode)");

    if (!file.isEmpty())
    {
        emit loadGcodeRequested(file);
    } });

    // HARD DISABLE ALL ACTIONS UNTIL FIRST STATE
    start_btn_->setEnabled(false);
    pause_btn_->setEnabled(false);
    resume_btn_->setEnabled(false);
    abort_btn_->setEnabled(false);
  }

  void MainWindow::updateUi(const UiState &state)
  {
    applyActionState(start_btn_, state.start, "Start");
    applyActionState(pause_btn_, state.pause, "Pause");
    applyActionState(resume_btn_, state.resume, "Resume");
    applyActionState(abort_btn_, state.abort, "Abort");
  }

  void MainWindow::applyActionState(
      QPushButton *button,
      const UiActionState &action,
      const QString &label)
  {
    Q_UNUSED(label);

    button->setEnabled(action.enabled);

    if (!action.enabled && !action.reason.empty())
    {
      button->setToolTip(QString::fromStdString(action.reason));
    }
    else
    {
      button->setToolTip(QString());
    }
  }

} // namespace wbp_ui

// end of file: ros2_ws/src/wbp_ui/src/main_window.cpp