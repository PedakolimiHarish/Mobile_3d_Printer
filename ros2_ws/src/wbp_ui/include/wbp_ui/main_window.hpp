#pragma once

#include <QWidget>
#include <QPushButton>

#include "wbp_ui/ui_policy.hpp"

namespace wbp_ui
{

  class MainWindow : public QWidget
  {
  Q_OBJECT // ← MUST be here, inside class, first line

      public : explicit MainWindow(QWidget *parent = nullptr);

    void updateUi(const UiState &state);

  signals:
    void startRequested();
    void pauseRequested();
    void resumeRequested();
    void abortRequested();
    void loadGcodeRequested(QString path);

  private:
    QPushButton *start_btn_;
    QPushButton *pause_btn_;
    QPushButton *resume_btn_;
    QPushButton *abort_btn_;

    void applyActionState(
        QPushButton *button,
        const UiActionState &action,
        const QString &label);
  };

} // namespace wbp_ui
