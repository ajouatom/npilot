#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>
#include <QStackedLayout>


#include "selfdrive/ui/qt/widgets/controls.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void reviewTrainingGuide();
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void reviewTrainingGuide();
  void showDriverView();
  void closeSettings();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  LabelControl *gitBranchLbl;
  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;
  ButtonControl *branchSwitcherBtn;

  Params params;
  QFileSystemWatcher *fs_watch;
};




class SelectCar : public QWidget {
  Q_OBJECT
public:
  explicit SelectCar(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selectedCar();

};

class LateralControl : public QWidget {
  Q_OBJECT
public:
  explicit LateralControl(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();

};

class CommunityPanel : public QWidget {
  Q_OBJECT

private:
  QStackedLayout* main_layout = nullptr;
  QWidget* homeScreen = nullptr;
  SelectCar* selectCar = nullptr;
  LateralControl* lateralControl = nullptr;

  QWidget* homeWidget;

public:
  explicit CommunityPanel(QWidget *parent = nullptr);
};

// ajouatom:
class CValueControl : public AbstractControl {
    Q_OBJECT

public:
    CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit = 1);

private:
    QPushButton btnplus;
    QPushButton btnminus;
    QLabel label;

    QString m_params;
    int     m_min;
    int     m_max;
    int     m_unit;

    void refresh();
};
class CPrebuiltToggle : public ToggleControl {
    Q_OBJECT

public:
    CPrebuiltToggle() : ToggleControl("Prebuilt", "Prebuilt.", "../assets/offroad/icon_shell.png", Params().getBool("OpkrPrebuiltOn")) {
        QObject::connect(this, &CPrebuiltToggle::toggleFlipped, [=](int state) {
            Params().putBool("OpkrPrebuiltOn", (bool)state);

            if (state)
            {
                std::system("cd /data/openpilot; touch prebuilt");
            }
            else
            {
                std::system("cd /data/openpilot; rm -f prebuilt");
            }

        });
    }
};
class GitHash : public AbstractControl {
    Q_OBJECT

public:
    GitHash();

private:
    QLabel local_hash;
    QLabel remote_hash;
};
