#ifndef PREFERENCES_DIALOG_H
#define PREFERENCES_DIALOG_H

#include <QDialog>

namespace Ui
{
class PreferencesDialog;
}

class PreferencesDialog : public QDialog
{
  Q_OBJECT

public:
  explicit PreferencesDialog(QWidget* parent = nullptr);
  ~PreferencesDialog();

private slots:
  void on_buttonBox_accepted();

private:
  Ui::PreferencesDialog* ui;
};

#endif  // PREFERENCES_DIALOG_H
