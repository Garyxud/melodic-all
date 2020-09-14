#include "preferences_dialog.h"
#include "ui_preferences_dialog.h"
#include <QSettings>

PreferencesDialog::PreferencesDialog(QWidget* parent) : QDialog(parent), ui(new Ui::PreferencesDialog)
{
  ui->setupUi(this);
  QSettings settings;
  QString theme = settings.value("Preferences::theme").toString();
  if (theme == "style_dark")
  {
    ui->comboBoxTheme->setCurrentIndex(1);
  }
  else
  {
    ui->comboBoxTheme->setCurrentIndex(0);
  }

  bool use_plot_color_index = settings.value("Preferences::use_plot_color_index", false).toBool();
  bool remember_color = settings.value("Preferences::remember_color", true).toBool();

  ui->checkBoxRememberColor->setChecked(remember_color);
  ui->radioLocalColorIndex->setChecked(use_plot_color_index);
  ui->radioGlobalColorIndex->setChecked(!use_plot_color_index);
}

PreferencesDialog::~PreferencesDialog()
{
  delete ui;
}

void PreferencesDialog::on_buttonBox_accepted()
{
  QSettings settings;
  settings.setValue("Preferences::theme", ui->comboBoxTheme->currentIndex() == 1 ? "style_dark" : "style_light");

  settings.setValue("Preferences::remember_color", ui->checkBoxRememberColor->isChecked());

  settings.setValue("Preferences::use_plot_color_index", ui->radioLocalColorIndex->isChecked());
}
