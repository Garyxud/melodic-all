#include "function_editor.h"
#include "custom_function.h"
#include "plotwidget.h"
#include <QDebug>
#include <QMessageBox>
#include <QFont>
#include <QDomDocument>
#include <QDomElement>
#include <QFontDatabase>
#include <QFile>
#include <QMenu>
#include <QAction>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QByteArray>
#include <QInputDialog>

#include "lua_custom_function.h"
#include "qml_custom_function.h"

AddCustomPlotDialog::AddCustomPlotDialog(PlotDataMapRef& plotMapData, const CustomPlotMap& mapped_custom_plots,
                                         QWidget* parent)
  : QDialog(parent)
  , _plot_map_data(plotMapData)
  , _custom_plots(mapped_custom_plots)
  , ui(new Ui::FunctionEditor)
  , _v_count(1)
{
  ui->setupUi(this);

  QSettings settings;
  bool is_qml = settings.value("CustomFunction/language", "qml").toString() == "qml";
  if (is_qml)
  {
    ui->labelLanguage->setText("Used language: Javascript");
  }
  else
  {
    ui->labelLanguage->setText("Used language: Lua");
  }

  this->setWindowTitle("Create a custom timeseries");

  const QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  ui->globalVarsTextField->setFont(fixedFont);
  ui->mathEquation->setFont(fixedFont);
  ui->snippetTextEdit->setFont(fixedFont);

  QStringList numericPlotNames;
  for (const auto& p : _plot_map_data.numeric)
  {
    QString name = QString::fromStdString(p.first);
    numericPlotNames.push_back(name);
  }
  numericPlotNames.sort(Qt::CaseInsensitive);
  for (const QString& name : numericPlotNames)
  {
    ui->combo_linkedChannel->addItem(name);
    ui->curvesListWidget->addItem(name);
  }

  QByteArray saved_xml = settings.value("AddCustomPlotDialog.savedXML", QByteArray()).toByteArray();
  restoreGeometry(settings.value("AddCustomPlotDialog.geometry").toByteArray());

  if (saved_xml.isEmpty())
  {
    QFile file("://resources/default.snippets.xml");
    if (!file.open(QIODevice::ReadOnly))
    {
      throw std::runtime_error("problem with default.snippets.xml");
    }
    saved_xml = file.readAll();
  }

  importSnippets(saved_xml);

  ui->snippetsListRecent->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(ui->snippetsListRecent, &QListWidget::customContextMenuRequested, this,
          &AddCustomPlotDialog::recentContextMenu);

  ui->snippetsListSaved->setContextMenuPolicy(Qt::CustomContextMenu);

  connect(ui->snippetsListSaved, &QListWidget::customContextMenuRequested, this,
          &AddCustomPlotDialog::savedContextMenu);

  ui->splitter->setStretchFactor(0, 3);
  ui->splitter->setStretchFactor(1, 2);

  ui->globalVarsTextField->setPlainText(settings.value("AddCustomPlotDialog.previousGlobals", "").toString());

  ui->mathEquation->setPlainText(settings.value("AddCustomPlotDialog.previousFunction", "return value").toString());

  QString language = settings.value("AddCustomPlotDialog.previousLanguage", "LUA").toString();

  if (language == "JS")
  {
    ui->radioButtonJS->setChecked(true);
  }
  else if (language == "LUA")
  {
    ui->radioButtonLua->setChecked(true);
  }
}

AddCustomPlotDialog::~AddCustomPlotDialog()
{
  QSettings settings;
  settings.setValue("AddCustomPlotDialog.savedXML", exportSnippets());
  settings.setValue("AddCustomPlotDialog.geometry", saveGeometry());
  settings.setValue("AddCustomPlotDialog.previousGlobals", ui->globalVarsTextField->toPlainText());
  settings.setValue("AddCustomPlotDialog.previousFunction", ui->mathEquation->toPlainText());
  if (ui->radioButtonJS->isChecked())
  {
    settings.setValue("AddCustomPlotDialog.previousLanguage", "JS");
  }
  else if (ui->radioButtonLua->isChecked())
  {
    settings.setValue("AddCustomPlotDialog.previousLanguage", "LUA");
  }
  delete ui;
}

void AddCustomPlotDialog::setLinkedPlotName(const QString& linkedPlotName)
{
  int idx = ui->combo_linkedChannel->findText(linkedPlotName);
  if (idx == -1)
  {
    idx = 0;
    ui->combo_linkedChannel->insertItem(idx, linkedPlotName);
  }
  ui->combo_linkedChannel->setCurrentIndex(idx);
}

void AddCustomPlotDialog::setEditorMode(EditorMode mode)
{
  ui->label_linkeChannel->setVisible(mode != FUNCTION_ONLY);
  ui->combo_linkedChannel->setVisible(mode != FUNCTION_ONLY);
  ui->pushButtonCreate->setVisible(mode != FUNCTION_ONLY);
  ui->pushButtonSave->setVisible(mode != TIMESERIES_ONLY);
}

QString AddCustomPlotDialog::getLinkedData() const
{
  return ui->combo_linkedChannel->currentText();
}

QString AddCustomPlotDialog::getGlobalVars() const
{
  return ui->globalVarsTextField->toPlainText();
}

QString AddCustomPlotDialog::getEquation() const
{
  return ui->mathEquation->toPlainText();
}

QString AddCustomPlotDialog::getName() const
{
  return ui->nameLineEdit->text();
}

QString AddCustomPlotDialog::getLanuguage() const
{
  if (ui->radioButtonLua->isChecked())
  {
    return "LUA";
  }
  return "JS";
}

void AddCustomPlotDialog::editExistingPlot(CustomPlotPtr data)
{
  ui->globalVarsTextField->setPlainText(data->globalVars());
  ui->mathEquation->setPlainText(data->function());
  setLinkedPlotName(QString::fromStdString(data->linkedPlotName()));
  ui->pushButtonCreate->setText("Update");
  ui->nameLineEdit->setText(QString::fromStdString(data->name()));
  ui->nameLineEdit->setEnabled(false);

  if (data->language() == "LUA")
  {
    ui->radioButtonLua->setChecked(true);
  }
  else
  {
    ui->radioButtonJS->setChecked(true);
  }
  ui->radioButtonLua->setEnabled(false);
  ui->radioButtonJS->setEnabled(false);

  _is_new = false;
}

CustomPlotPtr AddCustomPlotDialog::getCustomPlotData() const
{
  return _plot;
}

void AddCustomPlotDialog::on_curvesListWidget_doubleClicked(const QModelIndex& index)
{
  QString appendString = QString("V%1 = $$%2$$\n").arg(_v_count++).arg(ui->curvesListWidget->item(index.row())->text());

  QPlainTextEdit* edit = ui->mathEquation;

  if (ui->globalVarsTextField->hasFocus())
  {
    edit = ui->globalVarsTextField;
  }

  if (!edit->toPlainText().endsWith("\n"))
  {
    edit->insertPlainText("\n");
  }
  edit->insertPlainText(appendString);
}

void AddCustomPlotDialog::importSnippets(const QByteArray& xml_text)
{
  ui->snippetsListSaved->clear();

  _snipped_saved = GetSnippetsFromXML(xml_text);

  for (const auto& it : _snipped_saved)
  {
    ui->snippetsListSaved->addItem(it.first);
  }

  for (const auto& custom_it : _custom_plots)
  {
    const auto& math_plot = custom_it.second;
    SnippetData snippet;
    snippet.name = QString::fromStdString(math_plot->name());

    if (_snipped_saved.count(snippet.name) > 0)
    {
      continue;
    }

    snippet.globalVars = math_plot->globalVars();
    snippet.equation = math_plot->function();
    snippet.language = math_plot->language();

    _snipped_recent.insert({ snippet.name, snippet });
    ui->snippetsListRecent->addItem(snippet.name);
  }
  ui->snippetsListRecent->sortItems();
  ui->snippetsListSaved->sortItems();
}

QByteArray AddCustomPlotDialog::exportSnippets() const
{
  QDomDocument doc;
  auto root = ExportSnippets(_snipped_saved, doc);
  doc.appendChild(root);
  return doc.toByteArray(2);
}

void AddCustomPlotDialog::on_snippetsListSaved_currentRowChanged(int current_row)
{
  if (current_row < 0)
  {
    ui->snippetTextEdit->setPlainText("");
    return;
  }
  const auto& name = ui->snippetsListSaved->currentItem()->text();
  const SnippetData& snippet = _snipped_saved.at(name);
  QString desc = QString("%1\n\nfunction calc(time,value)\n{\n%2\n}").arg(snippet.globalVars).arg(snippet.equation);
  ui->snippetTextEdit->setPlainText(desc);
}

void AddCustomPlotDialog::on_snippetsListSaved_doubleClicked(const QModelIndex& index)
{
  const auto& name = ui->snippetsListSaved->item(index.row())->text();
  const SnippetData& snippet = _snipped_saved.at(name);

  ui->globalVarsTextField->setPlainText(snippet.globalVars);
  ui->mathEquation->setPlainText(snippet.equation);
  if (snippet.language == "LUA")
  {
    ui->radioButtonLua->setChecked(true);
  }
  else if (snippet.language == "JS")
  {
    ui->radioButtonJS->setChecked(true);
  }
}

void AddCustomPlotDialog::on_snippetsListRecent_currentRowChanged(int current_row)
{
  if (current_row < 0)
  {
    ui->snippetTextEdit->setPlainText("");
    return;
  }
  const auto& name = ui->snippetsListRecent->currentItem()->text();
  const SnippetData& snippet = _snipped_recent.at(name);

  QString desc = QString("%1\n\nfunction calc(time,value)\n{\n%2\n}").arg(snippet.globalVars).arg(snippet.equation);
  ui->snippetTextEdit->setPlainText(desc);
}

void AddCustomPlotDialog::on_snippetsListRecent_doubleClicked(const QModelIndex& index)
{
  const auto& name = ui->snippetsListRecent->item(index.row())->text();
  const SnippetData& snippet = _snipped_recent.at(name);

  if (snippet.language == "LUA")
  {
    ui->radioButtonLua->setChecked(true);
  }
  else if (snippet.language == "JS")
  {
    ui->radioButtonJS->setChecked(true);
  }
  ui->globalVarsTextField->setPlainText(snippet.globalVars);
  ui->mathEquation->setPlainText(snippet.equation);
}

void AddCustomPlotDialog::recentContextMenu(const QPoint& pos)
{
  auto list_recent = ui->snippetsListRecent;

  if (list_recent->selectedItems().size() != 1)
  {
    return;
  }

  auto list_saved = ui->snippetsListSaved;

  auto item = list_recent->selectedItems().first();
  const auto& name = item->text();

  QMenu menu;
  QAction* move_item = new QAction("Move to Saved", this);
  menu.addAction(move_item);

  connect(move_item, &QAction::triggered, this, [=]() {
    auto snippet_it = _snipped_recent.find(name);

    if (addToSaved(name, snippet_it->second))
    {
      _snipped_recent.erase(snippet_it);
      delete list_recent->takeItem(list_recent->row(item));
    }
  });

  menu.exec(list_recent->mapToGlobal(pos));
}

void AddCustomPlotDialog::savedContextMenu(const QPoint& pos)
{
  auto list_saved = ui->snippetsListSaved;

  if (list_saved->selectedItems().size() != 1)
  {
    return;
  }

  QMenu menu;

  QAction* rename_item = new QAction("Rename...", this);
  menu.addAction(rename_item);

  connect(rename_item, &QAction::triggered, this, &AddCustomPlotDialog::onRenameSaved);

  QAction* remove_item = new QAction("Remove", this);
  menu.addAction(remove_item);

  connect(remove_item, &QAction::triggered, this, [list_saved, this]() {
    const auto& item = list_saved->selectedItems().first();
    _snipped_saved.erase(item->text());
    delete list_saved->takeItem(list_saved->row(item));
  });

  menu.exec(list_saved->mapToGlobal(pos));
}

void AddCustomPlotDialog::on_nameLineEdit_textChanged(const QString& name)
{
  ui->pushButtonCreate->setEnabled(!name.isEmpty());
  ui->pushButtonSave->setEnabled(!name.isEmpty());

  if (_plot_map_data.numeric.count(name.toStdString()) == 0)
  {
    ui->pushButtonCreate->setText("Create New Timeseries");
  }
  else
  {
    ui->pushButtonCreate->setText("Modify Timeseries");
  }
}

void AddCustomPlotDialog::on_buttonLoadFunctions_clicked()
{
  QSettings settings;
  QString directory_path = settings.value("AddCustomPlotDialog.loadDirectory", QDir::currentPath()).toString();

  QString fileName =
      QFileDialog::getOpenFileName(this, tr("Open Snippet Library"), directory_path, tr("Snippets (*.snippets.xml)"));
  if (fileName.isEmpty())
  {
    return;
  }

  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName));
    return;
  }

  directory_path = QFileInfo(fileName).absolutePath();
  settings.setValue("AddCustomPlotDialog.loadDirectory", directory_path);

  importSnippets(file.readAll());
}

void AddCustomPlotDialog::on_buttonSaveFunctions_clicked()
{
  QSettings settings;
  QString directory_path = settings.value("AddCustomPlotDialog.loadDirectory", QDir::currentPath()).toString();

  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Open Snippet Library"), directory_path, tr("Snippets (*.snippets.xml)"));

  if (fileName.isEmpty())
  {
    return;
  }
  if (!fileName.endsWith(".snippets.xml"))
  {
    fileName.append(".snippets.xml");
  }

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName));
    return;
  }
  auto data = exportSnippets();

  file.write(data);
  file.close();
}

void AddCustomPlotDialog::on_pushButtonSave_clicked()
{
  QString name = ui->nameLineEdit->text();

  SnippetData snippet;
  snippet.name = name;
  snippet.globalVars = ui->globalVarsTextField->toPlainText();
  snippet.equation = ui->mathEquation->toPlainText();
  snippet.language = getLanuguage();

  addToSaved(name, snippet);

  on_snippetsListSaved_currentRowChanged(ui->snippetsListSaved->currentRow());
}

bool AddCustomPlotDialog::addToSaved(const QString& name, const SnippetData& snippet)
{
  if (_snipped_saved.count(name))
  {
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Warning");
    msgBox.setText(tr("A function with the same name exists already in the list of saved functions.\n"));
    msgBox.addButton(QMessageBox::Cancel);
    QPushButton* button = msgBox.addButton(tr("Overwrite"), QMessageBox::YesRole);
    msgBox.setDefaultButton(button);

    int res = msgBox.exec();

    if (res < 0 || res == QMessageBox::Cancel)
    {
      return false;
    }
  }
  else
  {
    ui->snippetsListSaved->addItem(name);
    ui->snippetsListSaved->sortItems();
  }
  _snipped_saved[name] = snippet;
  return true;
}

void AddCustomPlotDialog::onRenameSaved()
{
  auto list_saved = ui->snippetsListSaved;
  auto item = list_saved->selectedItems().first();
  const auto& name = item->text();

  bool ok;
  QString new_name =
      QInputDialog::getText(this, tr("Change the name of the function"), tr("New name:"), QLineEdit::Normal, name, &ok);

  if (!ok || new_name.isEmpty() || new_name == name)
  {
    return;
  }

  SnippetData snippet = _snipped_saved[name];
  _snipped_saved.erase(name);
  snippet.name = new_name;

  _snipped_saved.insert({ new_name, snippet });
  item->setText(new_name);
  ui->snippetsListSaved->sortItems();
}

void AddCustomPlotDialog::on_pushButtonCreate_clicked()
{
  try
  {
    std::string plotName = getName().toStdString();

    // check if name is unique (except if is custom_plot)
    if (_plot_map_data.numeric.count(plotName) != 0 && _custom_plots.count(plotName) == 0)
    {
      throw std::runtime_error("plot name already exists and can't be modified");
    }

    SnippetData snippet;
    snippet.equation = getEquation();
    snippet.globalVars = getGlobalVars();
    snippet.language = getLanuguage();
    snippet.name = getName();

    if (snippet.language == "LUA")
    {
      _plot = std::make_unique<LuaCustomFunction>(getLinkedData().toStdString(), snippet);
    }
    else if (snippet.language == "JS")
    {
      _plot = std::make_unique<JsCustomFunction>(getLinkedData().toStdString(), snippet);
    }
    else
    {
      throw std::runtime_error("Snippet language not recognized");
    }

    QDialog::accept();
  }
  catch (const std::runtime_error& e)
  {
    _plot.reset();
    QMessageBox::critical(this, "Error", "Failed to create math plot : " + QString::fromStdString(e.what()));
    QDialog::reject();
  }
}

void AddCustomPlotDialog::on_lineEditFilter_textChanged(const QString& search_string)
{
  QStringList spaced_items = search_string.split(' ');

  for (int row = 0; row < ui->curvesListWidget->count(); row++)
  {
    auto item = ui->curvesListWidget->item(row);
    QString name = item->text();
    bool toHide = false;

    for (const auto& item : spaced_items)
    {
      if (!name.contains(item))
      {
        toHide = true;
        break;
      }
    }
    ui->curvesListWidget->setRowHidden(row, toHide);
  }
}

void AddCustomPlotDialog::on_pushButtonCancel_pressed()
{
}
