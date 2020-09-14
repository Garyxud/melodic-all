#include "curvelist_view.h"
#include <QApplication>
#include <QDrag>
#include <QMessageBox>
#include <QMimeData>
#include <QDebug>
#include <QScrollBar>
#include "curvelist_panel.h"

CurveTableView::CurveTableView(CurveListPanel* parent) : QTableWidget(parent), CurvesView(parent)
{
  setColumnCount(2);
  setEditTriggers(NoEditTriggers);
  setDragEnabled(false);
  setDefaultDropAction(Qt::IgnoreAction);
  setDragDropOverwriteMode(false);
  setDragDropMode(NoDragDrop);
  viewport()->installEventFilter(this);

  verticalHeader()->setVisible(false);
  horizontalHeader()->setVisible(false);

  horizontalHeader()->setStretchLastSection(true);

  setColumnWidth(1, 120);

  setHorizontalHeaderItem(0, new QTableWidgetItem("Time series"));
  setHorizontalHeaderItem(1, new QTableWidgetItem("Current value"));

  setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  setShowGrid(false);
}

void CurveTableView::addItem(const QString& item_name)
{
  if (_inserted_curves.contains(item_name))
  {
    return;
  }

  auto item = new SortedTableItem<QTableWidgetItem>(item_name);
  QFont font = QFontDatabase::systemFont(QFontDatabase::GeneralFont);
  font.setPointSize(_point_size);
  item->setFont(font);
  const int row = rowCount();
  QTableWidget::setRowCount(row + 1);
  QTableWidget::setItem(row, 0, item);

  auto val_cell = new QTableWidgetItem("-");
  val_cell->setTextAlignment(Qt::AlignRight);
  val_cell->setFlags(Qt::NoItemFlags | Qt::ItemIsEnabled);
  font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  font.setPointSize(_point_size - 1);
  val_cell->setFont(font);
  val_cell->setFlags(Qt::NoItemFlags);

  QTableWidget::setItem(row, 1, val_cell);

  _inserted_curves.insert({ item_name });
}

void CurveTableView::refreshColumns()
{
  sortByColumn(0, Qt::AscendingOrder);
  horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
  setColumnWidth(1, 120);
}

std::vector<std::string> CurveTableView::getSelectedNames()
{
  std::vector<std::string> non_hidden_list;

  for (const QTableWidgetItem* cell : selectedItems())
  {
    non_hidden_list.push_back(cell->text().toStdString());
  }
  return non_hidden_list;
}

void CurveTableView::refreshFontSize()
{
  if (rowCount() == 0)
  {
    return;
  }
  setViewResizeEnabled(false);

  for (int row = 0; row < rowCount(); row++)
  {
    for (int col = 0; col < 2; col++)
    {
      auto cell = item(row, col);
      QFont font = QFontDatabase::systemFont(QFontDatabase::GeneralFont);
      font.setPointSize(_point_size - col);
      cell->setFont(font);
    }
  }
  setViewResizeEnabled(true);
}

void CurveTableView::removeCurve(const QString& name)
{
  for (int row = 0; row < model()->rowCount(); row++)
  {
    if (item(row, 0)->text() == name)
    {
      removeRow(row);
      break;
    }
  }
  _inserted_curves.remove(name);
}

bool CurveTableView::applyVisibilityFilter(const QString& search_string)
{
  setViewResizeEnabled(false);

  bool updated = false;
  _hidden_count = 0;

  QStringList spaced_items = search_string.split(' ');

  for (int row = 0; row < model()->rowCount(); row++)
  {
    auto cell = item(row, 0);
    QString name = cell->text();
    bool toHide = false;

    if (search_string.isEmpty() == false)
    {
      for (const auto& item : spaced_items)
      {
        if (name.contains(item, Qt::CaseInsensitive) == false)
        {
          toHide = true;
          break;
        }
      }
    }
    if (toHide)
    {
      _hidden_count++;
    }

    if (toHide != isRowHidden(row))
    {
      updated = true;
    }

    setRowHidden(row, toHide);
  }

  setViewResizeEnabled(true);
  return updated;
}

void CurveTableView::setViewResizeEnabled(bool enable)
{
  if (enable)
  {
    horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    setColumnWidth(1, 120);
    verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  }
  else
  {
    horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  }
}

void CurveTableView::hideValuesColumn(bool hide)
{
  setViewResizeEnabled(true);
  if (hide)
  {
    hideColumn(1);
  }
  else
  {
    showColumn(1);
  }
}

bool CurvesView::eventFilterBase(QObject* object, QEvent* event)
{
  QAbstractItemView* table_widget = qobject_cast<QAbstractItemView*>(object);

  if (qobject_cast<QScrollBar*>(object))
  {
    return false;
  }

  auto obj = object;
  while (obj && !table_widget)
  {
    obj = obj->parent();
    table_widget = qobject_cast<QAbstractItemView*>(obj);
  }

  bool ctrl_modifier_pressed = (QGuiApplication::keyboardModifiers() == Qt::ControlModifier);

  if (event->type() == QEvent::MouseButtonPress)
  {
    QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);

    _dragging = false;
    _drag_start_pos = mouse_event->pos();

    if (mouse_event->button() == Qt::LeftButton)
    {
      _newX_modifier = false;
    }
    else if (mouse_event->button() == Qt::RightButton)
    {
      _newX_modifier = true;
    }
    else
    {
      return true;
    }
    return false;
  }
  else if (event->type() == QEvent::MouseMove)
  {
    QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);
    double distance_from_click = (mouse_event->pos() - _drag_start_pos).manhattanLength();

    if ((mouse_event->buttons() == Qt::LeftButton || mouse_event->buttons() == Qt::RightButton) &&
        distance_from_click >= QApplication::startDragDistance() && !_dragging)
    {
      _dragging = true;
      QDrag* drag = new QDrag(table_widget);
      QMimeData* mimeData = new QMimeData;

      QByteArray mdata;
      QDataStream stream(&mdata, QIODevice::WriteOnly);

      auto selected_names = getSelectedNames();
      std::sort(selected_names.begin(), selected_names.end());

      for (const auto& curve_name : selected_names)
      {
        stream << QString::fromStdString(curve_name);
      }

      if (!_newX_modifier)
      {
        mimeData->setData("curveslist/add_curve", mdata);
      }
      else
      {
        if (selected_names.size() != 2)
        {
          if (selected_names.size() >= 1)
          {
            QMessageBox::warning(table_widget, "New in version 2.3+",
                                 "To create a new XY curve, you must select two "
                                 "timeseries and "
                                 "drag&drop them using the RIGHT mouse button.",
                                 QMessageBox::Ok);
          }
          return true;
        }
        mimeData->setData("curveslist/new_XY_axis", mdata);

        QPixmap cursor(QSize(160, 30));
        cursor.fill(Qt::transparent);

        QPainter painter;
        painter.begin(&cursor);
        painter.setPen(QColor(22, 22, 22));

        QString text("Create a XY curve");
        painter.setFont(QFont("Arial", 14));

        painter.setBackground(Qt::transparent);
        painter.setPen(table_widget->palette().foreground().color());
        painter.drawText(QRect(0, 0, 160, 30), Qt::AlignHCenter | Qt::AlignVCenter, text);
        painter.end();

        drag->setDragCursor(cursor, Qt::MoveAction);
      }

      drag->setMimeData(mimeData);
      drag->exec(Qt::CopyAction | Qt::MoveAction);
    }
    return true;
  }
  else if (event->type() == QEvent::Wheel)
  {
    QWheelEvent* wheel_event = dynamic_cast<QWheelEvent*>(event);
    int prev_size = _point_size;
    if (ctrl_modifier_pressed)
    {
      if (_point_size > 6 && wheel_event->delta() < 0)
      {
        _point_size--;
      }
      else if (_point_size < 14 && wheel_event->delta() > 0)
      {
        _point_size++;
      }
      if (_point_size != prev_size)
      {
        _parent_panel->changeFontSize(_point_size);
      }
      return true;
    }
  }

  return false;
}
