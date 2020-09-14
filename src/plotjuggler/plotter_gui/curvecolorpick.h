#ifndef CURVECOLORPICK_H
#define CURVECOLORPICK_H

#include <QDialog>
#include <QListWidgetItem>
#include "color_wheel.hpp"
#include "color_preview.hpp"
#include "color_palette.hpp"
#include "swatch.hpp"

namespace Ui
{
class CurveColorPick;
}

class CurveColorPick : public QDialog
{
  Q_OBJECT

public:
  explicit CurveColorPick(const std::map<std::string, QColor>& mapped_colors, QWidget* parent = 0);
  ~CurveColorPick();

  bool anyColorModified() const;

private slots:
  void on_pushButtonClose_clicked();

  void on_pushButtonUndo_clicked();

  void on_listWidget_itemClicked(QListWidgetItem* item);

  void on_colorChanged(QColor color);

signals:
  void changeColor(QString, QColor);

private:
  Ui::CurveColorPick* ui;
  color_widgets::ColorWheel* _color_wheel;
  color_widgets::ColorPreview* _color_preview;
  color_widgets::Swatch* _color_palette;

  const std::map<std::string, QColor>& _mapped_colors;
  bool _any_modified;
};

#endif  // CURVECOLORPICK_H
