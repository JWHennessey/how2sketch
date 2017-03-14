#ifndef SUGGESTIVECONTOURSPANEL_H
#define SUGGESTIVECONTOURSPANEL_H

#include <QVBoxLayout>
#include <QCheckBox>
#include <QPushButton>

class SuggestiveContoursPanel :public QWidget
{
    Q_OBJECT
public:
    SuggestiveContoursPanel();
    ~SuggestiveContoursPanel();
    auto init() -> void;
private:
    QVBoxLayout* items;
private slots:
    void cLimitValueChanged(double val);
    void scLimitValueChanged(double val);
    void dwkrLimitValueChanged(double val);
    void jeroenToggled(bool val);
signals:
    void updateViewer();
};

#endif // SUGGESTIVECONTOURSPANEL_H
