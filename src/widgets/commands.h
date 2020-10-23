#ifndef COMMANDS_H
#define COMMANDS_H

#include <QtWidgets>
#include "setting.h"

class Commands : public QWidget
{
    Q_OBJECT
public:
    explicit Commands(QString ac_id, QWidget *parent = nullptr);

signals:

public slots:

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void addFlightPlanButtons(QGridLayout* fp_buttons_layout);
    void addSettingsButtons(QGridLayout* settings_buttons_layout);
    void addSpecialCommands(QGridLayout* special_commands_layout);
    void addCommandButton(QGridLayout*, QString icon, int row, int col, std::function<void()> callback);
    QString ac_id;
};

#endif // COMMANDS_H
