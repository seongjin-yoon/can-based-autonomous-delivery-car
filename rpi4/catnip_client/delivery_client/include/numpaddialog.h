#ifndef NUMPADDIALOG_H
#define NUMPADDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

class NumPadDialog : public QDialog
{
    Q_OBJECT

public:
    explicit NumPadDialog(QWidget *parent = nullptr);
    QString getPin() const { return le_input->text(); }

private:
    QLineEdit *le_input;
    void applyStyles();
};

#endif
