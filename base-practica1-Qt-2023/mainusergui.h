#ifndef MAINUSERGUI_H
#define MAINUSERGUI_H

#include <QWidget>
#include <QtSerialPort/qserialport.h>
#include <QMessageBox>
#include "tiva_remotelink.h"
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <QPen>

namespace Ui {
class MainUserGUI;
}

class MainUserGUI : public QWidget
{
    Q_OBJECT
    
public: 
    explicit MainUserGUI(QWidget *parent = 0);
    ~MainUserGUI();
    
private slots:
    // slots privados asociados mediante "connect" en el constructor
    void cambiaLEDs();
    void cambiaMODO();
    void modoSwitches();
    void modoAdquisicion();
    void controlACME();
    void tivaStatusChanged(int status,QString message);
    void messageReceived(uint8_t type, QByteArray datos);

    //Slots asociados por nombre
    void on_runButton_clicked();    
    void on_serialPortComboBox_currentIndexChanged(const QString &arg1);
    void on_pushButton_clicked();    
    void on_colorWheel_colorChanged(const QColor &arg1);
    void on_Knob_valueChanged(double value);
    void on_ADCButton_clicked();
    void on_pingButton_clicked();
    void on_promedio_currentIndexChanged(int index);
    void on_frecuencia_valueChanged(double value);
    void on_factor_currentIndexChanged(int index);
    void on_estado_clicked();
    void on_activar_BMI_toggled(bool checked);
    void on_frec_BMI_valueChanged(double arg1);
    void on_rango_acc_currentIndexChanged(int index);
    void on_rango_giro_currentIndexChanged(int index);

private:
    // funciones privadas
    void processError(const QString &s);
    void activateRunButton();

    void set_max_oversample();

private:
    //Componentes privados
    Ui::MainUserGUI *ui;
    int transactionCount;
    QMessageBox ventanaPopUp;
    TivaRemoteLink tiva; //Objeto para gestionar la comunicacion de mensajes con el microcontrolador

    // Grafica ADC unipolar
    double g1xVal[1024]; //valores eje X
    double g1yVal[6][1024]; //valores ejes Y
    QwtPlotCurve *g1Channels[6]; //Curvas
    QwtPlotGrid  *g1m_Grid; //Cuadricula

    // Grafica ADC diferencial
    double g2xVal[1024]; //valores eje X
    double g2yVal[3][1024]; //valores ejes Y
    QwtPlotCurve *g2Channels[3]; //Curvas
    QwtPlotGrid  *g2m_Grid; //Cuadricula

    // Variables graficas ACC
    //valores eje X
    double xVal_acc[1024];
    //valores ejes Y
    double yVal_acc[3][1024];
    //Curvas
    QwtPlotCurve *curva_acc[3];

    // Variables graficas GYRO
    //valores eje X
    double xVal_gyro[1024];
    //valores ejes Y
    double yVal_gyro[3][1024];
    //Curvas
    QwtPlotCurve *curva_gyro[3];

    uint8_t RANGE_ACC;
    uint16_t RANGE_GYRO;
};

#endif // GUIPANEL_H
