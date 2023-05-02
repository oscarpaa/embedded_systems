#include "mainusergui.h"
#include "ui_mainusergui.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QMessageBox>      // Se deben incluir cabeceras a los componentes que se vayan a crear en la clase
// y que no estén incluidos en el interfaz gráfico. En este caso, la ventana de PopUp <QMessageBox>
// que se muestra al recibir un PING de respuesta

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos


MainUserGUI::MainUserGUI(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::MainUserGUI)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Interfaz de Control TIVA")); // Título de la ventana

    // Inicializa la lista de puertos serie
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        // La descripción realiza un FILTRADO que  nos permite que SOLO aparezcan los interfaces tipo USB serial con el identificador de fabricante
        // y producto de la TIVA
        // NOTA!!: SI QUIERES REUTILIZAR ESTE CODIGO PARA OTRO CONVERSOR USB-Serie, cambia los valores VENDOR y PRODUCT por los correspondientes
        // o cambia la condicion por "if (true) para listar todos los puertos serie"
        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    }

    //Inicializa algunos controles
    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio
    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que

    //Inicializa la ventana de PopUp que se muestra cuando llega la respuesta al PING
    ventanaPopUp.setIcon(QMessageBox::Information);
    ventanaPopUp.setText(tr("Status: RESPUESTA A PING RECIBIDA")); //Este es el texto que muestra la ventana
    ventanaPopUp.setStandardButtons(QMessageBox::Ok);
    ventanaPopUp.setWindowTitle(tr("Evento"));
    ventanaPopUp.setParent(this,Qt::Popup);

    // Inicializo Grafica ADC unipolar
    ui->graficaADC->setTitle("Canales ADC unipolar");;
    ui->graficaADC->setAxisTitle(QwtPlot::xBottom, "Tiempo");
    ui->graficaADC->setAxisTitle(QwtPlot::yLeft, "Voltaje");
    ui->graficaADC->setAxisScale(QwtPlot::yLeft, 0, 3.3);
    ui->graficaADC->setAxisScale(QwtPlot::xBottom, 0, 1024.0);

    // Formateo de la curva
    for(int i = 0; i < 6; i++){
        g1Channels[i] = new QwtPlotCurve();
        g1Channels[i]->attach(ui->graficaADC);
    }

    //Inicializacion de los valores básicos
    for(int i = 0; i < 1024; i++){
        g1xVal[i]=i;
        for (int j = 0; j < 6; j++) {
            g1yVal[j][i] = 0;
        }
    }
    for (int i = 0; i < 6; i++) {
        g1Channels[i]->setRawSamples(g1xVal,g1yVal[i],1024);
    }

    g1Channels[0]->setPen(QPen(Qt::red)); // Color de la curva
    g1Channels[1]->setPen(QPen(Qt::green));
    g1Channels[2]->setPen(QPen(Qt::blue));
    g1Channels[3]->setPen(QPen(Qt::cyan));
    g1Channels[4]->setPen(QPen(Qt::magenta));
    g1Channels[5]->setPen(QPen(Qt::yellow));

    g1m_Grid = new QwtPlotGrid();     // Rejilla de puntos
    g1m_Grid->attach(ui->graficaADC);    // que se asocia al objeto QwtPl
    ui->graficaADC->setAutoReplot(false); //Desactiva el autoreplot (mejora la eficiencia)
    // Fin Grafica ADC unipolar

    // Inicializo Grafica ADC diferencial
    ui->graficaADC_2->setTitle("Canales ADC diferencial");;
    ui->graficaADC_2->setAxisTitle(QwtPlot::xBottom, "Tiempo");
    ui->graficaADC_2->setAxisTitle(QwtPlot::yLeft, "Voltaje");
    ui->graficaADC_2->setAxisScale(QwtPlot::yLeft, -3.3, 3.3);
    ui->graficaADC_2->setAxisScale(QwtPlot::xBottom, 0, 1024.0);

    // Formateo de la curva
    for(int i = 0; i < 3; i++){
        g2Channels[i] = new QwtPlotCurve();
        g2Channels[i]->attach(ui->graficaADC_2);
    }

    //Inicializacion de los valores básicos
    for(int i = 0; i < 1024; i++){
        g2xVal[i]=i;
        for (int j = 0; j < 3; j++) {
            g2yVal[j][i] = 0;
        }
    }
    for (int i = 0; i < 3; i++) {
        g2Channels[i]->setRawSamples(g2xVal,g2yVal[i],1024);
    }

    g2Channels[0]->setPen(QPen(Qt::red)); // Color de la curva
    g2Channels[1]->setPen(QPen(Qt::green));
    g2Channels[2]->setPen(QPen(Qt::blue));

    g2m_Grid = new QwtPlotGrid();     // Rejilla de puntos
    g2m_Grid->attach(ui->graficaADC_2);    // que se asocia al objeto QwtPl
    ui->graficaADC_2->setAutoReplot(false); //Desactiva el autoreplot (mejora la eficiencia)
    // Fin Grafica ADC diferencial

    // Inicializo Grafica ACC
    ui->graficaAcc->setTitle("Aceleración");;
    ui->graficaAcc->setAxisTitle(QwtPlot::xBottom, "Tiempo");
    ui->graficaAcc->setAxisTitle(QwtPlot::yLeft, "Aceleracion (g)");
    ui->graficaAcc->setAxisScale(QwtPlot::yLeft, -2, 2);
    ui->graficaAcc->setAxisScale(QwtPlot::xBottom, 0, 128.0);

    for(int i = 0; i < 128; i++)
    {
        xVal_acc[i] = i;
        yVal_acc[0][i] = 0;
        yVal_acc[1][i] = 0;
        yVal_acc[2][i] = 0;
    }

    // Formato de las curvas
    for(int i = 0; i < 3; i++)
    {
        curva_acc[i] = new QwtPlotCurve();
        curva_acc[i]->attach(ui->graficaAcc);

        // Inicializacion de los datos apuntando a bloques de memoria,  por eficiencia
        curva_acc[i]->setRawSamples(xVal_acc, yVal_acc[i], 128);
    }

    // Colores de las curvas
    curva_acc[0]->setPen(QPen(Qt::red));
    curva_acc[1]->setPen(QPen(Qt::green));
    curva_acc[2]->setPen(QPen(Qt::yellow));

    //Desactiva el autoreplot (mejora la eficiencia)
    ui->graficaAcc->setAutoReplot(false);
    // Fin Grafica ACC

    // Inicializo Grafica GYRO
    ui->graficaGiro->setTitle("Velocidad angular");;
    ui->graficaGiro->setAxisTitle(QwtPlot::xBottom, "Tiempo");
    ui->graficaGiro->setAxisTitle(QwtPlot::yLeft, "Velocidad angular (º/s)");
    ui->graficaGiro->setAxisScale(QwtPlot::yLeft, -125, 125);
    ui->graficaGiro->setAxisScale(QwtPlot::xBottom, 0, 128.0);

    for(int i = 0; i < 128; i++)
    {
        xVal_gyro[i] = i;
        yVal_gyro[0][i] = 0;
        yVal_gyro[1][i] = 0;
        yVal_gyro[2][i] = 0;
    }

    // Formato de las curvas
    for(int i = 0; i < 3; i++)
    {
        curva_gyro[i] = new QwtPlotCurve();
        curva_gyro[i]->attach(ui->graficaGiro);

        // Inicializacion de los datos apuntando a bloques de memoria,  por eficiencia
        curva_gyro[i]->setRawSamples(xVal_gyro, yVal_gyro[i], 128);
    }

    // Colores de las curvas
    curva_gyro[0]->setPen(QPen(Qt::red));
    curva_gyro[1]->setPen(QPen(Qt::green));
    curva_gyro[2]->setPen(QPen(Qt::yellow));

    //Desactiva el autoreplot (mejora la eficiencia)
    ui->graficaGiro->setAutoReplot(false);
    // Fin Grafica GIRO

    //Conexion de signals de los widgets del interfaz con slots propios de este objeto
    connect(ui->rojo,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));
    connect(ui->verde,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));
    connect(ui->azul,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));

    connect(ui->gpio,SIGNAL(clicked(bool)),this,SLOT(cambiaMODO()));
    connect(ui->pwm,SIGNAL(clicked(bool)),this,SLOT(cambiaMODO()));

    connect(ui->sondeo,SIGNAL(clicked(bool)),this,SLOT(modoSwitches()));
    connect(ui->interrupt,SIGNAL(clicked(bool)),this,SLOT(modoSwitches()));

    connect(ui->manual,SIGNAL(clicked(bool)),this,SLOT(modoAdquisicion()));
    connect(ui->period,SIGNAL(clicked(bool)),this,SLOT(modoAdquisicion()));

    /* Conexiones ACME */
    connect(ui->GPIO4, SIGNAL(toggled(bool)), this, SLOT(controlACME()));
    connect(ui->GPIO5, SIGNAL(toggled(bool)), this, SLOT(controlACME()));
    connect(ui->GPIO6, SIGNAL(toggled(bool)), this, SLOT(controlACME()));
    connect(ui->GPIO7, SIGNAL(toggled(bool)), this, SLOT(controlACME()));

    //Conectamos Slots del objeto "Tiva" con Slots de nuestra aplicacion (o con widgets)
    connect(&tiva,SIGNAL(statusChanged(int,QString)),this,SLOT(tivaStatusChanged(int,QString)));
    connect(&tiva,SIGNAL(messageReceived(uint8_t,QByteArray)),this,SLOT(messageReceived(uint8_t,QByteArray)));
}

MainUserGUI::~MainUserGUI() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void MainUserGUI::on_serialPortComboBox_currentIndexChanged(const QString &arg1)
{
    activateRunButton();
}

// Funcion auxiliar de procesamiento de errores de comunicación
void MainUserGUI::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void MainUserGUI::activateRunButton()
{
    ui->runButton->setEnabled(true);
}


// SLOT asociada a pulsación del botón RUN
void MainUserGUI::on_runButton_clicked()
{
    //Intenta arrancar la comunicacion con la TIVA
    tiva.startRemoteLink( ui->serialPortComboBox->currentText());
}



//Slots Asociado al boton que limpia los mensajes del interfaz
void MainUserGUI::on_pushButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

void MainUserGUI::on_colorWheel_colorChanged(const QColor &arg1)
{
    MESSAGE_LED_PWM_PARAMETER parameter;

    parameter.red = arg1.red();
    parameter.green = arg1.green();
    parameter.blue = arg1.blue();

    tiva.sendMessage(MESSAGE_LED_PWM,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::on_Knob_valueChanged(double value)
{
    MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER parameter;
    parameter.rIntensity = value;

    tiva.sendMessage(MESSAGE_LED_PWM_BRIGHTNESS,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::set_max_oversample()
{
    bool encontrado = false;
    int totalItems = ui->promedio->count();
    int MAX_PRO = totalItems;

    if (ui->period->isChecked())
    {
        double t_sample = 1/(6 * (ui->frecuencia->value()) * pow(10,ui->factor->currentIndex()));
        double t_promedio;

        int i = 0;
        while (i < 7 && !encontrado)
        {
            t_promedio = 1e-6*pow(2,i);


            if (t_sample < t_promedio)
            {
                MAX_PRO = i;
                encontrado = true;
            }

            i++;
        }
    }

    if (encontrado)
    {
        for (int i = totalItems - 1; i >= MAX_PRO ; i--)
        {
            ui->promedio->removeItem(i);
        }
    }
    else
    {
        for (int i = MAX_PRO; i < 7; i++)
        {
            if (i == 0)
                ui->promedio->addItem(QString::number(i));
            else
                ui->promedio->addItem(QString::number(pow(2,i)));
        }
    }
}

void MainUserGUI::on_frecuencia_valueChanged(double value)
{
    MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER parameter;

    set_max_oversample();

    if (ui->period->isChecked())
    {

        ui->ADCButton->setHidden(true);
        parameter.frecuency = value * pow(10,ui->factor->currentIndex());

        tiva.sendMessage(MESSAGE_ADC_AUTO_FRECUENCY,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
}


void MainUserGUI::on_factor_currentIndexChanged(int index)
{
    MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER parameter;

    if (index == 3)
    {
        ui->frecuencia->setScale(1,4);
    }
    else
    {
        ui->frecuencia->setScale(1,10);
    }

    set_max_oversample();

    if (ui->period->isChecked())
    {
        ui->ADCButton->setHidden(true);
        parameter.frecuency = (ui->frecuencia->value()) * pow(10,index);

        tiva.sendMessage(MESSAGE_ADC_AUTO_FRECUENCY,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
}

void MainUserGUI::modoAdquisicion()
{
    MESSAGE_ADC_AUTO_PARAMETER parameter;

    set_max_oversample();

    if (ui->manual->isChecked())
    {
        ui->ADCButton->setHidden(false);
        tiva.sendMessage(MESSAGE_ADC_AUTO_DISABLE,NULL,0);
    }
    else if (ui->period->isChecked())
    {
        ui->ADCButton->setHidden(true);
        parameter.frecuency = (ui->frecuencia->value()) * pow(10,ui->factor->currentIndex());

        tiva.sendMessage(MESSAGE_ADC_AUTO,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
}

void MainUserGUI::on_ADCButton_clicked()
{
    tiva.sendMessage(MESSAGE_ADC_SAMPLE,NULL,0);
}

void MainUserGUI::on_pingButton_clicked()
{
    tiva.sendMessage(MESSAGE_PING,NULL,0);
}

void MainUserGUI::modoSwitches()
{
    if (ui->sondeo->isChecked())
    {
        ui->estado->setHidden(false);
        tiva.sendMessage(MESSAGE_SWITCHES_INTERRUPT_DISABLE,NULL,0);
    }
    else
    {
        ui->estado->setHidden(true);
        tiva.sendMessage(MESSAGE_SWITCHES_INTERRUPT,NULL,0);
    }
}

void MainUserGUI::on_estado_clicked()
{
    tiva.sendMessage(MESSAGE_SWITCHES_POLL,NULL,0);
}

void MainUserGUI::cambiaMODO()
{
    MESSAGE_LED_MODE_PARAMETER parameter;
    parameter.mode = ui->gpio->isChecked();

    tiva.sendMessage(MESSAGE_LED_MODE,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));

    if (parameter.mode)
    {
        cambiaLEDs();
    }
}

void MainUserGUI::cambiaLEDs(void)
{

    MESSAGE_LED_GPIO_PARAMETER parameter;

    parameter.leds.red=ui->rojo->isChecked();
    parameter.leds.green=ui->verde->isChecked();
    parameter.leds.blue=ui->azul->isChecked();

    tiva.sendMessage(MESSAGE_LED_GPIO,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::on_promedio_currentIndexChanged(int index)
{
    MESSAGE_OVERSAMPLE_PARAMETER parameter;
    if (!index)
    {
        parameter.factor = index;
    }
    else
    {
        parameter.factor = pow(2,index);
    }

    tiva.sendMessage(MESSAGE_OVERSAMPLE,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::controlACME()
{
    MESSAGE_ACME_PARAMETER parameter;

    parameter.GPIO = (ui->GPIO7->isChecked() << 7) | (ui->GPIO6->isChecked() << 6) | (ui->GPIO5->isChecked() << 5) | (ui->GPIO4->isChecked() << 4);

    tiva.sendMessage(MESSAGE_ACME,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::on_activar_BMI_toggled(bool checked)
{
    MESSAGE_BMI_ENABLE_PARAMETER parameter;

    if (checked)
    {
        parameter.frecuencia = ui->frec_BMI->value();
        tiva.sendMessage(MESSAGE_BMI_ENABLE,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
    else
    {
        tiva.sendMessage(MESSAGE_BMI_DISABLE,NULL,0);
    }
}

void MainUserGUI::on_frec_BMI_valueChanged(double arg1)
{
    MESSAGE_BMI_FRECUENCY_PARAMETER parameter;

    parameter.frecuencia = arg1;
    tiva.sendMessage(MESSAGE_BMI_FRECUENCY,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::on_rango_acc_currentIndexChanged(int index)
{
    MESSAGE_BMI_RANGE_ACC_PARAMETER parameter;

    parameter.range_acc = pow(2,index + 1);
    ui->graficaAcc->setAxisScale(QwtPlot::yLeft, -parameter.range_acc, parameter.range_acc);

    tiva.sendMessage(MESSAGE_BMI_RANGE_ACC,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

void MainUserGUI::on_rango_giro_currentIndexChanged(int index)
{
    MESSAGE_BMI_RANGE_GYRO_PARAMETER parameter;

    parameter.range_gyro = 125 * pow(2,index);
    ui->graficaGiro->setAxisScale(QwtPlot::yLeft, -parameter.range_gyro, parameter.range_gyro);

    tiva.sendMessage(MESSAGE_BMI_RANGE_GYRO,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
}

//**** Slot asociado a la recepción de mensajes desde la TIVA ********/
//Está conectado a una señale generada por el objeto TIVA de clase QTivaRPC (se conecta en el constructor de la ventana, más arriba en este fichero))
//Se pueden añadir los que casos que quieran para completar la aplicación

void MainUserGUI::messageReceived(uint8_t message_type, QByteArray datos)
{
    switch(message_type) // Segun el comando tengo que hacer cosas distintas
    {
        /** A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS COMANDOS QUE SE ENVIEN DESDE LA TIVA **/
        case MESSAGE_PING:
        {   //Recepcion de la respuesta al ping desde la TIVA
            // Algunos comandos no tiene parametros --> No se extraen
            ventanaPopUp.setStyleSheet("background-color: lightgrey");
            ventanaPopUp.setModal(true); //CAMBIO: Se sustituye la llamada a exec(...) por estas dos.
            ventanaPopUp.show();
        }
        break;

        case MESSAGE_REJECTED:
        {   //Recepcion del mensaje MESSAGE_REJECTED (La tiva ha rechazado un mensaje que le enviamos previamente)
            MESSAGE_REJECTED_PARAMETER parametro;
            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                // Muestra en una etiqueta (statuslabel) del GUI el mensaje
            ui->statusLabel->setText(tr("Status: Comando rechazado,%1").arg(parametro.command));
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_ADC_SAMPLE:
        {    // Este caso trata la recepcion de datos del ADC desde la TIVA
            MESSAGE_ADC_SAMPLE_PARAMETER parametro;
            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->lcdCh1->display(((double)parametro.chan[0])*3.3/4096.0);
                ui->lcdCh2->display(((double)parametro.chan[1])*3.3/4096.0);
                ui->lcdCh3->display(((double)parametro.chan[2])*3.3/4096.0);
                ui->lcdCh4->display(((double)parametro.chan[3])*3.3/4096.0);
                ui->lcdCh5->display(((double)parametro.chan[4])*3.3/4096.0);
                ui->lcdCh6->display(((double)parametro.chan[5])*3.3/4096.0);
                ui->temp->display(147.5-((75*3.3*(double)parametro.chan[6])/4096.0));
            }
            else
            {   //Si el tamanho de los datos no es correcto emito la senhal statusChanged(...) para reportar un error
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }

        }
        break;

        case MESSAGE_ADC_AUTO_SAMPLE16:
        {
            MESSAGE_ADC_AUTO_SAMPLE16_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->lcdCh1->display(((double)parametro.chan[0][15])*3.3/4096.0);
                ui->lcdCh2->display(((double)parametro.chan[1][15])*3.3/4096.0);
                ui->lcdCh3->display(((double)parametro.chan[2][15])*3.3/4096.0);
                ui->lcdCh4->display(((double)parametro.chan[3][15])*3.3/4096.0);
                ui->lcdCh5->display(((double)parametro.chan[4][15])*3.3/4096.0);
                ui->lcdCh6->display(((double)parametro.chan[5][15])*3.3/4096.0);

                //Recalcula los valores de la grafica
                for (int i = 1023; i >= 16; --i) {
                    for (int j = 0; j < 6; ++j) {
                        g1yVal[j][i] = g1yVal[j][i-16];
                    }
                }

                for (int i = 0; i < 16; ++i) {
                    for (int j = 0; j < 6; ++j) {
                         g1yVal[j][i] = (((double)parametro.chan[j][i])*3.3/4096.0);
                    }
                }

                ui->graficaADC->replot(); //Refresca la grafica una vez actualizados los valores
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_ADC_AUTO_SAMPLE32:
        {
            MESSAGE_ADC_AUTO_SAMPLE32_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                //Recalcula los valores de la grafica
                for (int i = 1023; i >= 32; --i) {
                    for (int j = 0; j < 3; ++j) {
                        g2yVal[j][i] = g2yVal[j][i-32];
                    }
                }

                for (int i = 0; i < 32; ++i) {
                    for (int j = 0; j < 3; ++j) {
                         g2yVal[j][i] = (((double)parametro.chan[j][i])*2*3.3/4096.0) -3.3;
                    }
                }

                ui->graficaADC_2->replot(); //Refresca la grafica una vez actualizados los valores
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_SWITCHES_POLL:
        {
            MESSAGE_POLL_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->sw1->setChecked(parametro.sw1);
                ui->sw2->setChecked(parametro.sw2);
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_SWITCHES_INTERRUPT:
        {
            MESSAGE_INTERRUPT_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->sw1->setChecked(parametro.sw1);
                ui->sw2->setChecked(parametro.sw2);
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_ACME:
        {
            MESSAGE_ACME_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->GPIO0->setChecked(parametro.GPIO & (1<<0));
                ui->GPIO1->setChecked(parametro.GPIO & (1<<1));
                ui->GPIO2->setChecked(parametro.GPIO & (1<<2));
                ui->GPIO3->setChecked(parametro.GPIO & (1<<3));
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        case MESSAGE_BMI_SAMPLE:
        {
            MESSAGE_BMI_SAMPLE_PARAMETER parametro;
            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                uint8_t RANGE_ACC = pow(2,ui->rango_acc->currentIndex() + 1);
                uint16_t RANGE_GYRO = 125 * pow(2,ui->rango_giro->currentIndex());

                //Recalcula los valores de la grafica
                for (int i = 127; i >= 1; --i) {
                    for (int j = 0; j < 3; ++j) {
                        yVal_acc[j][i] = yVal_acc[j][i-1];
                        yVal_gyro[j][i] = yVal_gyro[j][i-1];
                    }
                }

                for (int j = 0; j < 3; ++j) {
                    yVal_acc[j][0] = (((double)parametro.acc[j])*RANGE_ACC/32768.0);
                    yVal_gyro[j][0] = (((double)parametro.gyro[j])*RANGE_GYRO/32768.0);
                }

                ui->graficaAcc->replot(); //Refresca la grafica una vez actualizados los valores
                ui->graficaGiro->replot(); //Refresca la grafica una vez actualizados los valores
            }
            else
            {
                ui->statusLabel->setText(tr("Status: MSG %1, recibí %2 B y esperaba %3").arg(message_type).arg(datos.size()).arg(sizeof(parametro)));
            }
        }
        break;

        default:
            //Notifico que ha llegado un tipo de mensaje desconocido
            ui->statusLabel->setText(tr("Status: Recibido mensaje desconocido %1").arg(message_type));
        break; //Innecesario
    }
}

//Este Slot lo hemos conectado con la señal statusChange de la TIVA, que se activa cuando
//El puerto se conecta o se desconecta y cuando se producen determinados errores.
//Esta función lo que hace es procesar dichos eventos
void MainUserGUI::tivaStatusChanged(int status,QString message)
{
    switch (status)
    {
        case TivaRemoteLink::TivaConnected:

            //Caso conectado..
            // Deshabilito el boton de conectar
            ui->runButton->setEnabled(false);

            // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
            ui->statusLabel->setText(tr("Ejecucion, conectado al puerto %1.").arg(ui->serialPortComboBox->currentText()));

            //   Y se habilitan los controles deshabilitados
            ui->pingButton->setEnabled(true);

        break;

        case TivaRemoteLink::TivaIsDisconnected:
            //Caso desconectado..
            // Rehabilito el boton de conectar
             ui->runButton->setEnabled(false);
             ui->statusLabel->setText(message);
        break;
        case TivaRemoteLink::FragmentedPacketError:
        case TivaRemoteLink::CRCorStuffError:
            //Errores detectados en la recepcion de paquetes
            ui->statusLabel->setText(message);
        break;
        default:
            //Otros errores (por ejemplo, abriendo el puerto)
            processError(tiva.getLastErrorMessage());
    }
}

