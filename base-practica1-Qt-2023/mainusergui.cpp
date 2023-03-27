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

    // --- Inicializo Grafica ADC ---
    ui->graficaADC->setTitle("Canales ADC");;
    ui->graficaADC->setAxisTitle(QwtPlot::xBottom, "Tiempo");
    ui->graficaADC->setAxisTitle(QwtPlot::yLeft, "Voltaje");
    ui->graficaADC->setAxisScale(QwtPlot::yLeft, 0, 3.3);
    ui->graficaADC->setAxisScale(QwtPlot::xBottom, 0, 1024.0);

    // Formateo de la curva
    for(int i = 0; i < 6; i++){
        Channels[i] = new QwtPlotCurve();
        Channels[i]->attach(ui->graficaADC);
    }

    //Inicializacion de los valores básicos
    for(int i = 0; i < 1024; i++){
        xVal[i]=i;
        for (int j = 0; j < 6; j++) {
            yVal[j][i] = 0;
        }
    }
    for (int i = 0; i < 6; i++) {
        Channels[i]->setRawSamples(xVal,yVal[i],1024);
    }

    Channels[0]->setPen(QPen(Qt::red)); // Color de la curva
    Channels[1]->setPen(QPen(Qt::green));
    Channels[2]->setPen(QPen(Qt::blue));
    Channels[3]->setPen(QPen(Qt::cyan));
    Channels[4]->setPen(QPen(Qt::magenta));
    Channels[5]->setPen(QPen(Qt::yellow));

    m_Grid = new QwtPlotGrid();     // Rejilla de puntos
    m_Grid->attach(ui->graficaADC);    // que se asocia al objeto QwtPl
    ui->graficaADC->setAutoReplot(false); //Desactiva el autoreplot (mejora la eficiencia)
    // --- Inicializo Grafica ADC ---

    //Conexion de signals de los widgets del interfaz con slots propios de este objeto
    connect(ui->rojo,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));
    connect(ui->verde,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));
    connect(ui->azul,SIGNAL(toggled(bool)),this,SLOT(cambiaLEDs()));

    connect(ui->gpio,SIGNAL(toggled(bool)),this,SLOT(cambiaMODO()));
    connect(ui->sondeo,SIGNAL(toggled(bool)),this,SLOT(modoSwitches()));
    connect(ui->estado,SIGNAL(clicked(bool)),this,SLOT(leeSwitches()));
    connect(ui->manual,SIGNAL(toggled(bool)),this,SLOT(modoAdquisicion()));

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

void MainUserGUI::on_frecuencia_valueChanged(double value)
{
    MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER parameter;
    if (ui->period->isChecked())
    {
        parameter.frecuency = value * pow(10,ui->multiplica->currentIndex());

        tiva.sendMessage(MESSAGE_ADC_AUTO_FRECUENCY,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
}


void MainUserGUI::on_multiplica_currentIndexChanged(int index)
{
    MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER parameter;
    if (ui->period->isChecked())
    {
        parameter.frecuency = (ui->frecuencia->value()) * pow(10,index);

        tiva.sendMessage(MESSAGE_ADC_AUTO_FRECUENCY,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
    }
}

void MainUserGUI::modoAdquisicion()
{
    MESSAGE_ADC_AUTO_ENABLE_PARAMETER parameter;
    if (ui->manual->isChecked())
    {
        ui->ADCButton->setHidden(false);
        tiva.sendMessage(MESSAGE_ADC_AUTO_DISABLE,NULL,0);
    }
    else
    {
        ui->ADCButton->setHidden(true);
        parameter.frecuency = (ui->frecuencia->value()) * pow(10,ui->multiplica->currentIndex());

        tiva.sendMessage(MESSAGE_ADC_AUTO_ENABLE,QByteArray::fromRawData((char *)&parameter,sizeof(parameter)));
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

void MainUserGUI::leeSwitches()
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

void MainUserGUI::on_factor_currentIndexChanged(int index)
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

        case MESSAGE_ADC_AUTO_SAMPLING_6X16:
        {
            MESSAGE_ADC_AUTO_SAMPLING_6X16_PARAMETER parametro;

            if (check_and_extract_command_param(datos.data(), datos.size(), &parametro, sizeof(parametro))>0)
            {
                ui->lcdCh1->display(((double)parametro.chan[0][15])*3.3/4096.0);
                ui->lcdCh2->display(((double)parametro.chan[1][15])*3.3/4096.0);
                ui->lcdCh3->display(((double)parametro.chan[2][15])*3.3/4096.0);
                ui->lcdCh4->display(((double)parametro.chan[3][15])*3.3/4096.0);
                ui->lcdCh5->display(((double)parametro.chan[4][15])*3.3/4096.0);
                ui->lcdCh6->display(((double)parametro.chan[5][15])*3.3/4096.0);

                //Recalcula los valores de la grafica
                for (int i = 0; i < 1024-16; ++i) {
                    for (int j = 0; j < 6; ++j) {
                         yVal[j][i] = yVal[j][i+16];
                    }
                }

                for (int i = 0; i < 16; ++i) {
                    for (int j = 0; j < 6; ++j) {
                         yVal[j][1024-16+i] = (((double)parametro.chan[j][i])*3.3/4096.0);
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

