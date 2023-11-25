//**************************Librerias**************************
// Incluimos las librerías necesarias para los módulos que vamos a utilizar.
// Pantalla OLED
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// Sensores
#include <DHT.h>
#include "DFRobot_SHT20.h"

#include "RTClib.h"
//#include <avr/wdt.h>

//***********************Iniciar Modulos**********************
// Inicializamos los módulos que vamos a utilizar.

// Pantalla OLED
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

// Sensor SHT20
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

// Sensor DHT11
#define DHTPIN 2       // Pin digital conectado al sensor DHT
#define DHTTYPE DHT11  // Tipo de sensor DHT (DHT 11)
DHT dht(DHTPIN, DHTTYPE);

// Configurar el RTC (Reloj en tiempo real)
RTC_DS1307 rtcAerofresas;


//*********************Iniciar Variables*********************
// Aquí declaramos las variables que vamos a utilizar para almacenar datos.
//En el fragmento de código que has proporcionado, estas líneas se utilizan para declarar y inicializar cuatro variables de tipo float.
//Cada una de estas variables representa valores de temperatura y humedad provenientes de diferentes sensores o fuentes.
//Aquí está la explicación más detallada:
//Por ejemplo:
//float tempIn = 0;: Esta línea declara y define una variable llamada tempIn,
//que se utiliza para almacenar el valor de la temperatura interior.
//La variable se declara como tipo float, lo que significa que puede almacenar números decimales (con coma flotante).
//Se inicializa con un valor de 0, lo que asegura que la variable tenga un valor válido desde el principio.

int tempIn = 0;  //variable para almacenar el valor de la temperatura interior e inicia en cero
int tempEx = 0;  //variable para almacenar el valor de la temperatura exterior e inicia en cero
int humdIn = 0;  //variable para almacenar el valor de la humedad interior e inicia en cero
int humdEx = 0;  //variable para almacenar el valor de la humedad exterior e inicia en cero

//Iniciar variable tipo "unsigned long"
//"unsigned" significa que esta variable solo puede almacenar valores positivos o cero
//"long" tiene un rango tan amplio, es adecuado para medir intervalos de tiempo largos sin preocuparse por desbordamientos
unsigned long tiempoActual = 0;  // Tiempo actual en milisegundos
unsigned long tiempoPrevio = 0;  // Tiempo previo para comparación y control de intervalo
const long intervalo = 1000;     // Intervalo de ejecución en milisegundos (1 segundo)

//Se crean en variable byte porque su numero no va ha superar al 256 que es numero max que puede almacenar
//Algunas de estas variables son Const debido a que no van a cambiar en ningun momento en el codigo, osea no se les asignara otro valor
const byte tiempoOLED = 5;    // Intervalo de actualización de la pantalla OLED en segundos
const byte tiempoWifi = 10;  // Intervalo de envío de datos por WiFi en segundos
byte tiempoVent = 10;  // Intervalo de control del ventilador en segundos
byte tiempoBombAir = 10;     // Intervalo de control de aireador en segundos
byte tiempoHumi = 30;

byte tempoOLED = 5;       // Contador de tiempo para la actualización de OLED
byte tempoWifi = 10;     // Contador de tiempo para envío de datos por WiFi
byte tempoVent = 10;     // Contador de tiempo para control de ventilador
byte tempoBombAir = 0;  // Contador de tiempo para control de aireador
byte tempoHumi = 30;

// Actuadores
// Definimos los pines para cada actuador
//Los números (8, 7 y 6): Estos son los números de los pines físicos en la placa Arduino o microcontrolador a los que se han conectado los dispositivos.
//Por ejemplo, el ventilador se conecta al pin 8, el humificador al pin 7 y la bomba de aire al pin 6.

const byte vent = 9;  // Constante que representa el pin del ventilador (pin 9_PWM)
const byte humi = 7;  // Constante que representa el pin del humificador (pin 7)
const byte air = 6;   // Constante que representa el pin de la bomba de aire (pin 6)


// Estado de los actuadores (encendido/apagado)
//"bool" son un tipo de dato en la programación que puede tener uno de dos valores posibles: verdadero (true) o falso (false).
//En función de ciertas condiciones, podrían cambiarse a true para encender los dispositivos o a false para apagarlos

bool on_vent = false;  // Variable para representar el estado del ventilador (apagado inicialmente)
bool on_humi = false;  // Variable para representar el estado del humificador (apagado inicialmente)
bool on_air = false;   // Variable para representar el estado de la bomba de aire (apagada inicialmente)

// Histeresis del Riego para evitar cambios rápidos
//const byte hys = 1;        // Histeresis del riego
const byte humIdeal = 95;  // Humedad ideal

// Variables para el manejo de datos desde Serial
char inputBuffer[100];
int inputIndex = 0;
bool newData = false;
String response = "";

byte hora = 0;

// Parte del código que se ejecuta una sola vez en el inicio (setup)
void setup() {
  Serial.begin(9600);
  Serial.println("Iniciar Aerofresas");
  // Inicialización de la pantalla OLED
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);

  // Inicialización del sensor DHT11
  dht.begin();

  // Inicialización del sensor SHT20
  sht20.initSHT20();
  sht20.checkSHT20();

  // Verificamos el estado del RTC (Reloj en tiempo real)
  if (!rtcAerofresas.begin() || !rtcAerofresas.isrunning()) {
    oled.print("RTC Desconectado");
    delay(2000);
    // Ejemplos de ajuste de hora manualmente o desde la fecha de compilación
    // rtcAerofresas.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // rtcAerofresas.adjust(DateTime(2019, 9, 28, 1, 19, 0));
  }
  
  // Configuración de pines para actuadores y establecimiento de estados iniciales
  pinMode(vent, OUTPUT);
  pinMode(humi, OUTPUT);
  pinMode(air, OUTPUT);

  digitalWrite(vent, LOW);
  digitalWrite(humi, LOW);
  digitalWrite(air, LOW);

  // Inicialización de la pantalla de bienvenida
  aerofresas();
  display();

  // Habilitar el Watchdog Timer con un tiempo de temporización de 2 segundos
  //wdt_enable(WDTO_4S);
}

void aerofresas(){
  oled.setCursor(4, 3);
  oled.set2X();
  oled.print("AEROFRESAS");
  delay(5000);
}

void display(){
  char buffer[20];  // Crear un buffer para almacenar la cadena formateada
  // Borrar el contenido previo de la pantalla OLED
  oled.clear();
  oled.set1X();
  sprintf(buffer, "Temp_Int    Temp_Ext");
  oled.println(buffer);
  oled.setCursor(0, 3);
  sprintf(buffer, "Humd_Int    Humd_Ext");
  oled.println(buffer);
}

// Loop principal que se ejecuta continuamente
void loop() {
  // Capturamos el tiempo actual en milisegundos
  tiempoActual = millis();

  // Manejo de desbordamiento de tiempo
  if (tiempoActual < tiempoPrevio) {
    tiempoPrevio = tiempoActual;
  }

  // Verificamos si ha pasado el intervalo de tiempo
  if (tiempoActual - tiempoPrevio >= intervalo) {
    tiempoPrevio = millis();  // Reiniciamos el tiempo previo

    // Reiniciar el Watchdog Timer para evitar el reinicio automático
    //wdt_reset();

    // Incrementamos los contadores de tiempo
    tempoOLED++;
    tempoWifi++;
    tempoVent++;
    tempoBombAir++;
    tempoHumi++;
    actualizarHora();

    // Control del Humificador
    if (tempoHumi >= tiempoHumi) {
      on_humi = !on_humi;
      controlActuador(humi, on_humi);
      // Ajuste de intervalo dependiendo del estado del humificador
      if(hora<7 || hora>=20){
        if (humdIn >= humIdeal)                                 tiempoHumi = on_humi ? 10 : 100;
        else if (humdIn >= (humIdeal-10) && humdIn < humIdeal)  tiempoHumi = on_humi ? 10 : 70;
        else                                                    tiempoHumi = on_humi ? 10 : 40;
      }
      else {
        if (humdIn >= humIdeal)                                 tiempoHumi = on_humi ? 10 : 60;
        else if (humdIn >= (humIdeal-10) && humdIn < humIdeal)  tiempoHumi = on_humi ? 10 : 40;
        else                                                    tiempoHumi = on_humi ? 10 : 20;
      } 
      tempoHumi = 0;
      Serial.print("Humidificador: ");
      Serial.print(on_humi ? "Encendido" : "Apagado");
      Serial.print(",  tiempoHumi: ");
      Serial.println(tiempoHumi);
    }

    // Control del ventilador
    if (tempoVent >= tiempoVent) {
      on_vent = !on_vent;
      controlActuador(vent, on_vent);
      // Ajuste de intervalo dependiendo del estado del aireador
      tiempoVent = on_vent ? 10 : 20;
      //tiempoVent = tiempoHumi;
      tempoVent = 0;
      Serial.print("Ventilador: ");
      Serial.println(on_vent ? "Encendido" : "Apagado");
    }

        // Control del aireador
    if (tempoBombAir >= tiempoBombAir) {
      on_air = !on_air;
      controlActuador(air, on_air);
      // Ajuste de intervalo dependiendo del estado del aireador
      tiempoBombAir = on_air ? 10 : 120;
      tempoBombAir = 0;
      Serial.print("Bomba Aire: ");
      Serial.println(on_air ? "Encendido" : "Apagado");
    }

    // Control de la humedad
    //controlHumificador();

    // Actualización de la pantalla oled
    if (tempoOLED>= tiempoOLED) {
      leerSensor();
      printOLED();
      tempoOLED = 0;
      Serial.println("Actualizar pantalla OLED");
    }

    // Envío de datos por WiFi
    if (tempoWifi >= tiempoWifi) {
      sendWifiData();
      tempoWifi = 0;
      Serial.println("Datos enviados por Wi-Fi");
    }
  }

  while (Serial.available()) {
      char inChar = Serial.read();
      if (inChar == '{') {
        inputIndex = 0;
        newData = true;
      } else if (newData) {
        if (inChar == '}') {
          inputBuffer[inputIndex] = '\0'; // Null-terminate the string
          readDataToFirebase(inputBuffer); // Procesar los datos recibidos
          newData = false;
        } else {
          inputBuffer[inputIndex] = inChar;
          inputIndex++;
        }
      }
    }
}

// Enviar datos a Firebase
void readDataToFirebase(const char *response) {
  Serial.println(response);
}

/*// Función para controlar el humificador
void controlHumificador() {
  // Verificar si la humedad es mayor o igual al valor objetivo más la histeresis
  if (humdIn >= humIdeal + hys) {
    // Si la humedad es alta, se apaga el humificador
    on_humi = false;
    controlActuador(humi, on_humi);
  }
  // Verificar si la humedad es menor o igual al valor objetivo menos la histeresis
  else if (humdIn <= humIdeal - hys) {
    // Si la humedad es baja, se enciende el humificador
    on_humi = true;
    controlActuador(humi, on_humi);
  }
}*/

// Función para controlar los actuadores
//digitalWrite(pin, estado ? HIGH : LOW);: Esta línea utiliza la función digitalWrite() para establecer el estado del pin del actuador.
//El pin se especifica mediante el parámetro pin, y el estado se determina usando el operador ternario (? :).
//El operador ternario evalúa la condición estado.
//Si estado es true, entonces HIGH (que es equivalente a 5V o "encendido" en el contexto de un pin digital) se aplica al pin.
//Si estado es false, entonces LOW (que es equivalente a 0V o "apagado") se aplica al pin.
void controlActuador(byte pin, bool estado) {
  // Utiliza digitalWrite para establecer el estado del pin del actuador
  // El operador ternario (condición ? valor_si_verdadero : valor_si_falso) se utiliza aquí
  if(pin == vent) analogWrite(pin, estado ? 255 : 0);
  else digitalWrite(pin, estado ? HIGH : LOW);
}

void leerSensor() {
  // Leer las medidas de temperatura y humedad de los sensores internos y externos
  // Leer la temperatura y humedad internas del sensor SHT20
  tempIn = sht20.readTemperature();
  tempIn = constrain(tempIn, -40, 40);
  humdIn = sht20.readHumidity();
  humdIn = map(humdIn,0,103,0,99);
  humdIn = constrain(humdIn, 0, 99);
  // Leer la temperatura y humedad externas del sensor DHT11
  tempEx = dht.readTemperature();
  humdEx = dht.readHumidity();
  Serial.println("Leer Sensores");
}

// Función para actualizar la pantalla OLED
void printOLED() {
  char buffer[20];  // Crear un buffer para almacenar la cadena formateada
  // Borrar el contenido previo de la pantalla OLED
  oled.set2X();
  oled.setCursor(0, 1);
  sprintf(buffer, "%d%s", tempIn, "oC ");
  oled.println(buffer);
  oled.setCursor(72, 1);
  sprintf(buffer, "%d%s", tempEx, "oC ");
  oled.println(buffer);

  oled.setCursor(0, 4);
  sprintf(buffer, "%d%s", humdIn, "% ");
  oled.println(buffer);  
  oled.setCursor(72, 4);
  sprintf(buffer, "%d%s", humdEx, "% ");
  oled.println(buffer);
}

// Función para enviar datos por WiFi
void sendWifiData() {
  // Crea el objeto JSON manualmente
  String jsonData = "{";
  jsonData += "\"tempIn\": " + String(tempIn) + ",";
  jsonData += "\"humdIn\": " + String(humdIn) + ",";
  jsonData += "\"tempEx\": " + String(tempEx) + ",";
  jsonData += "\"humdEx\": " + String(humdEx) + ",";
  jsonData += "\"fanState\": " + String(on_vent ? "true" : "false") + ",";
  jsonData += "\"humState\": " + String(on_humi ? "true" : "false") + ",";
  jsonData += "\"airState\": " + String(on_air ? "true" : "false");
  jsonData += "}";

  // Envía los datos a través del puerto serial
  Serial.println(jsonData);
}

// Función para actualizar la hora en la pantalla OLED
void actualizarHora() {
  char buffer[20];  // Crear un buffer para almacenar la cadena formateada
  // Obtener la fecha y hora actual del módulo RTC
  DateTime now = rtcAerofresas.now();

  // Establecer el tamaño de fuente y mostrar la fecha y hora en la pantalla OLED
  oled.set1X();
  oled.setCursor(0, 7);

  // Formatear la fecha y hora en una cadena y almacenarla en el búfer
  sprintf(buffer, "%04d/%02d/%02d %02d:%02d:%02d  ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  //Guardar la hora, para cambiar el funcionamiento.
  hora = now.hour();

  // Mostrar la cadena formateada en la pantalla OLED
  oled.print(buffer);
}
