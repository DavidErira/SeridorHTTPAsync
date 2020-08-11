

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>


//variables del MPU------------------------------------------------------

#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

long tiempo_prev;
float dt;

boolean  mpuOk = false;
boolean  wifiOk = false;
//variables del MPU----------------------------------------------------------

// variables del servidor---------------------------------------------------------
AsyncWebServer server(80);

const char* ssid = "NOELBA";
const char* password = "qazxsw1234";
const char* PARAM_MESSAGE = "message";

int vari=0;

IPAddress ip(192, 168, 1, 255);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


// variables del servidor---------------------------------------------------------




void setup() {

    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    //WiFi.config(ip, gateway, subnet);

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    else
    {
        wifiOk = true;
    }
    
    
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

        //String sale =String(vari);
        String sale1 =String(Angle[0]);
        String sale2 =String(Angle[1]);

        String sale = sale1;
        AsyncWebServerResponse *response = request->beginResponse(200,"text/plain",sale);
        response->addHeader("Access-Control-Allow-Origin", "*");

        request->send(response);
        
        Serial.print("salio mensaje con el valor: ");
        Serial.println(sale);
        vari=vari+1;
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(notFound);
    server.begin();

    //delay(200);
    //setup MPU-------------------------------------------------------------
    Wire.begin(4,5); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
 //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;
 
   //Mostrar los valores por consola
   //valores = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90";
   //Serial.println(valores);

   //Serial.println("Angulo en x: "+ String(Angle[0]) +" |  Angulo en y: "+ String(Angle[1])+" |  Angulo en z: "+ String(Angle[2]));
   
   delay(10);

    if(String(Angle[0]) == "nan" && mpuOk==false){
        //mpuOk = true;
        Serial.println("estado: NAN");
        ESP.reset();
    }

    if (wifiOk && String(Angle[0]) != "nan"){
        digitalWrite(LED_BUILTIN, LOW);
        wifiOk = false;
    }

}