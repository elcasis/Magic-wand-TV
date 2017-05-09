/*
Codigo basado en sensor MPU6050
El uso de todo este codigo es libre asi su distribucion y modificacion
Falta de terminar la parte de la señal pero espero tenerla terminada en una semana



*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//===================================================================================================
//=============                Mapeado 1        =====================================================
//===================================================================================================
// Declaracion de ejes generales
int cero = 0;
int uno = 1;
float ejex; // altura
float ejey; // inclunacion lateral
float ejez; // orientacion lateral
int castingX, castingY, castingZ, castingZ2, castingZ3;
int castingX2, castingX3;
int mapX, mapY, mapZ;
int mapZmenos90, mapZmas90, mapXmenos120, mapXmas120;



int boton = 7; // boton para iniciar movimiento

byte conversorZ = 0;
byte conversorX = 0;
byte faseMapeado = 0;
byte estadoactual = LOW;
byte estadoanterior = HIGH;
int estadomapa = 0;
long previousMillis = 0;
long interval = 1100;
int contador = 0;
int contador2 = 0; // se utiliza para hacer una media de los valore
int contadorBoton = 0;
//===================================================================================================
//=============                Mapeado 2           =========== Creacion de plano virtual  ===========
//===================================================================================================

int plano[9][5];
int mapaLineal[45];
int s = 0;
int suma = 0;
int lineal = 0;
int contador3 = 0;
int vertical = 0;
int horizontal = 0;
// ================================================================
// ===                       sumas                              ===
// ================================================================
int l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11;
// ================================================================
// ===                       funciones                          ===
// ================================================================
int comparacion(int suma2);
void borradoMapa(); //funcion de borrado de mapa (se poner todo a cero)
// ================================================================
// ===               arrays de movimientos para señal           ===
// ================================================================
int prueba[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};






// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int i = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    Serial.println(F("Initializing DMP..."));
    //devStatus = 
    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
//===================================================================================================
//=============        Mapeado setup           ===========    Creacion de plano virtual   ===========
//===================================================================================================
  pinMode(boton, INPUT);

borradoMapa();

}    // FIONAL DE SET UP
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


//=============================================el calibrado de  
            ypr[0] = ypr[0] * 100;    // giro de plano z
            ypr[1] = ypr[1] * 100;    // giro lateral (sonido)   
            ypr[2] = ypr[2] * 100;    // pitch    
      //      Serial.print("posicion en plano\t");
      //      Serial.print(mapX);
     //       Serial.print("\t");
   //         Serial.print(ypr[2]);
      //      Serial.print("\t");
      //      Serial.print(mapZ);
      //      Serial.print("\t");    // seccion de ciclos
      //      Serial.println(""); 
       //     contador++;  
            
 //unsigned long currentMillis = millis();         // esto es para ver los ciclos por tiempos 
 //   if (currentMillis - previousMillis > 1000){  // reseteo el contador en esta parte
 //     previousMillis = currentMillis;
 //     Serial.println("tiempo");
  //    Serial.println(contador2);
 //     contador2++;
 //     contador = 1;
//          }                          // final de ciclos  funciona a 67 ciclos por segundo
//          if(contador2 == 1000){contador2 = 0;}
    //==== sacamos los valores del rango local al general en formato entero=========   
        castingX = (int) ypr[2]; 
        castingY = (int) ypr[1];
        castingZ = (int) ypr[0];
   //===== limitamos el corte de giro a 
        #endif
        
    }
  //======final de rango eje z es de 314,15. Lo limitamos a 314.14
    if(castingZ < -314){castingZ = -314;}
    if(castingZ > 314){castingZ = 314;}
  //==============iniciamos pulsacion===================
     
 estadoactual = digitalRead(boton);

//=====================================  inicio premapeado  =================
if(estadoactual == HIGH){faseMapeado = 0;}
if(estadoactual == LOW){faseMapeado++;}
      if(faseMapeado == 1){mapZmenos90 = castingZ - 90;  // ejez
                           mapZmas90 = castingZ + 90;
                           castingZ2 = castingZ;         
                           mapXmenos120 = castingX - 70;
                           mapXmas120 = castingX + 70;
                           castingX2 = castingX;
                           }// fin fase 1
                           
        //=========== operaciones para eliminar el limite opuesto =======================
        
      if(faseMapeado >= 2){if(castingZ2 < -222){if(castingZ > 214 && castingZ < 314)
                                                {castingZ3 = 314 - castingZ;
                                                 castingZ = 314 + castingZ3;
                                                 castingZ = castingZ * -1;}}
                           if(castingZ2 > 222){if(castingZ > -314 && castingZ < -214)
                                                {castingZ = castingZ * -1;
                                                 castingZ3 = 314 - castingZ;
                                                 castingZ = 314 + castingZ3;}}
                                         }  //fin fase dos        
       mapZ = map(castingZ,mapZmenos90,mapZmas90,0,5);   // asignacion eje z para el plano
                      if(mapZ > 5){mapZ = 5;}
                      if(mapZ < 0){mapZ = 0;}
       mapX = map(castingX,mapXmenos120,mapXmas120,0,9); // asignacion eje x 
                      if(mapX < 0){mapX = 0;}
                      if(mapX > 5){mapX = 5;}
 //================= asignacion de unos al mapa;
 
 if(estadoactual == LOW){plano[mapX][mapZ] = 1;}  // esto es el puntero

 if(estadoactual == HIGH){if(estadoanterior == LOW){Serial.println("==============================================");
                          //======== obtencion de datos y transformacion lineal    
                                      while(vertical < 9){while(horizontal < 5){
                                                                        mapaLineal[lineal] = plano[vertical][horizontal];                     
                                                                        Serial.print(mapaLineal[lineal]);
                                                                        lineal++;
                                                                        horizontal++;
                                                                        }
                                                          vertical++;
                                                          horizontal = 0;
                                                          }
                                      vertical = 0;
                                      horizontal = 0;
                                      lineal = 0;
                                      Serial.println("");
                                      borradoMapa();
                                   
                                   
                                   }}//fin asignacion de mapa

// comparativas para detectar secciones especificas a 1 y hacer la asignacion de la señal
suma = 0;
 if(estadoactual == HIGH){if(estadoanterior == LOW){while(s < 35){suma = suma + mapaLineal[s];s++;}
                                                                  Serial.println("");
                                                                  Serial.print("==============");
                                                                  Serial.print(suma);
                                                                  Serial.println("");
                                                                  s = 0;
                                                                  if(suma == 1){if(mapaLineal[22] == 1){Serial.println("============== acierto");}
                                                                  else{Serial.println("error en mapeado");}}
  // seccion de sumas de valores por lines x e y para modificaciones o comparaciones futuras 
  // dejo la puerta aqui para poder hacer las comparaciones por funciones 
  // recordat que el arreglo a compara es mapaLineal[]
                                 l1 = mapaLineal[0] + mapaLineal[1] + mapaLineal[2] + mapaLineal[3] + mapaLineal[4];
                                 l2 = mapaLineal[5] + mapaLineal[6] + mapaLineal[7] + mapaLineal[8] + mapaLineal[9];
                                 l3 = mapaLineal[10] + mapaLineal[11] + mapaLineal[12] + mapaLineal[13] + mapaLineal[14];                  
                                 l4 = mapaLineal[15] + mapaLineal[16] + mapaLineal[17] + mapaLineal[18] + mapaLineal[19];
                                 l5 = mapaLineal[20] + mapaLineal[21] + mapaLineal[22] + mapaLineal[23] + mapaLineal[24];
                                 l6 = mapaLineal[25] + mapaLineal[26] + mapaLineal[27] + mapaLineal[28] + mapaLineal[29];
                                 l7 = mapaLineal[0] + mapaLineal[5] + mapaLineal[10] + mapaLineal[15] + mapaLineal[20];
                                 l8 = mapaLineal[1] + mapaLineal[6] + mapaLineal[11] + mapaLineal[16] + mapaLineal[21];                  
                                 l9 = mapaLineal[2] + mapaLineal[7] + mapaLineal[12] + mapaLineal[17] + mapaLineal[22];
                                 l10 = mapaLineal[3] + mapaLineal[8] + mapaLineal[13] + mapaLineal[18] + mapaLineal[23];
                                 l11= mapaLineal[4] + mapaLineal[9] + mapaLineal[14] + mapaLineal[19] + mapaLineal[24];
                                                  if((l7 + l11) == 0){if((l8 + l9 + l10) > 12){Serial.println("============== encendido ===========");}}               
                                                     }}// fin de ajuste para la señal






 
 estadoanterior = estadoactual;    
      if(faseMapeado > 3){faseMapeado = 3;}





//fin de asignacion de mapeado




            }// FINAL LOOP
// ================================================================
// ===                 MAIN PROGRAM LOOP   FINAL                ===
// ================================================================

// ================================================================
// ===                 FUNCIONES CDE CONTROL                    ===
// ================================================================

void borradoMapa()   // vision  y borrado de mapeado 
{
    while(vertical < 9){
                      while(horizontal < 5){Serial.print(plano[vertical][horizontal]);
                                            Serial.print("\t");
                                            plano[vertical][horizontal] = 0;
                                            horizontal++;
                                            }
                      Serial.println("");
                      vertical++;
                      horizontal = 0;
                      }
                      vertical = 0;
                      horizontal = 0;
  }


//0000000000000000000000100  00000000000000000000 _____ valor central


//011000111001010011100111000110001100000000000


