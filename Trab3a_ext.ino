#include <inttypes.h>

#define BUTTON_PULLUP HIGH
#define BUTTON_PULLUP_INTERNAL 2
#define BUTTON_PULLDOWN LOW

class Button;
typedef void (*buttonEventHandler)(Button&);

class Button {
  public:
  
    Button(uint8_t buttonPin, uint8_t buttonMode=BUTTON_PULLUP_INTERNAL, bool _debounceMode=true, int _debounceDuration=20);
    
    uint8_t             pin;
    void pullup(uint8_t buttonMode);
    void pulldown();
    void process();

    bool isPressed(bool proc=true);
    bool wasPressed(bool proc=true);
    bool stateChanged(bool proc=true);
    bool uniquePress();
    
    void setHoldThreshold(unsigned int holdTime);
    bool held(unsigned int time=0);
    bool heldFor(unsigned int time);
    
    void pressHandler(buttonEventHandler handler);
    void releaseHandler(buttonEventHandler handler);
    void clickHandler(buttonEventHandler handler);
    void holdHandler(buttonEventHandler handler, unsigned int holdTime=0);
  
    unsigned int holdTime() const;
    inline unsigned int presses() const { return numberOfPresses; }
    
    bool operator==(Button &rhs);
    
  private: 
    uint8_t             mode;
    uint8_t             state;
    bool                debounceMode;
    unsigned long       pressedStartTime;
    unsigned int        holdEventThreshold;
    unsigned long       debounceStartTime;
    int                 debounceDuration;
    buttonEventHandler  cb_onPress;
    buttonEventHandler  cb_onRelease;
    buttonEventHandler  cb_onClick;
    buttonEventHandler  cb_onHold;
    unsigned int        numberOfPresses;
    bool                triggeredHoldEvent;
};

// bit positions in the state byte
#define CURRENT 0
#define PREVIOUS 1
#define CHANGED 2

/*
|| @constructor
|| | Set the initial state of this button
|| #
|| 
|| @parameter buttonPin  sets the pin that this switch is connected to
|| @parameter buttonMode indicates BUTTON_PULLUP or BUTTON_PULLDOWN resistor
*/
Button::Button(uint8_t buttonPin, uint8_t buttonMode, bool _debounceMode, int _debounceDuration){
  pin=buttonPin;
  pinMode(pin,INPUT);
  
  debounceMode = _debounceMode;
  debounceDuration = _debounceDuration;
  debounceStartTime = millis();

  buttonMode==BUTTON_PULLDOWN ? pulldown() : pullup(buttonMode);
  state = 0;
  bitWrite(state,CURRENT,!mode);
  
  cb_onPress = 0;
  cb_onRelease = 0;
  cb_onClick = 0;
  cb_onHold = 0;
  
  numberOfPresses = 0;
  triggeredHoldEvent = true;
}

/*
|| @description
|| | Prepare logic for a pullup button.
|| | If mode is internal set pin HIGH as default
|| #
*/
void Button::pullup(uint8_t buttonMode)
{
  mode=BUTTON_PULLUP;
  if (buttonMode == BUTTON_PULLUP_INTERNAL) 
  {
    digitalWrite(pin,HIGH);
  }
}

/*
|| @description
|| | Set pin LOW as default
|| #
*/
void Button::pulldown(void)
{
  mode=BUTTON_PULLDOWN;
}

/*
|| @description
|| | Read and write states; issue callbacks
|| #
|| 
|| @return true if button is pressed
*/
void Button::process(void)
{
  //save the previous value
  bitWrite(state,PREVIOUS,bitRead(state,CURRENT));
  
  //get the current status of the pin
  if (digitalRead(pin) == mode)
  {
    //currently the button is not pressed
    bitWrite(state,CURRENT,false);
  } 
  else 
  {
    //currently the button is pressed
    bitWrite(state,CURRENT,true);
  }
  
  //handle state changes
  if (bitRead(state,CURRENT) != bitRead(state,PREVIOUS))
  {
    unsigned int interval = millis() - debounceStartTime;
    // if(debounceMode){
    //   Serial.print("debounceStartTime: ");
    //   Serial.print(debounceStartTime);
    //   Serial.print("\tdebounceDuration: ");
    //   Serial.println(debounceDuration);
      // Serial.println(interval);
    // }
    if(debounceMode && interval < debounceDuration){
      // not enough time has passed; ignore
      return;
    }
    // Serial.println("state changed");
    debounceStartTime = millis();
    //the state changed to PRESSED
    if (bitRead(state,CURRENT) == true) 
    {
      numberOfPresses++;
      if (cb_onPress) { cb_onPress(*this); }   //fire the onPress event
      pressedStartTime = millis();             //start timing
      triggeredHoldEvent = false;
    } 
    else //the state changed to RELEASED
    {
      if (cb_onRelease) { cb_onRelease(*this); } //fire the onRelease event
      if (cb_onClick) { cb_onClick(*this); }   //fire the onClick event AFTER the onRelease
      //reset states (for timing and for event triggering)
      pressedStartTime = -1;
    }
    //note that the state changed
    bitWrite(state,CHANGED,true);
  }
  else
  {
    //note that the state did not change
    bitWrite(state,CHANGED,false);
    //should we trigger an onHold event?
    if (pressedStartTime!=-1 && !triggeredHoldEvent) 
    {
      if (millis()-pressedStartTime > holdEventThreshold) 
      { 
        if (cb_onHold) 
        { 
          cb_onHold(*this); 
          triggeredHoldEvent = true;
        }
      }
    }
  }
}

/*
|| @description
|| | Return the bitRead(state,CURRENT) of the switch
|| #
|| 
|| @return true if button is pressed
*/
bool Button::isPressed(bool proc)
{
  if(proc) process();
  return bitRead(state,CURRENT);
}

/*
|| @description
|| | Return true if the button has been pressed
|| #
*/
bool Button::wasPressed(bool proc)
{
  if(proc) process();
  return bitRead(state,CURRENT);
}

/*
|| @description
|| | Return true if state has been changed
|| #
*/
bool Button::stateChanged(bool proc)
{
  if(proc) process();
  return bitRead(state,CHANGED);
}

/*
|| @description
|| | Return true if the button is pressed, and was not pressed before
|| #
*/
bool Button::uniquePress()
{
  process();
  return (isPressed(false) && stateChanged(false));
}

/*
|| @description
|| | onHold polling model
|| | Check to see if the button has been pressed for time ms
|| | This will clear the counter for next iteration and thus return true once
|| #
*/
bool Button::held(unsigned int time /*=0*/) 
{
  process();
  unsigned int threshold = time ? time : holdEventThreshold; //use holdEventThreshold if time == 0
  //should we trigger a onHold event?
  if (pressedStartTime!=-1 && !triggeredHoldEvent) 
  {
    if (millis()-pressedStartTime > threshold) 
    { 
      triggeredHoldEvent = true;
      return true;
    }
  }
  return false;
}

/*
|| @description
|| | Polling model for holding, this is true every check after hold time
|| | Check to see if the button has been pressed for time ms
|| #
*/
bool Button::heldFor(unsigned int time) 
{
  if (isPressed()) 
  {
    if (millis()-pressedStartTime > time) { return true; }
  }
  return false;
}

/*
|| @description
|| | Set the hold event time threshold
|| #
*/
void Button::setHoldThreshold(unsigned int holdTime) 
{ 
  holdEventThreshold = holdTime; 
}

/*
|| @description
|| | Register a handler for presses on this button
|| #
||
|| @parameter handler The function to call when this button is pressed
*/
void Button::pressHandler(buttonEventHandler handler)
{
  cb_onPress = handler;
}

/*
|| @description
|| | Register a handler for releases on this button
|| #
||
|| @parameter handler The function to call when this button is released
*/
void Button::releaseHandler(buttonEventHandler handler)
{
  cb_onRelease = handler;
}

/*
|| @description
|| | Register a handler for clicks on this button
|| #
||
|| @parameter handler The function to call when this button is clicked
*/
void Button::clickHandler(buttonEventHandler handler)
{
  cb_onClick = handler;
}

/*
|| @description
|| | Register a handler for when this button is held
|| #
||
|| @parameter handler The function to call when this button is held
*/
void Button::holdHandler(buttonEventHandler handler, unsigned int holdTime /*=0*/)
{
  if (holdTime>0) { setHoldThreshold(holdTime); }
  cb_onHold = handler;
}

/*
|| @description
|| | Get the time this button has been held
|| #
||
|| @return The time this button has been held
*/
unsigned int Button::holdTime() const { if (pressedStartTime!=-1) { return millis()-pressedStartTime; } else return 0; }

/*
|| @description
|| | Compare a button object against this
|| #
|| 
|| @parameter  rhs the Button to compare against this Button
|| 
|| @return true if they are the same
*/
bool Button::operator==(Button &rhs) 
{
  return (this==&rhs);
}

//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************


#include <Keypad.h>
#include <LiquidCrystal.h>

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {12, 11, 10, 9}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {8, 7, 6}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 4, en = 13, d4 = 3, d5 = 2, d6 = 1, d7 = 0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

Button UP = Button(A2,BUTTON_PULLUP_INTERNAL);      
Button DOWN = Button(A1,BUTTON_PULLUP_INTERNAL);
Button ENTER = Button(A0,BUTTON_PULLUP_INTERNAL);
Button DEBUG = Button(5,BUTTON_PULLUP_INTERNAL);

const int LED_TEMP_HIGH=A3;
const int LED_TEMP_LOW=22;
const int LED_PHOTO_HIGH=24;
const int LED_PHOTO_LOW=26;

int TEMP=A4;
int PHOTO=A5;

float temp;
int photo;

unsigned long currentMillis= millis();
unsigned long previousMillis = millis();

int buttonStateUP = 0;         // variable for reading the pushbutton status
int buttonStateDOWN = 0;
int buttonStateENTER = 0;

int Index=0;
int Menu =0;

int debugMode=0;

float TpLimSup = 1000;
float TpLimInf = -100;
int LmLimSup = 1000;
int LmLimInf = -100;

float TpLimSupHist;
float TpLimInfHist;
float LmLimSupHist;
float LmLimInfHist;

char unidade = '_';
char dezena = '_';
char centena = '_';
boolean levalor = true;
int valor = 0; 

//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************
//**************************************************************



void setup(){
  Serial.begin(9600);
  Serial.setTimeout(0);
  //while (!Serial) {} ;
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  analogReference(EXTERNAL);
  
  pinMode(LED_TEMP_HIGH, OUTPUT);
  pinMode(LED_TEMP_LOW, OUTPUT);
  pinMode(LED_PHOTO_HIGH, OUTPUT);
  pinMode(LED_PHOTO_LOW, OUTPUT);
}

void loop(){
  
  //leitura clock arduino
  currentMillis = millis();        
  
  // Leitura e linearização das entradas analogicas
  photo = map(analogRead(PHOTO),0,1024,0,100);  
  temp = (3.3*analogRead(TEMP)*10)/1023;
  
  
  //Acionamento dos Leds
  //Calcula a histerese
  TpLimSupHist=TpLimSup*0.95;
  TpLimInfHist=TpLimInf*0.95;
  LmLimSupHist=LmLimSup*0.95;
  LmLimInfHist=LmLimInf*0.95;
    
  //Temperatura
  if(temp>=TpLimSup){
    digitalWrite(LED_TEMP_HIGH,HIGH);
          if(debugMode==1){
                  Serial.println("LED_TEMP_HIGH:ON");}          
  }
  
  if(temp<=TpLimSupHist){
    digitalWrite(LED_TEMP_HIGH,LOW);
          if(debugMode==1){
                  Serial.println("LED_TEMP_HIGH:OFF");}
  }
  
  if(temp<=TpLimInf){
    digitalWrite(LED_TEMP_LOW,HIGH);
          if(debugMode==1){
                  Serial.println("LED_TEMP_LOW:ON");}
  }
  
  if(temp>=TpLimInfHist){
    digitalWrite(LED_TEMP_LOW,LOW);
          if(debugMode==1){
                  Serial.println("LED_TEMP_LOW:OFF");}
  }
  
  //Luminosidade
  if(photo>=LmLimSup){
    digitalWrite(LED_PHOTO_HIGH,HIGH);
          if(debugMode==1){
                  Serial.println("LED_PHOTO_HIGH:ON");}
  }
  
  if(photo<=LmLimSupHist){
    digitalWrite(LED_PHOTO_HIGH,LOW);
          if(debugMode==1){
                  Serial.println("LED_PHOTO_HIGH:OFF");}
  }
  
  if(photo<=LmLimInf){
    digitalWrite(LED_PHOTO_LOW,HIGH);
          if(debugMode==1){
                  Serial.println("LED_PHOTO_LOW:ON");}
  }
  
  if(photo>=LmLimInfHist){
    digitalWrite(LED_PHOTO_LOW,LOW);
          if(debugMode==1){
                  Serial.println("LED_PHOTO_LOW:OFF");}
  }
  
  
  //Atualiza o LCD a cada 1seg
  if (currentMillis - previousMillis >= 1000) {
    // save the last time you updated the LCD
    previousMillis = currentMillis;
    lcd.setCursor(0, 0);
    lcd.print(String("Tp.") + String(temp,1) + String("  Lm.") + String(photo) );
          
          if(debugMode==1){
                  Serial.print("Tp.");
                  Serial.println(String(temp,1));
                  Serial.print("Lm.");
                  Serial.println(String(photo));}
    }
  
  //Menu = 1  >> Conf.Temp
    //Index = 0 >> Lim.Sup
    //Index = 1 >> Lim.Inf
    //Index = 2 >> Voltar

    //Menu = 2  >> Conf.Lumi
    //Index = 0 >> Lim.Sup
    //Index = 1 >> Lim.Inf
    //Index = 2 >> Voltar
  
  
  if((Menu==1||Menu==2)&& Index!=2){    
  // le teclado
  char tecla = keypad.getKey();
  // só mostra a tecla se algo for pressionada
     if(tecla != 0) { // nada pressionada é igual a 0
    // alguma coisa foi pressionada
      // testa se a tecla válida - usa a função isdigit() - função das bibliotecas padrões C
      if(isdigit(tecla)) { 
        // desloca da unidade para a centena
        centena = dezena;
        dezena = unidade;
        unidade = tecla; 
      }
       
       // Mostra no LCD
          if(Menu==1&& Index!=2){
            lcd.setCursor(11,1);
            lcd.print(centena);
            lcd.print(dezena);
            lcd.print(".");
            lcd.print(unidade);
          }

          if(Menu==2&& Index!=2){
            lcd.setCursor(11,1);
            lcd.print(centena);
            lcd.print(dezena);
            lcd.print(unidade);
          }
       
      if (tecla == '#') { // indica que terminou de digitar o valor...
        // agora temos que converter o valor para um numero inteiro
        char aux[4]; 
        if (isdigit(centena)) { // testa se a centena foi digitada
          aux[0] = centena;
        } else {
          aux[0] = '0';
        }
        if (isdigit(dezena)) {
          aux[1] = dezena;
        } else {
          aux[1] = '0';
        }
        if (isdigit(unidade)) {
          aux[2] = unidade;
        } else {
          aux[2] = '0';
        }
        //aux[3] = 0; // insiro um zero para FINALIZAR a string... 
        //valor = atoi(aux); // converte a string temp para um numero inteiro
       
        //Move valor para constante
        if(Menu==1 && Index==0){
        TpLimSup=atoi(aux)/10;
          if(debugMode==1){
                  Serial.print("Novo Limite Sup Temp: ");
                Serial.println(TpLimSup);}
        }
      
        if(Menu==1 && Index==1){
        TpLimInf=atoi(aux)/10;  
          if(debugMode==1){
                  Serial.print("Novo Limite Inf Temp: ");
                Serial.println(TpLimInf);}
        }
        
        if(Menu==2 && Index==0){
        LmLimSup=atoi(aux);

          if(LmLimSup>=100){
            LmLimSup=100;
            lcd.setCursor(11,1);
            lcd.print(1);
            lcd.print(0);
            lcd.print(0);}
            
            if(debugMode==1){
                  Serial.print("Novo Limite Sup Lumi: ");
                Serial.println(LmLimSup);}
        }
      
        if(Menu==2 && Index==1){
        LmLimInf=atoi(aux); 
            if(debugMode==1){
                  Serial.print("Novo Limite Inf Lumi: ");
                Serial.println(LmLimInf);}
        }
      
      }
    }
 }
  else{
    lcd.setCursor(11,1);
    lcd.print("    ");} 
                      
  
  // Leitura dos botões
  if(UP.uniquePress()){
    buttonStateUP = HIGH;}
    else{
    buttonStateUP = LOW;}

  if(DOWN.uniquePress()){
    buttonStateDOWN = HIGH;}
    else{
    buttonStateDOWN = LOW;}

  if(ENTER.uniquePress()){
    buttonStateENTER = HIGH;}
    else{
    buttonStateENTER = LOW;}
  
  if(DEBUG.isPressed()){
    debugMode = 1;}
    else{
    debugMode = 0;}
  
  
  //Incrementa/decrementa index
      if (buttonStateUP == HIGH) {    
        Index++;}     
      if (buttonStateDOWN == HIGH) {
        Index--;}     
     
    /*// Mantem Indexador dentro do limite de opcoes do menu
    switch (Menu) {
    case 0:
      if (Index<=1){Index=Index;}
      else{Index=1;}
      if (Index<=-1){Index=0;}
      break;  
  case 1:     
      if (Index<=1){Index=Index;}
      else{Index=1;}
      if (Index<=-1){Index=0;}
      break;
    case 2:     
      if (Index<=5){Index=Index;}
      else{Index=5;}
      if (Index<=-1){Index=0;}
      break;  
    }*/
  
  //Muda de menu
        //Entra no menu Conf. Temp
      if (buttonStateENTER == HIGH && Index == 0 && Menu == 0){
            //lcd.clear();
          Index=0;
          Menu=1;
          //Serial.println("Conf. Temp selecionado");
        }
        //Entra no menu Conf. Lumi
        if (buttonStateENTER == HIGH && Index == 1 && Menu == 0){
            //lcd.clear();
          Index=0;
          Menu=2;
          //Serial.println("Conf.Lumi selecionado");
        }
        //Dentro do menu Conf.Temp, volta para menu inicial
      //Somente os valores limites sejam definidos 
        if (buttonStateENTER == HIGH && Index == 2 && Menu == 1 && TpLimSup!=0 && TpLimInf!=0){
            //lcd.clear();
          Index=0;
          Menu=0;
          //Serial.println("Tela Inicial");
        }
      //Dentro do menu Conf. Lumi, volta para menu inicial
      //Somente os valores limites sejam definidos
      if (buttonStateENTER == HIGH && Index == 2 && Menu == 2 && LmLimSup!=0 && LmLimInf!=0){
            //lcd.clear();
          Index=0;
          Menu=0;
          //Serial.println("Tela Inicial");
        }
  
// Atualiza LCD  
  if (Menu==0){
     switch(Index){
     case 0:
          lcd.setCursor(0, 1);
            lcd.print("Conf. Temp      ");
     break;
     
     case 1:
          lcd.setCursor(0, 1);
            lcd.print("Conf. Lumi      ");
     break;
    } 
  }  
  
  
  if (Menu==1){
    switch(Index){
     case 0:
          lcd.setCursor(0, 1);
            lcd.print("TpLimSup  ");
     break;
     
     case 1:
          lcd.setCursor(0, 1);
            lcd.print("TpLimInf  ");
     break;
      
     case 2:
          lcd.setCursor(0, 1);
            lcd.print("Voltar          ");
     break;
    } 
  }        
    
  
if (Menu==2){
    switch(Index){
     case 0:
          lcd.setCursor(0, 1);
            lcd.print("LmLimSup  ");
     break;
     
     case 1:
          lcd.setCursor(0, 1);
            lcd.print("LmLimInf  ");
     break;
      
     case 2:
          lcd.setCursor(0, 1);
            lcd.print("Voltar          ");
     break;
    } 
  }             
          
          
}



