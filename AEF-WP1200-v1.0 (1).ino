//
// Plateforme MERI
// Automate à États Finis - Slave Intermédiaire - Braccio++
// WP1200_Emballage_Braccio++
//

///////////////
// IMPORTANT //
///////////////
// Sur les Braccio++ le entrées D2,D3,D45 et A0, A1 sont utilisées par 
// le joystick du obot:
// BTN_DOWN  -> 2
// BTN_LEFT  -> 3
// BTN_RIGHT -> 4
// BTN_UP    -> 5
// BTN_SEL   -> A0
// BTN_ENTER -> A1
//
// Le bus SPI utilise les ports 11, 12 et 13 :
// SPI-CIPO -> D12
// SPI-COPI -> D11
// SPI-SCK  -> D13
//////////////

/**********************************************************************
  Les includes nécessaires au programme  
 **********************************************************************/

// 'Braccio++.h' : gestion du bars robotisé Braccio++
#include <Braccio++.h>

// 'WiFiNANA.h' : accès à la LED RBG du Braccio++
#include <WiFiNINA.h>


/**********************************************************************
  Définition des Macros utiles : états automate, numéros des broches...
 **********************************************************************/
#define BUTTON_ENTER 6
#define TIME_DELAY 1500

#define INIT 1
#define WAKE_UP 2
#define WAIT 3
#define RUN  4

// la broche de liaison avec la barrière optique
#define pinCapteur 2

///////////////
// IMPORTANT //
///////////////
// 
// On utilise les I/O 3, 4 & 5 qui sont nomalement utilisées
// pour le Joystick du Braccio++ : 
//
// !!! ATTENTION !!! il ne faut pas manipuler le Joystick du Bracccio
//                   sous peine de griller les E/S du RP2040 !!!
//////////////

#define pinReadMaster  3
#define pinWriteMaster 4
#define pinReadSlave   6
#define pinWriteSlave  5

/**********************************************************************
  Déclaration/définition des variables globales 
 **********************************************************************/

// Événements gérés par l'automate
bool Master_GO  = false;
bool Slave_OK   = false;
bool Event_BPA  = false;
bool Event_DONE = false;
bool Event_BD   = false;    // événement 'Bougie Détectée'

//
// Braccio ++ joints
//
auto gripper    = Braccio.get(1);
auto wristRoll  = Braccio.get(2);
auto wristPitch = Braccio.get(3);
auto elbow      = Braccio.get(4);
auto shoulder   = Braccio.get(5);

int state;                 // L'ETAT de l'automate
int prev_BP_ENTER_state;   // mémorisation état bouton poussoir
int oldReceivedFromMaster; // Mémorisation du signal émis par Master
int oldReceivedFromSlave;  // Mémorisation du signal émis par Slave
int tempo;                 // délais de la boucle loop 
int tau = 500;             // durée du front descendant

int etat_LEDG = 0;

lv_obj_t* label;
String mess;
lv_style_t style;

//
// Les positions du bras robotisé
//

// Déclaration du type "six_angle" : tableau de 6 flots
typedef float six_angles[6];

// Position de sécurité, bras vertical:
const six_angles SAFE_POS = {157.5, 157.5, 157.5, 157.5, 157.5, 90.0};

// Position "prêt à travailler" (Cobra):
const six_angles WAIT_POS = {160.0, 160.0, 210.0, 240.0, 100.0, 180.0};

// Position de travail du bras:
const six_angles MOV1 = {208.45, 160.57, 159.94, 66.23, 162.54, 100.25};
const six_angles MOV2 = {144.66, 161.91, 158.68, 68.91, 163.17, 100.09};
const six_angles MOV3 = {144.66, 163.17, 159.63, 71.74, 162.70, 187.27};
const six_angles MOV4 = {207.51, 163.17, 158.68, 80.01, 155.69, 186.95};

bool movement = false; // Initialise le mouvement des braccios
int oldInfoCapteur;

/**********************************************************************
   Déclaration des fonctions utiliées dans le programme 
   (la définition est codée à la fin du programme)
 **********************************************************************/
void init_LSC_screen(void) ;
void update_message(const char* message);

// Fonctions pour les transition d'état
void change_to_state_WAKE_UP();
void change_to_state_WAIT();
void change_to_state_RUN();

void setup() 
{
  // Initialisation de l'objet 'Serial' pour afficher des messages à l'écran
  // via le moniteur:
  Serial.begin(9600);    // 9600 : vitesse de tranfert des octets par la liaison USB

  // Initialisation de l'écran du Braccio:
  if (!Braccio.begin(init_LSC_screen)) 
  {
    update_message("Error Braccio");
    for (;;) {}
  }

  // initialization variables:
  oldReceivedFromMaster = HIGH;
  oldReceivedFromSlave  = HIGH;
  prev_BP_ENTER_state   = LOW;
  oldInfoCapteur        = HIGH;

  //
  // Configuration des E/S numériques
  //
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  pinMode(pinReadMaster, INPUT_PULLUP);
  pinMode(pinWriteMaster, OUTPUT);
  pinMode(pinReadSlave, INPUT_PULLUP);
  pinMode(pinWriteSlave, OUTPUT);

  // Ligne numérique avec la barrière optique:
  pinMode(pinCapteur, INPUT);

   // maintient de la sortie Master à HIGH:
  digitalWrite(pinWriteMaster, HIGH);  
  
  // maintient de la sortie Slave à HIGH:
  digitalWrite(pinWriteSlave, HIGH);  

  // Temporisation démarrage
  update_message("Waiting 2s");
  delay(2000);

  // mettre le bras robotisé en position SAFE (aligné vertical):
  Braccio.moveTo(SAFE_POS[0], SAFE_POS[1], SAFE_POS[2], SAFE_POS[3], SAFE_POS[4], SAFE_POS[5]);
  delay(1000);

  //
  // L'automate démarre dans l'état INIT
  //  
  state = INIT;
  tempo = 100;    //ms, pour clignotement rapide
  update_message("INIT");

  // éteindre la LED RB:
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
  
  delay(1000);  
}

void loop() 
{
 //
  // 1 - lecture des périphériques qui peuvent fournir un événement
  //     susceptible de provoquer un changement d'état de l'automate.…   

  int receivedFromMaster = digitalRead(pinReadMaster);
  int receivedFromSlave  = digitalRead(pinReadSlave);
  int cur_BP_ENTER_state = Braccio.isButtonPressed_ENTER();

  int infoCapteur = digitalRead(pinCapteur);
  String mess = "infoCapteur: " + String(infoCapteur) + "; oldInfoCapteur: " + String(oldInfoCapteur);
  Serial.println(mess);
  
  //
  // 2 - Traitement des événements entraînant une transition d'état
  //     (cf Tableau des transitions d'état)

  // Événement Event_PBA (Bouton Poussoir Appuyé: front montant)
  Event_BPA = (prev_BP_ENTER_state  == LOW) && (cur_BP_ENTER_state == HIGH);

  // Master:GO 
  // L'automate slave-Intermédiaire reçoit un front descendant venant de l'automate-Amont)
  Master_GO = (oldReceivedFromMaster == HIGH) && (receivedFromMaster == LOW);  

  // Slave:OK 
  // L'automate Slave-Intermédiaire reçoit un front descendant venant de l'automate-Aval)
  Slave_OK = (oldReceivedFromSlave  == HIGH) && (receivedFromSlave  == LOW);

  // Événement Event_BD : une bougie est détectée par la barrière optique
  Event_BD =  (oldInfoCapteur == HIGH) && (infoCapteur == LOW);
  
  if (Master_GO)
  {
    switch(state)
    {
      case INIT:
        change_to_state_WAKE_UP();
      break;
    }
  }
  else if (Slave_OK)
  {
    switch (state)
    {
      case WAKE_UP:
        change_to_state_WAIT();
        
        // envoyer OK à l'automate Amont (WP800-Transport(Tri))
        digitalWrite(pinWriteMaster, LOW);
        delay(tau); // ms
        digitalWrite(pinWriteMaster, HIGH);  // keep the line HIGH
      break;
    }
  }    
  else if (Event_BPA)
  {
    switch (state)
    {
      case INIT:
        change_to_state_WAKE_UP();
      break;

      case WAKE_UP:
        change_to_state_WAIT();
      break;

      case WAIT:
        change_to_state_RUN();        
      break;
    }
  }    
  else if (Event_BD)
  {
    switch (state)
    {
      case WAIT:
        Serial.println("Faisceau coupé");
        change_to_state_RUN();
      break;
    }
  }    
  else if (Event_DONE)
  {
    switch (state)
    {
      case RUN:
        // désactiver le signal DONE:
        Event_DONE = false;
        change_to_state_WAIT();
      break;
    }
  }    


  //
  // 3 - Traitement des états
  //
  
  switch (state) 
  {
    case INIT:
      // clignotement LED verte
      etat_LEDG = 1 - etat_LEDG;
      digitalWrite(LEDG, PinStatus(etat_LEDG));
    break;

    case WAKE_UP:
      // clignotement LED verte
      etat_LEDG = 1 - etat_LEDG;
      digitalWrite(LEDG, PinStatus(etat_LEDG));
    break;
        
    case WAIT:
      // RAF
      ;
    break;

    case RUN:
      //faire tout les mouvements puis remettre à la position HOME à la fin et mettre fin au mouvement
      Braccio.moveTo(MOV1[0], MOV1[1], MOV1[2], MOV1[3], MOV1[4], MOV1[5]); delay(3000);
      Braccio.moveTo(MOV2[0], MOV2[1], MOV2[2], MOV2[3], MOV2[4], MOV2[5]); delay(3000);
      Braccio.moveTo(MOV3[0], MOV3[1], MOV3[2], MOV3[3], MOV3[4], MOV3[5]); delay(3000);
      Braccio.moveTo(MOV4[0], MOV4[1], MOV4[2], MOV4[3], MOV4[4], MOV4[5]); delay(3000);

      // mettre le bras robotisé en position SAFE (aligné vertical):
      Braccio.moveTo(SAFE_POS[0], SAFE_POS[1], SAFE_POS[2], SAFE_POS[3], SAFE_POS[4], SAFE_POS[5]);

      // envoyer le signal DONE à l'automate:
      Event_DONE = true;
    break;
  }

  //
  // 4 - Finir
  //

  // mémoriser ce qui doit l’être, pour le prochain tour de boucle:
  oldReceivedFromMaster = receivedFromMaster;
  oldReceivedFromSlave  = receivedFromSlave;
  prev_BP_ENTER_state   = cur_BP_ENTER_state;
  oldInfoCapteur        = infoCapteur;
  
  delay(tempo);
}

inline void change_to_state_WAKE_UP()
{
  // 
  // Actions à toujours faire pour la transition vers l'état WAKE_UP
  //
  state= WAKE_UP;

  // éteindre la LED, elle clignotera
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
  
  // Affichage sur LCD:
  update_message("WAKE_UP");
          
  // Send a GO to the next slave:
  delay(1000);
  digitalWrite(pinWriteSlave, LOW);
  delay(tau); // ms        
  digitalWrite(pinWriteSlave, HIGH);  // keep the line HIGH

  // remettre le clignotement à 1Hz
  tempo = 500;
}

inline void change_to_state_WAIT()
{
  // 
  // Actions à toujours faire pour la transition vers l'état WAKE_UP
  //
  state = WAIT;

  // Allumer LED verte:
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  
  // Affichage sur LCD:
  update_message("WAIT");

  // Mettre le bras en position d'attente (Cobra):
  Braccio.moveTo(WAIT_POS[0], WAIT_POS[1], WAIT_POS[2], WAIT_POS[3], WAIT_POS[4], WAIT_POS[5]);
  delay(2000);
  
  // tempo sans clignotement
  tempo = 300;
}

void change_to_state_RUN()
{
  // 
  // Actions à toujours faire pour la transition vers l'état RUN
  //

  state = RUN;

  // Affichage sur LCD:
  update_message("RUN");

  // Allumer LED rouge:
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);

  // tempo sans clignotement          
  tempo = 300;
}

void init_LSC_screen(void) 
{
  //
  // Initialisation de l'écran du Braccio++
  //
  Braccio.lvgl_lock();
  lv_style_init(&style);
  lv_style_set_text_font(&style, &lv_font_montserrat_32);  //lv_font_montserrat_48);
  label = lv_label_create(lv_scr_act());
  Braccio.lvgl_unlock();
}

void update_message(const char* message) 
{
  //
  // écrire un mssage sur l'écran du Braccio++
  //
  Braccio.lvgl_lock();
  mess = message;
  lv_label_set_text_static(label, mess.c_str());
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_style(label, &style, LV_PART_MAIN);
  Braccio.lvgl_unlock();
}
