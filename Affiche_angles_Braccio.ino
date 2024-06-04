//
// Programme pour afficher les angles des 6 moteurs du Braccio 
// quand il est dans une position donnée.
//
// Plateforme MERI - Affiche_angles_Braccio++.ino
//
// v1.0 2023-03-10 Théo Parrinello & Fatima Cruz-Solano : version initiale
// v1.1 2023-03-28 JLC : mise en forme
//

// Le but de ce pogramme est de permettre à l'utilisateur de faire afficher 
// dans le terminal Arduino les valeurs des angles des 6 servomoteurs du bras 
// robotisé Braccio++ mis dans des positions successives qui décomposent
// un mouvement complet.
// - un appui sur le bouton ENTER permetd'afficher les valeur des 6 angles
//   de la position courante du Braccio++
// - un appui sur le bouton SELECT du joystick permet de finir l'acquisition.
//
// A la fin de l(acquisition on obtient sur le moniteur Arduino un affichage 
// semblalble à : 
// 
//   //============== Position de travail ========================//
//   const six_angles WORK_POS[] = \
//   {{211.0, 158.0, 219.0, 62.0, 106.0, 91.0},
//   {169.0, 158.0, 220.0, 64.0, 108.0, 92.0},
//   {169.0, 158.0, 159.0, 101., 132.0, 92.0},
//   {169.0, 158.0, 154.0, 93.0, 149.0, 150.0},
//   {169.0, 158.0, 160.0, 160.0, 130.0, 139.0},
//   {169.0, 159.0, 192.0, 87.0, 115.0, 139.0},
//   {211.0, 159.0, 193.0, 87.0, 115.0, 139.0},
//   {-1}};
//   //================= Fin positions ===========================//
//
// Il ne reste plus qu'à copier le code C++ entre les lignes //====...====// et 
// le coller dans le fichier .ino cible.
//

#include <Braccio++.h>

//////////////////////////////////////////
// Les variables globales et les macros //
//////////////////////////////////////////

#define BUTTON_SELECT 3
#define BUTTON_ENTER  6

// Braccio ++ joints
auto gripper    = Braccio.get(1);
auto wristRoll  = Braccio.get(2);
auto wristPitch = Braccio.get(3);
auto elbow      = Braccio.get(4);
auto shoulder   = Braccio.get(5);
auto base       = Braccio.get(6);

bool movement = false;

float angles[6];


void setup() 
{
  // Attendre le démarrage de la liason série:
  Serial.begin(115200);
  while(!Serial){}

  // Démarrer le Braccio:
  if (Braccio.begin())
  {
    delay(500);
    Braccio.disengage();
    delay(500);
    Serial.println("Braccio++ prêt :\n");
    Serial.println("  1 - Mettre le bras dans la position voule et appuyer sur ENTER pour faire ");
    Serial.println("      afficher les poistions angulaires des 6 moteur du Braccio++.");
    Serial.println("  2 - Répéter 1 - pour toutes les positions du bras qui décomposent le");
    Serial.println("      mouvement complet.");
    Serial.println("À la fin, appuyer sur le joystcik-SELECT pour terminer : ");
    Serial.println("le code C++ à copier coller est entre les 2 lignes '//=====...=====//\n\n\n\n");
  }
  else
  {
    Serial.println("PB démarrage Braccio++, please check and reset");
    while (true) {}
  }

  // Afficher l'entête du code C++ qui devra être copié/collé :
  Serial.println("//============== Position de travail ========================//");
  Serial.println("const six_angles WORK_POS[] = {\\\n");
}

void loop() 
{
  int pressedKey = Braccio.getKey();

  if (pressedKey == BUTTON_ENTER)
  {
    Braccio.positions(angles);
    Serial.println(" {"+String(angles[0])+", "+String(angles[1])+", "+String(angles[2])+", " +\
                        String(angles[3])+", "+String(angles[4])+", "+String(angles[5])+"},");
   }
  else if  (pressedKey == BUTTON_SELECT)
  {
    Serial.println(" {-1}};");
    Serial.println("//================= Fin positions ===========================//");
    Serial.println("\n\n\nTu peux maintenant copier/coller le code entre les deux lignes //====...====// ");
  } 
}
