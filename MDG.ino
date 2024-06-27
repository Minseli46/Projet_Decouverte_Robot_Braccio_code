#include <Braccio.h>
#include <Servo.h>
#include <math.h>
// Définition des servomoteurs du Braccio
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Paramètres DH du Braccio (mis à jour en cm)
float d1 = 7.0;               // Hauteur de la base à l'épaule
float a1 = 0.0;               // Distance le long de l'axe x1
float alpha1 = M_PI / 2.0;    // Angle entre z0 et z1

float d2 = 0.0;               // Distance le long de l'axe z1
float a2 = 12.5;              // Distance le long de l'axe x2
float alpha2 = 0.0;           // Angle entre z1 et z2

float d3 = 0.0;               // Distance le long de l'axe z2
float a3 = 12.5;              // Distance le long de l'axe x3
float alpha3 = 0.0;           // Angle entre z2 et z3

float d4 = 0.0;               // Distance le long de l'axe z3
float a4 = 5.8;               // Distance le long de l'axe x4
float alpha4 = 0.0;    // Angle entre z3 et z4

float d5 = 0.0;               // Distance le long de l'axe z4
float a5 = 13.0;               // Distance le long de l'axe x5
float alpha5 = 0.0;   // Angle entre z4 et z5

float d6 = 0.0;               // Distance le long de l'axe z5
float a6 = 0.0;               // Distance le long de l'axe x6
float alpha6 = -M_PI / 2.0;           // Angle entre z5 et z6

// Fonction pour calculer la matrice de transformation homogène
void calculateDHMatrix(float theta, float d, float a, float alpha, float matrix[4][4]) {
    matrix[0][0] = cos(theta); matrix[0][1] = -sin(theta) * cos(alpha); matrix[0][2] = sin(theta) * sin(alpha); matrix[0][3] = a * cos(theta);
    matrix[1][0] = sin(theta); matrix[1][1] = cos(theta) * cos(alpha); matrix[1][2] = -cos(theta) * sin(alpha); matrix[1][3] = a * sin(theta);
    matrix[2][0] = 0;          matrix[2][1] = sin(alpha);               matrix[2][2] = cos(alpha);              matrix[2][3] = d;
    matrix[3][0] = 0;          matrix[3][1] = 0;                        matrix[3][2] = 0;                       matrix[3][3] = 1;
}

// Fonction pour multiplier deux matrices 4x4
void multiplyMatrix(float A[4][4], float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Fonction pour calculer la position de l'effecteur en temps réel
void calculateForwardKinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float& x, float& y, float& z) {
    // Matrices de transformation pour chaque articulation
    float T1[4][4], T2[4][4], T3[4][4], T4[4][4], T5[4][4], T6[4][4];
    
    // Matrice de transformation globale
    float T[4][4];

    // Calcul des matrices de transformation homogène pour chaque articulation
    calculateDHMatrix(theta1, d1, a1, alpha1, T1);
    calculateDHMatrix(theta2, d2, a2, alpha2, T2);
    calculateDHMatrix(theta3, d3, a3, alpha3, T3);
    calculateDHMatrix(theta4, d4, a4, alpha4, T4);
    calculateDHMatrix(theta5, d5, a5, alpha5, T5);
    calculateDHMatrix(theta6, d6, a6, alpha6, T6);

    // Calcul de la matrice de transformation globale
    float T_temp1[4][4], T_temp2[4][4], T_temp3[4][4], T_temp4[4][4];
    multiplyMatrix(T1, T2, T_temp1);
    multiplyMatrix(T_temp1, T3, T_temp2);
    multiplyMatrix(T_temp2, T4, T_temp3);
    multiplyMatrix(T_temp3, T5, T_temp4);
    multiplyMatrix(T_temp4, T6, T);

    // Extraction des coordonnées x, y, z de la matrice de transformation globale
    x = T[0][3];
    y = T[1][3];
    z = T[2][3];
}

void setup() {
    // Initialisation des servomoteurs et positionnement initial
    base.attach(2);         // Exemple : broche 2 pour le servomoteur de la base
    shoulder.attach(3);     // Exemple : broche 3 pour le servomoteur de l'épaule
    elbow.attach(4);        // Exemple : broche 4 pour le servomoteur du coude
    wrist_rot.attach(5);    // Exemple : broche 5 pour le servomoteur du poignet rotation
    wrist_ver.attach(6);    // Exemple : broche 6 pour le servomoteur du poignet vertical
    gripper.attach(7);      // Exemple : broche 7 pour le servomoteur de la pince

    // Position initiale du Braccio
    Braccio.begin();

    // Initialisation du moniteur série pour afficher les positions
    Serial.begin(9600);
}

void loop() {

    Braccio.ServoMovement(20,         0, 90, 90, 90, 90,  73);  
    // Lecture des angles actuels des servomoteurs (convertis en radians)
    float theta1 = base.read() * M_PI / 180.0;
    float theta2 = shoulder.read() * M_PI / 180.0;
    float theta3 = elbow.read() * M_PI / 180.0;
    float theta4 = wrist_ver.read() * M_PI / 180.0;
    float theta5 = wrist_rot.read() * M_PI / 180.0;
    float theta6 = gripper.read() * M_PI / 180.0;

    // Calcul de la position de l'effecteur en temps réel
    float x, y, z;
    calculateForwardKinematics(theta1, theta2, theta3, theta4, theta5, theta6, x, y, z);

    // Affichage de la position de la pince en temps réel
    Serial.print("Position de la pince - x = ");
    Serial.print(x);
    Serial.print(" cm, y = ");
    Serial.print(y);
    Serial.print(" cm, z = ");
    Serial.print(z);
    Serial.println(" cm");

    // Attente pour éviter une mise à jour trop fréquente de la position
    delay(500); // 500 millisecondes (0.5 seconde)
}
