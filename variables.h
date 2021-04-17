
#ifndef VARIABLES_H_INCLUDED
#define VARIABLES_H_INCLUDED

#define RED_LED_PIN 6
#define BLUE_LED_PIN 5
#define GREEN_LED_PIN 3

// =================================================SIMULATION============================================================//

std::stringstream packedArgs1;
//zmienne globalne
static float simTime=0.0f;
static long lastTimeReceived=0;
static b0RemoteApi* cl=nullptr;

b0RemoteApi client("b0RemoteApi_c++Client","b0RemoteApi");
int leftMotorHandle{-1};
int rightMotorHandle{-1};
int sensorHandle1{-1};
int sensorHandle2{-1};
int sensorHandle3{-1};
int sensorHandle4{-1};
int colourSensorHandle{-1};
int leftSensorHandle{-1};
int rightSensorHandle{-1};
int proxSensorHandle{-1};

float currentTime{0};
float normalnaPredkosc = 6.;
// ======= side sensors ============ //
static std::vector<float> leftSensorDataArray{};
static std::vector<float> rightSensorDataArray{};
// ======= colour sensor=============//
static std::vector<float> colourSensorDataArray{};
// ======== sensor Array ============//
static std::vector<float> sensorDataArray1{};
static std::vector<float> sensorDataArray2{};
static std::vector<float> sensorDataArray3{};
static std::vector<float> sensorDataArray4{};
//========proximity sensor ==========/
float sensorTrigger{0.};

//====================================================MOTOR CONTROL=====================================================//

int velocityThroughout{0};
const int velocityMinNumber{-2};
const int velocityMaxNumber{2};

//==========================================================LEDS=========================================================//

int redLedState{0};
int blueLedState{0};
int greenLedState{0};

bool actionTooMuchBeeping{false};
float tooMuchBeepingStartTime{0.};

bool actionTooFewBeeping{false};
float tooFewBeepingStartTime{0.};

float timeDifference{0.}; // uzywana

bool wlasniemZawrocil{false};
const float stillTime{4.}; // czas postoju

bool isMusicPlaying{false};

enum actionType {
    wPrawo,
    wLewo,
    zawroc,
    postoj,
    muzyka,
    zagadka,
    obrotLewo,
    obrotPrawo,
    wolniej,
    sporoWolniej,
    szybciej,
    sporoSzybciej
};

//=======================================================TURNING============================================================//

float threshold{0.5};
uint8_t turnSelection{0}; // 0 - prosto, 1 - lewo, 2 - prawo
uint8_t turnPossibility{0};

const float turningTime{.65f};

//=====================================================FOLLOWNIG THE LINE===================================================//
double sensorValues[4] = {0.,0.,0.,0.};

int _lastValue {0};
unsigned int linePos {0};
float error {0};
float lastError {0};
// === PD regulator === //

const float turboKp = 0.001;
const float turboKd = 0.020;
const float Kp = 0.0004;
const float Kd = 0.008;
//====================================================== COLOUR READING ====================================================//

const float blindInterval {.15f};
bool flag {false};
float timeOfTheColourChange{0};

char colour {'-'};
char previousColour {'-'};
char colourArray[3] = { '-','-','-' }; // - tablica niezapisana
char colourArrayTemp[3] = { '-','-','-' };

enum colourCheckingType {
    akcja,
    zaMalo,
    zaDuzo,
    nic
};
colourCheckingType colourCheckingResult {nic};

//===========================================================================================================================//

const float callibrationVelocity {3};
float callibrationWaitTime{2.};

float timeOfTheFirstTime {0.};
bool forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight {false};
bool secondTime {false};
bool pierwszyRaz {false};
bool jeszczeRaz_zabierzMnieDoPierwszejCostam {false};
bool timeSet {false};

//===========================================================================================================================//



#endif // VARIABLES_H_INCLUDED
