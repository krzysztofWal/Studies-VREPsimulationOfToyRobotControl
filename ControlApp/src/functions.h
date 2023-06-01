#ifndef VARIABLESANDFUNCTIONS_H_INCLUDED
#define VARIABLESANDFUNCTIONS_H_INCLUDED

#include "variables.h"

#define LEFT 1
#define RIGHT 2

//====colour reading===//
void cleancolourArrayTemp();
void cleancolourArray();
char colourFromSensorTranslate(const std::vector<float> &colourSensorDataArray);
colourCheckingType colourChecking();
uint8_t comparingWithArray(char fC, char sC, char tC);
//======turning======//
uint8_t leftOrRight();
void checkForTurn();
void turningLeft();
void turningRight();
void turning();
//======motor========//
void velocityHandler(int changeInVelocity);
float velocityThroughoutTranslator(const int &velocityThroughout);
//==following the line==//
int readSensors();
void lineFollowing();
//==actions===//
actionType whatAction();
void muzykaFunc();
void obrotLewoFunc();
void obrotPrawoFunc();
void zawrocFunc();
void zagadkaFunc();
void wPrawoFunc();
void wLewoFunc();
void postojFunc();
void szybciejFunc();
void wolniejFunc();
void sporoWolniejFunc();
//====LED===//
template<int LEDPIN> void handleLed(bool onOff, int& ledState);
//===CALLIBRATION====//
void sensorArrayCallibration();
void checkForCallibration();

//====================================================MOTOR CONTROL============================//

void velocityHandler(int changeInVelocity) {
    int temp = velocityThroughout;
    temp += changeInVelocity;
    if (temp < velocityMinNumber) {
        temp = velocityMinNumber;
        printf("motorControl.h: Chciales za bardzo zwolnic\n");
    } else if (temp > velocityMaxNumber) {
        temp = velocityMaxNumber;
        printf("motorControl.h: Chciales za bardzo przyspieszyc\n");
    }
    velocityThroughout = temp;
   printf("motorControl.h: predkosc zmieniona na: ");printf("%i",velocityThroughout);printf("\n");
}
float velocityThroughoutTranslator(const int &velocityThroughout) {
    switch(velocityThroughout) {
    case -2:
        return 6.*0.2;
    case - 1:
        return 6.*0.4;
    case 0:
        return 6.*0.6;
    case 1:
        return 6.*0.8;
    case 2:
        return 6.;
    default:
        printf("velocityThroughoutTranslator.h: Shouldn't have happended");
        return 0;
    }
};

//==============================FOLLOWING THE LINE=============================================//

int readSensors() {

/* algorytm: QTRs:SensorsLibrary
Copyright (c) 2014 Pololu Corporation.  For more information, see

http://www.pololu.com/
http://forum.pololu.com/

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/
    unsigned char i, on_line = 0;
    unsigned long avg = 0;
    unsigned int sum = 0;

    sensorValues[0] = 1000-static_cast<double>(sensorDataArray1.at(10))*1000/0.74;
    sensorValues[1] = 1000-static_cast<double>(sensorDataArray2.at(10))*1000/0.74;
    sensorValues[2] = 1000-static_cast<double>(sensorDataArray3.at(10))*1000/0.74;
    sensorValues[3] = 1000-static_cast<double>(sensorDataArray4.at(10))*1000/0.74;


    for(i=0;i<4;i++) {
        int value = sensorValues[i];
        if(value > 200) {
            on_line = 1;
        }
        if(value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }

    if(!on_line)
    {
        if(_lastValue < (3)*1000/2)
            return 0;
        else
            return 3000;
    }

    _lastValue = avg/sum;
    std::cout << _lastValue << std::endl;
    return _lastValue;

}

void lineFollowing() {
    unsigned int linePos = readSensors();
    int error = 1500 - linePos;


    float regulation;
    if (error == -1500 || error == 1500)
        regulation = (error) * turboKp + turboKd*(error - lastError);
    else
        regulation = (error) * Kp + Kd*(error - lastError);
    lastError = error;
    std::cout << linePos << " " << error << std::endl;
   client.simxSetJointTargetVelocity(leftMotorHandle, velocityThroughoutTranslator(velocityThroughout) + regulation,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle, velocityThroughoutTranslator(velocityThroughout) - regulation,client.simxDefaultPublisher());
}

// =============================================COLOUR READING===================================//
void cleancolourArrayTemp() {
    colourArrayTemp[2] = '-';
    colourArrayTemp[1] = '-';
    colourArrayTemp[0] = '-';
}

void cleancolourArray() {
    colourArray[2] = '-';
    colourArray[1] = '-';
    colourArray[0] = '-';
}

char colourFromSensorTranslate(const std::vector<float> &colourSensorDataArray) {
    if(colourSensorDataArray.at(11) > 0.35 && colourSensorDataArray.at(12) < 0.1 && colourSensorDataArray.at(13) < 0.1) {
        return 'R';
    } else if (colourSensorDataArray.at(11) < 0.1 && colourSensorDataArray.at(12) > 0.35 && colourSensorDataArray.at(13) < 0.1) {
        return 'G';
    } else if (colourSensorDataArray.at(11) < 0.1 && colourSensorDataArray.at(12) < 0.1 && colourSensorDataArray.at(13) > 0.35) {
        return 'B';
    } else {
        return 'C';
    }
}

colourCheckingType colourChecking() {
    colourCheckingType temp = nic;
    colour = colourFromSensorTranslate(colourSensorDataArray);

    if (colour != previousColour) {
        timeOfTheColourChange = currentTime;
        flag = true;
    }
    if(currentTime - timeOfTheColourChange > blindInterval && flag) {
        if (colour != 'C') {
            if(!actionTooMuchBeeping) {
                if (colour == 'R') {
                    handleLed<GREEN_LED_PIN>(false, greenLedState);
                    handleLed<BLUE_LED_PIN>(false, blueLedState);
                    handleLed<RED_LED_PIN>(true, redLedState);
                } else if (colour == 'G') {
                    handleLed<RED_LED_PIN>(false, redLedState);
                    handleLed<BLUE_LED_PIN>(false, blueLedState);
                    handleLed<GREEN_LED_PIN>(true, greenLedState);
                } else if (colour == 'B') {
                    handleLed<RED_LED_PIN>(false, redLedState);
                    handleLed<GREEN_LED_PIN>(false, greenLedState);
                    handleLed<BLUE_LED_PIN>(true, blueLedState);
                }
            }
            if(colourArrayTemp[2] != '-') { // tablica jest juz pelna przed dodaniem - za dluga sekwencja!
                cleancolourArrayTemp();
                temp = zaDuzo;
            } else { // jesli nie, dodaj do tablicy, na razie sekwencjaa nie za dluga
                colourArrayTemp[2] = colourArrayTemp[1];
                colourArrayTemp[1] = colourArrayTemp[0];
                colourArrayTemp[0] = colour;
                std::cout << "colourReading.h " << currentTime << " Dodano do tablicy " <<  colour << std::endl;
            }
            //jezeli kolor jest czarny
        } else {
            handleLed<RED_LED_PIN>(false, redLedState);
            handleLed<GREEN_LED_PIN>(false, greenLedState);
            handleLed<BLUE_LED_PIN>(false, blueLedState);
            //jezeli tablica jest pelna
            if (colourArrayTemp[2] != '-' && colourArrayTemp[1] != '-' && colourArrayTemp[0] != '-') {
                colourArray[0] = colourArrayTemp[0];
                colourArray[1] = colourArrayTemp[1];
                colourArray[2] = colourArrayTemp[2];
                cleancolourArrayTemp();
                temp = akcja; // jest ok
                //jezeli tablica nie jest pelna - zawsze wtedy ostatnia komorka jest niezapelniona ale tablica nie jest tez pusta - czyli pierwsza komorka jest zapisana
            } else if (colourArrayTemp[2] == '-' && colourArrayTemp[0] != '-') {
                cleancolourArrayTemp();
                temp = zaMalo; //sekwencja skonczyla sie przedwczesnie - za malo kolorow
            }
        }
    flag = false;
    }
    previousColour = colour;
    return temp;
}

uint8_t comparingWithArray(char fC, char sC, char tC) {
    if (colourArray[2] == fC && colourArray[1] == sC && colourArray[0] == tC)
        return 1;
    else
        return 0;
}
//==============================================TURNING=========================================//
uint8_t leftOrRight() {
    srand(static_cast<long>(currentTime*1000));
    int i = rand() / (RAND_MAX + 1.0) * 100.0;
    if (i < 33) {
        return LEFT;
    } else if  (i< 66) {
        return RIGHT;
    } else {
        return 0;
    }
}

void checkForTurn() {
    if (leftSensorDataArray.at(10) < threshold && rightSensorDataArray.at(10) < threshold) {
        turnPossibility = 1;
    }
}

void turningLeft() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < turningTime) {
    client.simxSetJointTargetVelocity(leftMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    }
}

void turningRight() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < turningTime) {
    client.simxSetJointTargetVelocity(leftMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    }
}

void turning(){

    //jesli jest mozliwy skret
    if(turnPossibility) {//[0] == 1 || turnPossibility[1] == 1) {
        uint8_t turnTemp = 0; // 1 -skrecamy w LEFT;  2 - skrecamy w RIGHT
        if (turnSelection == 0) { //nie ma instrukcji co do skretu
            //turnTemp = leftOrRight();
	    turnTemp = 0;
        } else {
            turnTemp = turnSelection;
        }

    // po skrecie nalezy wyzerowac turn Possibilities i turnSelection
    // zerowanie turn selection, zeby nie zostala w pamieci poprzednia instrukcja

    if (turnTemp == LEFT) {
        turningLeft();
        turnPossibility = 0;
        turnSelection = 0;
    } else if (turnTemp == RIGHT) {
        turningRight();
        turnPossibility = 0;
        turnSelection = 0;
    } else {
        //jedzie prosto
        turnPossibility = 0;
        turnSelection = 0;
    }
    }
}

//===================================ACTIONS==================================================//
actionType whatAction() {
    /*1*/   if(comparingWithArray('R','B','R')) {//na nastepnym skrec w prawo
                return wPrawo;
    /*2*/   } else if(comparingWithArray('R','G','R')) {//zawroc
                return zawroc;
    /*3*/   } else if(comparingWithArray('B','R','B')) {//na nastepnym skrec w lewo 3 1 3
                return wLewo;
    /*3*/   } else if(comparingWithArray('B','G','B')) {
                return postoj;
    /*4*/   } else if(comparingWithArray('G','R','G')) {
                return muzyka;
    /*5*/   } else if(comparingWithArray('G','B','G')) {
                return zagadka;
    /*6*/   } else if(comparingWithArray('R','B','G')) {
                return obrotLewo;
    /*7*/   } else if(comparingWithArray('R','G','B')) {
                return wolniej;
    /*8*/   } else if(comparingWithArray('G','B','R')) {
                return obrotPrawo;
    /*9*/   } else if(comparingWithArray('G','R','B')) {
                return sporoSzybciej;
    /*10*/  } else if(comparingWithArray('B','R','G')) {
                return sporoWolniej;
    /*11*/  } else if(comparingWithArray('B','G','R')) {
                return szybciej;
    /*12*/  }
}

void muzykaFunc() {
    isMusicPlaying ? isMusicPlaying = false : isMusicPlaying = true;
}

void obrotLewoFunc() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 2.58) {
    client.simxSetJointTargetVelocity(leftMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    }
}

void obrotPrawoFunc() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 2.58) {
    client.simxSetJointTargetVelocity(leftMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    }
}

void zawrocFunc() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < turningTime*2) {
    client.simxSetJointTargetVelocity(leftMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    }
 wlasniemZawrocil = true;
}

void zagadkaFunc() {
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 5.4) {
    client.simxSetJointTargetVelocity(leftMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    }
    startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 5.25) {
    client.simxSetJointTargetVelocity(leftMotorHandle,-normalnaPredkosc,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,normalnaPredkosc,client.simxDefaultPublisher());
    }
}

void wPrawoFunc() {
    turnSelection = RIGHT;
}

void wLewoFunc() {
    turnSelection = LEFT;
}

void postojFunc() {
 float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < stillTime) {
    client.simxSetJointTargetVelocity(leftMotorHandle,0.,client.simxDefaultPublisher());
    client.simxSetJointTargetVelocity(rightMotorHandle,0.,client.simxDefaultPublisher());
    }
client.simxSetJointTargetVelocity(leftMotorHandle,velocityThroughout,client.simxDefaultPublisher());
client.simxSetJointTargetVelocity(rightMotorHandle,velocityThroughout,client.simxDefaultPublisher());
}

void szybciejFunc() {
    velocityHandler(1);
}

void wolniejFunc() {
    velocityHandler(-1);
}

void sporoSzybciejFunc() {
    velocityHandler(2);
}

void sporoWolniejFunc() {
    velocityHandler(-2);
}
//=================================================LED======================================//
template<int LEDPIN>
void handleLed(bool onOff, int& ledState) {
    if (onOff) { //on
        if (ledState == 0) {
            ledState = 1;
            switch(LEDPIN) {
            case RED_LED_PIN:
                client.simxCallScriptFunction("turnREDon@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
              //  std::cout << "red on" << std::endl;
                break;
            case BLUE_LED_PIN:
                client.simxCallScriptFunction("turnBLUEon@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
              //  std::cout << "blue on" << std::endl;
                break;
            case GREEN_LED_PIN:
                client.simxCallScriptFunction("turnGREENon@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
               // std::cout << "green on" << std::endl;
                break;
            default:
                std::cout << "handleLed():  Shouldn't have happened" << std::endl;
            }
        }

    }
    else { //off
        if (ledState != 0) {
            ledState = 0;
            switch(LEDPIN) {
            case RED_LED_PIN:
               client.simxCallScriptFunction("turnREDoff@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
              //  std::cout << "red off" << std::endl;
                break;
            case BLUE_LED_PIN:
               client.simxCallScriptFunction("turnBLUEoff@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
              //  std::cout << "blue off" << std::endl;
                break;
            case GREEN_LED_PIN:
                  client.simxCallScriptFunction("turnGREENoff@DefaultCamera","sim.scripttype_childscript",packedArgs1.str().c_str(),packedArgs1.str().size(),client.simxServiceCall());
               // std::cout << "green off" << std::endl;
                break;
            default:
                std::cout << "handleLed():  Shouldn't have happened" << std::endl;
            }
        }
    }
}

void checkForObstacle() {
    if (sensorTrigger > 0)
        zawrocFunc();
}

void tooMuchBeeping() {
if(actionTooMuchBeeping) {
        timeDifference = currentTime - tooMuchBeepingStartTime;
        if(timeDifference < 0.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 1.) {
            if (redLedState == 0) handleLed<RED_LED_PIN>(true,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 1.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 0) handleLed<GREEN_LED_PIN>(true,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 2.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 3) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 0) handleLed<BLUE_LED_PIN>(true,blueLedState);
        } else {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
            actionTooMuchBeeping = false;
        }
    }
}

void tooFewBeeping() {
 if(actionTooFewBeeping && !actionTooMuchBeeping) {
        timeDifference = currentTime - tooFewBeepingStartTime;
        std::cout << timeDifference << std::endl;
        if(timeDifference < 0.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 1.) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 0) handleLed<BLUE_LED_PIN>(true,blueLedState);
        } else if (timeDifference < 1.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 0) handleLed<GREEN_LED_PIN>(true,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 2.2) {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else if (timeDifference < 3) {
            if (redLedState == 0) handleLed<RED_LED_PIN>(true,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
        } else {
            if (redLedState == 1) handleLed<RED_LED_PIN>(false,redLedState);
            if (greenLedState == 1) handleLed<GREEN_LED_PIN>(false,greenLedState);
            if (blueLedState == 1) handleLed<BLUE_LED_PIN>(false,blueLedState);
            actionTooFewBeeping = false;
        }
    }
}

//=============================================CALLIBRATION================================//

void sensorArrayCallibration() {
    //miejsce na kalibracje z wykorzystaniem biblioteki producenta listwy, podczas kalibracji robot "skanuje" linie i okolice
    float startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 0.9) {
        client.simxSetJointTargetVelocity(leftMotorHandle,-callibrationVelocity,client.simxDefaultPublisher());
        client.simxSetJointTargetVelocity(rightMotorHandle,callibrationVelocity,client.simxDefaultPublisher());
    }
    for (int i{0}; i<4; i++) {
        startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
        while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 1.8) {
            client.simxSetJointTargetVelocity(leftMotorHandle,callibrationVelocity,client.simxDefaultPublisher());
            client.simxSetJointTargetVelocity(rightMotorHandle,-callibrationVelocity,client.simxDefaultPublisher());
        }
        startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
        while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 1.8) {
            client.simxSetJointTargetVelocity(leftMotorHandle,-callibrationVelocity,client.simxDefaultPublisher());
            client.simxSetJointTargetVelocity(rightMotorHandle, callibrationVelocity,client.simxDefaultPublisher());
        }
    }
    startTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) ;
    while (b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1) - startTime < 0.9) {
        client.simxSetJointTargetVelocity(leftMotorHandle,callibrationVelocity,client.simxDefaultPublisher());
        client.simxSetJointTargetVelocity(rightMotorHandle,-callibrationVelocity,client.simxDefaultPublisher());
    }
}

void checkForCallibration() {
    if(sensorTrigger>0 && !forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight) {
        forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight = true;
    }
    if(forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight && sensorTrigger == 0) {
        pierwszyRaz = true;
        if (!timeSet) {
            timeOfTheFirstTime = currentTime;
            timeSet = true;
        }
    }

    if (currentTime - timeOfTheFirstTime > callibrationWaitTime && pierwszyRaz) {
        std::cout<<"in" <<std::endl;
        pierwszyRaz = false;
        forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight = false;
        timeSet = false;
    }

    if (pierwszyRaz && sensorTrigger >0) {
        secondTime = true;
    }

    if(secondTime && sensorTrigger ==0) {
        jeszczeRaz_zabierzMnieDoPierwszejCostam  = true;
    }

    if(jeszczeRaz_zabierzMnieDoPierwszejCostam) {
        forTheFirstTimeInForeverTherellBeMusicAndTherellBeLight = false;
        pierwszyRaz = false;
        secondTime = false;
        jeszczeRaz_zabierzMnieDoPierwszejCostam  = false;
        timeSet = false;
        sensorArrayCallibration();
    }

}


#endif // VARIABLESANDFUNCTIONS_H_INCLUDED
