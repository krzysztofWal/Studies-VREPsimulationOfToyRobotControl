
#include "b0RemoteApi.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "variables.h"
#include "functions.h"


void simulationStepStarted_CB(std::vector<msgpack::object>* msg){
    std::map<std::string,msgpack::object> data=msg->at(1).as<std::map<std::string,msgpack::object>>();
    std::map<std::string,msgpack::object>::iterator it=data.find("simulationTime");
    if (it!=data.end())
        simTime=it->second.as<float>();
}

void proxSensor1_CB(std::vector<msgpack::object>* msg){
    b0RemoteApi::readFloatArray(msg,sensorDataArray1,2);
    lastTimeReceived=cl->simxGetTimeInMs();
  //  printf(".");
}

void proxSensor2_CB(std::vector<msgpack::object>* msg){
    b0RemoteApi::readFloatArray(msg,sensorDataArray2,2);
    lastTimeReceived=cl->simxGetTimeInMs();
 //   printf(".");
}

void proxSensor3_CB(std::vector<msgpack::object>* msg){
    b0RemoteApi::readFloatArray(msg,sensorDataArray3,2);
    lastTimeReceived=cl->simxGetTimeInMs();
   // printf(".");
}

void proxSensor4_CB(std::vector<msgpack::object>* msg){
    b0RemoteApi::readFloatArray(msg,sensorDataArray4,2);
    lastTimeReceived=cl->simxGetTimeInMs();
  //  printf(".");
}

void colourSensor_CB(std::vector<msgpack::object> *msg) {
    b0RemoteApi::readFloatArray(msg,colourSensorDataArray,2);
    lastTimeReceived=cl->simxGetTimeInMs();
}

void leftSensor_CB(std::vector<msgpack::object> *msg) {
    b0RemoteApi::readFloatArray(msg,leftSensorDataArray,2);
    lastTimeReceived=cl->simxGetTimeInMs();
}

void rightSensor_CB(std::vector<msgpack::object> *msg) {
    b0RemoteApi::readFloatArray(msg,rightSensorDataArray,2);
    lastTimeReceived=cl->simxGetTimeInMs();
}

void proxSensor_CB(std::vector<msgpack::object>* msg) {
    sensorTrigger=b0RemoteApi::readFloat(msg, 1);
    lastTimeReceived=cl->simxGetTimeInMs();
}

int main()
{
    std::vector<bool> handleCheck {};
    cl=&client;

    std::tuple<std::string,std::vector<int>,float> args1{};
    msgpack::pack(packedArgs1,args1);

    auto inf = client.simxGetObjectHandle("lewy_dyn_joint", client.simxServiceCall());
    leftMotorHandle =  b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));
    inf = client.simxGetObjectHandle("prawy_dyn_joint", client.simxServiceCall());
    rightMotorHandle = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));

    inf = client.simxGetObjectHandle("czuj_nat_1", client.simxServiceCall());
    sensorHandle1 = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));
    inf = client.simxGetObjectHandle("czuj_nat_2", client.simxServiceCall());
    sensorHandle2 = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));
    inf = client.simxGetObjectHandle("czuj_nat_3", client.simxServiceCall());
    sensorHandle3 = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));
    inf = client.simxGetObjectHandle("czuj_nat_4", client.simxServiceCall());
    sensorHandle4 = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));

    inf = client.simxGetObjectHandle("czuj_koloru", client.simxServiceCall());
    colourSensorHandle = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));

    inf = client.simxGetObjectHandle("czuj_bocz_lewy", client.simxServiceCall());
    leftSensorHandle = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));
    inf = client.simxGetObjectHandle("czuj_bocz_prawy", client.simxServiceCall());
    rightSensorHandle = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));

    inf = client.simxGetObjectHandle("czuj_odleglosci", client.simxServiceCall());
    proxSensorHandle = b0RemoteApi::readInt(inf, 1);
    handleCheck.emplace_back(b0RemoteApi::readBool(inf,0));

    std::cout << handleCheck.size() << std::endl;
    bool temp {true};
    for (const auto &i : handleCheck) {
        if (!i) {temp = false;}
    }

    if (!temp) return 1;


    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted_CB));

    client.simxReadVisionSensor(sensorHandle1,client.simxDefaultSubscriber(proxSensor1_CB,0));
    client.simxReadVisionSensor(sensorHandle2,client.simxDefaultSubscriber(proxSensor2_CB,0));
    client.simxReadVisionSensor(sensorHandle3,client.simxDefaultSubscriber(proxSensor3_CB,0));
    client.simxReadVisionSensor(sensorHandle4,client.simxDefaultSubscriber(proxSensor4_CB,0));

    client.simxReadVisionSensor(colourSensorHandle,client.simxDefaultSubscriber(colourSensor_CB,0));

    client.simxReadVisionSensor(leftSensorHandle,client.simxDefaultSubscriber(leftSensor_CB,0));
    client.simxReadVisionSensor(rightSensorHandle,client.simxDefaultSubscriber(rightSensor_CB,0));

    client.simxReadProximitySensor(proxSensorHandle,client.simxDefaultSubscriber(proxSensor_CB,0));

    lastTimeReceived=client.simxGetTimeInMs();

    //czekanie aż czujniki zaczną przesyłać dane
    while (sensorDataArray1.size() == 0 || sensorDataArray2.size() == 0 || sensorDataArray3.size() == 0 || sensorDataArray4.size() == 0 || colourSensorDataArray.size() == 0 || leftSensorDataArray.size() == 0 || rightSensorDataArray.size() == 0 ) {
        client.simxSpinOnce();
    };

  //  client.simxSetJointTargetVelocity(leftMotorHandle,velocityThroughout,client.simxDefaultPublisher());
  //  client.simxSetJointTargetVelocity(rightMotorHandle,velocityThroughout,client.simxDefaultPublisher());

    currentTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1);

    while (currentTime-lastTimeReceived<2000)
    {
    //std::cout << "hopla;
        currentTime = b0RemoteApi::readFloat(client.simxGetSimulationTime(client.simxServiceCall()),1);

     //   checkForObstacle();
        checkForTurn();
        checkForCallibration();
        lineFollowing();
        colourCheckingResult = colourChecking();
//        std::cout <<  << " " << rightSensorDataArray.at(10) << std::endl;
     //   std::cout << static_cast<int>(turnPossibility) << std::endl;
          switch(colourCheckingResult) {
            case akcja:
            switch(whatAction()) {
            case wLewo:
                wLewoFunc();
                break;
            case wPrawo:
                wPrawoFunc();
                break;
            case postoj:
                postojFunc();
                break;
            case zawroc:
                if (!wlasniemZawrocil)
                    zawrocFunc();
                else
                    wlasniemZawrocil = false;
                break;
            case muzyka:
                muzykaFunc();
                break;
            case zagadka:
                zagadkaFunc();
                break;
            case obrotLewo:
                obrotLewoFunc();
                break;
            case obrotPrawo:
                obrotPrawoFunc();
                break;
            case wolniej:
                wolniejFunc();
                break;
            case sporoWolniej:
                sporoWolniejFunc();
                break;
            case sporoSzybciej:
                sporoSzybciejFunc();
                break;
            case szybciej:
                szybciejFunc();
                break;
            default:
                printf("main.ino It shouldn't have happened\n");
         }
        break;
    case zaDuzo:
        actionTooMuchBeeping = true;
        tooMuchBeepingStartTime = currentTime;
      // printf("main.ino Wrong sequence - too much colours\n");
        break;
    case zaMalo:
        if (!wlasniemZawrocil) {
        actionTooFewBeeping = true;
        tooFewBeepingStartTime = currentTime;}
       // std::cout << "main.ino Wrong sequence - too few colours " << actionTooFewBeeping << std::endl;
        break;
    default:
        {}
    }

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
    turning();

        client.simxSpinOnce();
        client.simxSleep(50);
    }

    std::cout << "Ended!" << std::endl;
    return(0);
}
