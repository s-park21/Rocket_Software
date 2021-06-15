#pragma once
#include "FS.h"
#include "SD.h"
#include "SPI.h"



class HandleSD {
    public:
    HandleSD(int LEDPin):
    LEDPin_{LEDPin}
    {
    }
    bool setup(char* dirname); 
    void appendFile(const char* filename, char* data);  

    private:
    char imuFile[30];
    char baroFile[30];
    char GPSFile[30];
    int LEDPin_;
    void errorBlink();
};
