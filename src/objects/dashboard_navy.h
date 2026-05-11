#include "Arduino.h"
#include <TFT_eSPI.h>   // https://github.com/Bodmer/TFT_eSPI

/*
 * dashboard.h - Library to drive a 80x160px color LCD as rc truck dashboard
 *
 * https://github.com/Gamadril/Rc_Engine_Sound_ESP32
 * MIT License
 */

#ifndef DASHBOARD_H_NAVY
#define DASHBOARD_H_NAVY

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

// NAVY 
#define RADAR_R  115    // Radius radar
#define RADAR_CX 120    // Centre X
#define RADAR_CY 120    // Centre Y

// Colors
#define RADAR_BG   TFT_BLACK
#define RADAR_LINE TFT_GREEN
#define RADAR_GRID TFT_DARKGREEN
#define RADAR_ECHO TFT_GREENYELLOW

// Version : V1 basic, V2 avec effet de balayage plus fluide et échos qui bougent
#define VERSION 1
//#define VERSION 2


/**
 * @brief Class acts as a wrapper around TFT_eSPI routines to draw dashboard elements.
 *
 */
class Dashboard
{
    // Datas
private:
    TFT_eSPI _tft = TFT_eSPI();
    uint32_t lastupdate_ms = 0; 
    uint16_t alpha = 0;

    // Angle du balayage (en degrés)
    float angle = 0.0;
    float angleStep = 5.0;  // Speed rotation
    float lastAngle = 0.0;


    // Pour simuler quelques échos
    //const uint8_t NB_ECHOS = 20;
    #define MAX_ECHOS 10
    struct Echo {
        float angle;
        int distance;
        uint16_t color;
        uint32_t ttl;   // durée de vie en ms
        int pos_x;      // Position
        int pos_y;
        int offset_x;   // pour faire bouger les échos
        int offset_y;
        bool active;
    };
    //struct Echo {
    //    float angleDeg;
    //    float dist;
    //};
    Echo echos[MAX_ECHOS];


    // Methodes
public:
    /**
     * Default constructor
     */
    Dashboard();
    /*
    */

    /**
     * Performs initialization of the display, rotates the coordinate system of the screen
     * (0,0) is the upper left corner if the connector is on the top.
     * Draws static dashboard elements and sets all values to 0.
     */
    void init(uint8_t value);

    
    void update(uint32_t value);


private:
    void drawFrame();
    void drawArc(int16_t cx, int16_t cy, int16_t r, int16_t deg_start, int16_t deg_end,
                 uint32_t color);

#if VERSION == 2
    // Radar specific V2
    void drawRadarBackground();
    void drawSweep(float deg, uint16_t color, bool erase);
    void updateEchos();
    void updatePositionEchos(float angle);
    void maybeCreateEcho(float angle);
    void drawEchos(float oldAngle, float newAngle);
    void clearEcho(int i);

#else
    // radar specific V1
    void setupEchos();
    void drawGrid();
    void eraseSweepLine(float aDeg);    
    void drawSweepLine(float aDeg);
    void drawEchos(float sweepAngleDeg);
#endif
    //
    float normalizeAngle(float value);

};

#endif
// End of Vrevic dashboard