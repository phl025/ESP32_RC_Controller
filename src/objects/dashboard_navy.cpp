// LCD dashboard from Frevic.
// See: https://www.rc-modellbau-portal.de/index.php?threads/esp32-arduino-rc-sound-und-licht-controller.7183/post-490891

/*
 * dashboard.cpp - Library to drive a 80x160px color LCD as rc truck dashboard
 *
 * https://github.com/Gamadril/ESP32_LASE
 * MIT License
 */
// From TheDIYGuy999 : RC_Engine_Sound_V9.13 but split in two files
//
#include "objects/global.h"
//
#ifdef NAVY_DASHBOARD
#include "objects/dashboard_navy.h"

#include <TFT_eSPI.h>
#include <stdint.h>
// #include "dashLogos.h"

Dashboard::Dashboard() {}

void Dashboard::init(uint8_t value)
{
	_tft.init();
	_tft.setRotation(value); // 3 = normal, 1 = upside down
	// Add PLA
	/*
	// Color checker
	_tft.fillScreen(TFT_BLACK);
	_tft.drawCentreString("BLACK", 120, 34, 1);
	Serial.printf("* TFT test : BLACK");
	delay(500);
	_tft.fillScreen(TFT_WHITE);
	_tft.drawCentreString("WHITE", 120, 34, 1);
	Serial.printf(", WHITE");
	delay(500);
	_tft.fillScreen(TFT_BLUE);
	_tft.drawCentreString("BLUE", 120, 34, 1);
	Serial.printf(", BLUE");
	delay(500);
	_tft.fillScreen(TFT_RED);
	_tft.drawCentreString("RED", 120, 34, 1);
	Serial.printf(", RED");
	delay(500);
	_tft.fillScreen(TFT_GREEN);
	_tft.drawCentreString("GREEN", 120, 34, 1);
	Serial.printf(", GREEN\n");
	delay(500);
	//
	*/
	_tft.fillScreen(TFT_BLACK);
	_tft.fillScreen(RADAR_BG);

#if VERSION == 2
	// fond de radar
	drawRadarBackground();

	// init des échos
	for (int i = 0; i < MAX_ECHOS; i++)
	{
		echos[i].active = false;
	}
	randomSeed(analogRead(0));
	angle = 2.0;
	drawSweep(angle, RADAR_LINE, false);
	lastAngle = 2.0;

#else
	randomSeed(analogRead(0));
	setupEchos();
	drawGrid();
	Serial.println("\n ECHO positions: \n");
	for (uint8_t i = 0; i < MAX_ECHOS; i++)
	{
		Serial.printf("ECHO %i: angle=%f, distance=%f\n", i, echos[i].angle, echos[i].distance);
	}
	angle = 2.0;
	drawSweepLine(angle);
	lastAngle = 2.0;
#endif
	//
	lastupdate_ms = millis();
}

#if VERSION == 2
/// @brief Update the dashboard display with new values.
/// @param value
void Dashboard::update(uint32_t cur_ms)
{
	if (cur_ms > lastupdate_ms + 50)
	{
		// Effacer l’ancienne ligne de balayage avec une ligne plus sombre (effet de persistance)
		drawSweep(lastAngle, RADAR_BG, true); // efface l’ancienne

		// Dessiner la nouvelle ligne de balayage
		angle = normalizeAngle(lastAngle + angleStep);
		//if (angle >= 360) angle -= 360;
		drawSweep(angle, RADAR_LINE, false);

		if (angle >= 5.0 && angle < 10.0)
		{
			// cercles de portée
			_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, TFT_DARKGREEN);
			_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R * 2 / 3, TFT_DARKGREEN);
			_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R / 3, TFT_DARKGREEN);

			// croix centrale
			_tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, TFT_DARKGREEN);
			_tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, TFT_DARKGREEN);
		}

		// Mettre à jour les échos (vieillissement)
		// updateEchos();
		updatePositionEchos(angle);

		// Générer de nouveaux échos aléatoires
		maybeCreateEcho(angle);

		// Dessiner les échos
		//drawEchos(lastAngle, angle);
		lastAngle = angle;
		// Update last update time
		lastupdate_ms = cur_ms;
	}
}

// Dessine le fond du radar (cercles + lignes)
void Dashboard::drawRadarBackground()
{
	_tft.fillScreen(RADAR_BG);

	// cercles de portée
	_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, TFT_DARKGREEN);
	_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R * 2 / 3, TFT_DARKGREEN);
	_tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R / 3, TFT_DARKGREEN);

	// croix centrale
	_tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, TFT_DARKGREEN);
	_tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, TFT_DARKGREEN);

	// texte
	_tft.setTextColor(TFT_GREEN, RADAR_BG);
	_tft.setTextDatum(MC_DATUM);
	_tft.drawString("RADAR", RADAR_CX, RADAR_CY - RADAR_R - 10);
}

// Dessine ou efface la ligne de balayage
void Dashboard::drawSweep(float deg, uint16_t color, bool erase)
{
	float rad = deg * 3.14159265 / 180.0;
	int x = RADAR_CX + RADAR_R * cos(rad);
	int y = RADAR_CY + RADAR_R * sin(rad);

	// si on efface, on peut utiliser une couleur plus sombre pour un effet “trainée”
	uint16_t c = erase ? RADAR_BG : color;
	_tft.drawLine(RADAR_CX, RADAR_CY, x, y, c);
}

// Met à jour la durée de vie des échos
void Dashboard::updatePositionEchos(float angle)
{
	int px, py;
	float alpha, dist;
	// uint32_t now = millis();
	for (int i = 0; i < MAX_ECHOS; i++)
	{
		if (echos[i].active && (echos[i].angle < normalizeAngle(angle + angleStep) && echos[i].angle > normalizeAngle(angle - angleStep)))
		//if (echos[i].active)
		{
			// Clear old position
			clearEcho(i);
			// Faire bouger les échos légèrement pour simuler un mouvement
			px = echos[i].pos_x + echos[i].offset_x;
			py = echos[i].pos_y + echos[i].offset_y;
			dist = sqrt((px - RADAR_CX) * (px - RADAR_CX) + (py - RADAR_CY) * (py - RADAR_CY));
			if (px < 0 || px >= SCREEN_WIDTH || py < 0 || py >= SCREEN_HEIGHT)	// dist > 120)
			{
				echos[i].offset_x = -echos[i].offset_x;
				echos[i].offset_y = -echos[i].offset_y;
				// echos[i].active = false;
			}
			else
			{
				echos[i].pos_x = px;
				echos[i].pos_y = py;
				// Recalcul de l'angle
				alpha = atan2(echos[i].pos_y - RADAR_CY, echos[i].pos_x - RADAR_CX);
				echos[i].angle = normalizeAngle((alpha * 180.0 / 3.14));	// + 180.0);
				// Update Position
				_tft.fillCircle(echos[i].pos_x, echos[i].pos_y, 3, echos[i].color);
			}
		}
	}
}
// Met à jour la durée de vie des échos
void Dashboard::updateEchos()
{
	uint32_t now = millis();
	for (int i = 0; i < MAX_ECHOS; i++)
	{
		if (echos[i].active && now > echos[i].ttl)
		{
			echos[i].active = false;
			clearEcho(i);
		}
	}
}

// Crée parfois un nouvel écho sur la ligne de balayage
void Dashboard::maybeCreateEcho(float currentAngle)
{
	// probabilité d’apparition
	if (random(0, 100) < 10) // 10% à chaque pas
	{
		for (int i = 0; i < MAX_ECHOS; i++)
		{
			if (!echos[i].active)
			{
				echos[i].active = true;
				echos[i].angle = currentAngle + random(-2, 2); // léger décalage
				if (echos[i].angle < 0)
					echos[i].angle += 360;
				if (echos[i].angle >= 360)
					echos[i].angle -= 360;
				echos[i].distance = random(RADAR_R / 4, RADAR_R); // distance aléatoire

				// Convert to position
				float rad = echos[i].angle * 3.14159265 / 180.0;
				echos[i].pos_x = RADAR_CX + echos[i].distance * cos(rad);
				echos[i].pos_y = RADAR_CY + echos[i].distance * sin(rad);
				// faire bouger les échos légèrement pour simuler un mouvement
				while (echos[i].offset_x == 0 && echos[i].offset_y == 0)
				{
					echos[i].offset_x = random(-3, 3); // -1, 0 ou +1
					echos[i].offset_y = random(-3, 3); // -1, 0 ou +1
				}
				// Color
				echos[i].color = TFT_GREENYELLOW;
				// echos[i].ttl = millis() + random(500, 2000); // durée de vie 0,5 à 2 s
				echos[i].ttl = millis() + random(20000, 100000); // durée de vie 2s a 10s
				//
				_tft.fillCircle(echos[i].pos_x, echos[i].pos_y, 3, echos[i].color);
				break;
			}
		}
	}
}

// Dessine les échos actifs
void Dashboard::drawEchos(float oldAngle, float newAngle)
{
	for (int i = 0; i < MAX_ECHOS; i++)
	{
		if (echos[i].active)
		{
			// float rad = echos[i].angle * 3.14159265 / 180.0;
			// int x = RADAR_CX + echos[i].distance * cos(rad);
			// int y = RADAR_CY + echos[i].distance * sin(rad);

			// petit point ou petit cercle
			//_tft.fillCircle(x, y, 3, echos[i].color);
			_tft.fillCircle(echos[i].pos_x, echos[i].pos_y, 3, echos[i].color);
		}
	}
}

/*
void Dashboard::drawEchos() {
  for (int i = 0; i < MAX_ECHOS; i++) {
	if (echos[i].active) {
	  float rad = echos[i].angle * 3.14159265 / 180.0;
	  int x = RADAR_CX + echos[i].distance * cos(rad);
	  int y = RADAR_CY + echos[i].distance * sin(rad);

	  // petit point ou petit cercle
	  _tft.fillCircle(x, y, 3, echos[i].color);
	}
  }
}
*/

// Dessine les échos actifs
void Dashboard::clearEcho(int i)
{
	/*
	float rad = echos[i].angle * 3.14159265 / 180.0;
	int x = RADAR_CX + echos[i].distance * cos(rad);
	int y = RADAR_CY + echos[i].distance * sin(rad);

	// petit point ou petit cercle
	_tft.fillCircle(x, y, 3, RADAR_BG); // echos[i].color);
	*/
	_tft.fillCircle(echos[i].pos_x, echos[i].pos_y, 3, RADAR_BG); // echos[i].color);
}
#endif

/********************************************************************************/
/*																				*/
/********************************************************************************/
#if VERSION == 1
/// @brief Update the dashboard display with new values.
/// @param value
void Dashboard::update(uint32_t cur_ms)
{
	if (cur_ms > lastupdate_ms + 50)
	{
		// Effacer l’ancienne ligne de balayage
		eraseSweepLine(lastAngle);

		// Mettre à jour l’angle du balayage
		angle += angleStep;
		if (angle >= 360.0)
			angle -= 360.0;

		// Dessiner la nouvelle ligne de balayage
		drawSweepLine(angle);

		// Dessiner les échos visibles dans le secteur du balayage
		drawEchos(angle);

		lastAngle = angle;

		// Update last update time
		lastupdate_ms = cur_ms;
	}
}

void Dashboard::setupEchos()
{
	for (uint8_t i = 0; i < MAX_ECHOS; i++)
	{
		echos[i].angle = random(0, 360);
		echos[i].distance = random(10, RADAR_R);
	}
}

void Dashboard::drawGrid()
{
	_tft.fillScreen(RADAR_BG);

	// Cercles de portée
	for (int r = 30; r <= RADAR_R; r += 30)
	{
		_tft.drawCircle(RADAR_CX, RADAR_CY, r, RADAR_GRID);
	}

	// Axes principaux
	_tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, RADAR_GRID);
	_tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, RADAR_GRID);

	// Quelques axes supplémentaires (tous les 30°)
	for (int a = 0; a < 360; a += 30)
	{
		float rad = a * DEG_TO_RAD;
		int16_t x2 = RADAR_CX + cos(rad) * RADAR_R;
		int16_t y2 = RADAR_CY + sin(rad) * RADAR_R;
		_tft.drawLine(RADAR_CX, RADAR_CY, x2, y2, RADAR_GRID);
	}
}

void Dashboard::eraseSweepLine(float aDeg)
{
	float rad = aDeg * DEG_TO_RAD;
	int16_t x2 = RADAR_CX + cos(rad) * RADAR_R;
	int16_t y2 = RADAR_CY + sin(rad) * RADAR_R;
	// On “efface” en redessinant la ligne dans la couleur du fond
	_tft.drawLine(RADAR_CX, RADAR_CY, x2, y2, RADAR_BG);

	// Optionnel : redessiner la grille sous la ligne effacée
	// (simple mais pas parfait : on redessine toute la grille)
	if ((uint16_t)(aDeg) % 30 < angleStep)
	{ // Si on est proche d’un axe de la grille, redessiner la grille
		drawGrid();
	}
	else
	{
		// Cercles de portée
		for (int r = 30; r <= RADAR_R; r += 30)
		{
			_tft.drawCircle(RADAR_CX, RADAR_CY, r, RADAR_GRID);
		}

		// Axes principaux
		_tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, RADAR_GRID);
		_tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, RADAR_GRID);

		// Quelques axes supplémentaires (tous les 30°)
		for (int a = 0; a < 360; a += 30)
		{
			float rad = a * DEG_TO_RAD;
			int16_t x2 = RADAR_CX + cos(rad) * RADAR_R;
			int16_t y2 = RADAR_CY + sin(rad) * RADAR_R;
			_tft.drawLine(RADAR_CX, RADAR_CY, x2, y2, RADAR_GRID);
		}
	}
}

void Dashboard::drawSweepLine(float aDeg)
{
	float rad = aDeg * DEG_TO_RAD;
	int16_t x2 = RADAR_CX + cos(rad) * RADAR_R;
	int16_t y2 = RADAR_CY + sin(rad) * RADAR_R;
	_tft.drawLine(RADAR_CX, RADAR_CY, x2, y2, RADAR_LINE);
}

void Dashboard::drawEchos(float sweepAngleDeg)
{
	// Largeur angulaire du secteur “actif” (où les échos sont visibles)
	const float sectorWidth = 90.0;

	for (uint8_t i = 0; i < MAX_ECHOS; i++)
	{
		float da = fabsf(normalizeAngle(echos[i].angle - sweepAngleDeg));
		if (da < sectorWidth)
		{
			// Echo dans le secteur du balayage
			float rad = echos[i].angle * DEG_TO_RAD;
			int16_t x = RADAR_CX + cos(rad) * echos[i].distance;
			int16_t y = RADAR_CY + sin(rad) * echos[i].distance;
			_tft.fillCircle(x, y, 2, RADAR_ECHO);
		}
		else
		{
			// Optionnel : faire “disparaître” progressivement les anciens échos
			// Ici on ne les efface pas pour garder un effet de persistance
		}
	}
}

#endif

// Normalise un angle en degrés dans [0, 360)
float Dashboard::normalizeAngle(float a)
{
	while (a < 0)
		a += 360.0;
	while (a >= 360.0)
		a -= 360.0;
	return round(a);
}

/*
void Dashboard::setLeftIndicator(boolean on)
{
	drawLeftIndicator(on ? TFT_DARKGREEN : offColor);
}

void Dashboard::setRightIndicator(boolean on)
{
	drawRightIndicator(on ? TFT_DARKGREEN : offColor);
}

void Dashboard::setSpeed(uint16_t value) { drawSpeed(value); }

void Dashboard::setRPM(uint16_t value) { drawRPM(value); }

void Dashboard::setFuelLevel(uint16_t value) { drawFuel(value); }

void Dashboard::setAdBlueLevel(uint16_t value) { drawAdBlue(value); }

void Dashboard::setLowBeamIndicator(boolean on)
{
	drawLowBeamIndicator(on ? TFT_GREEN : offColor);
}

void Dashboard::setHighBeamIndicator(boolean on)
{
	drawHighBeamIndicator(on ? TFT_BLUE : offColor);
}

void Dashboard::setFogLightIndicator(boolean on)
{
	drawFogLightIndicator(on ? TFT_GREEN : offColor);
}

void Dashboard::setGear(int8_t value)
{
	char symbol = ' ';
	if (value == -1)
	{
		symbol = 'R';
	}
	else if (value == 0)
	{
		symbol = 'N';
	}
	else if (value > 0 && value < 10)
	{
		symbol = '0' + value;
	}
	drawGear(symbol);
}

void Dashboard::setVolt(float value, float threshold)
{
	drawVolt(value, threshold);
}

void Dashboard::drawFrame()
{
	unsigned short color1;
	unsigned short color2;

	// Central display ----
	for (int i = 0; i <= 3; i++)
	{
		_tft.fillRect(SCREEN_WIDTH / 4, 15 + (i * 12), (SCREEN_WIDTH / 4) * 2, 10, 0x0290);
	}

	_tft.fillRect(SCREEN_WIDTH / 2 - 10, 17, 4, 4, TFT_DARKGREY);
	_tft.fillRect(SCREEN_WIDTH / 2 - 5, 17, 4, 4, TFT_DARKGREY);
	_tft.fillRect(SCREEN_WIDTH / 2, 17, 4, 4, TFT_DARKGREY);
	_tft.fillRect(SCREEN_WIDTH / 2 + 5, 17, 4, 4, TFT_DARKGREY);

	drawPump(SCREEN_WIDTH / 2 - 3, 53, TFT_WHITE);
	drawPump(SCREEN_WIDTH / 2 - 3, 29, TFT_SKYBLUE);

	_tft.fillRect(SCREEN_WIDTH / 2 - 2, 42, 4, 4, TFT_DARKGREY);
	// Punto Rojo
	_tft.fillCircle(SCREEN_WIDTH - 5, 5, 2, TFT_RED);
	for (int i = 0; i <= 2; i++)
	{
		_tft.drawLine(3 + (i * 3), 3, 3 + (i * 3), 8, TFT_ORANGE);
	}

	_tft.fillCircle(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS + 2, TFT_BLACK);
	_tft.fillCircle(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS + 2, TFT_BLACK);

	// SPEED FREDY ----
	// Speed gauge outer ring segments
	drawArc(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS,
			SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END, 0x0290); // TFT_LIGHTGREY);
	drawArc(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS - 1,
			SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END, 0x0290); // TFT_LIGHTGREY);
	drawArc(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS - 2,
			SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END, 0x0290); // TFT_LIGHTGREY

	drawArc(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS - 4,
			SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END, TFT_WHITE); // TFT_LIGHTGREY);

	// Speed scale segments
	for (int16_t i = 0; i <= SPEED_CIRCLE_SEGMENTS; i++)
	{
		if (i < 9)
		{
			color1 = gaugeColor;
			color2 = TFT_WHITE;
		}
		else
		{
			color1 = purple;
			color2 = purple;
		}
		if (i % 2 == 0)
		{
			drawSep(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, 3,
					SPEED_CIRCLE_ANGLE_START + i * SPEED_CIRCLE_SEGMENT_ANGLE, SPEED_CIRCLE_RADIUS - 5,
					color1);
		}
		else
		{
			drawSep(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, 3,
					SPEED_CIRCLE_ANGLE_START + i * SPEED_CIRCLE_SEGMENT_ANGLE, SPEED_CIRCLE_RADIUS - 5,
					color2);
		}
	}

	// RPM Fredy ----
	// RPM gauge outer ring segments
	drawArc(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS,
			RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END, 0x0290);
	drawArc(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS - 1,
			RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END, 0x0290);
	drawArc(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS - 2,
			RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END, 0x0290);

	drawArc(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS - 4,
			RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END, TFT_WHITE);

	// RPM scale segments
	for (int16_t i = 0; i <= RPM_CIRCLE_SEGMENTS; i++)
	{
		color1 = gaugeColor;
		color2 = TFT_WHITE;
		if (i % 2 == 0)
		{
			drawSep(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, 3,
					SPEED_CIRCLE_ANGLE_START + i * RPM_CIRCLE_SEGMENT_ANGLE, RPM_CIRCLE_RADIUS - 5,
					color1);
		}
		else
		{
			drawSep(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, 3,
					RPM_CIRCLE_ANGLE_START + i * RPM_CIRCLE_SEGMENT_ANGLE, RPM_CIRCLE_RADIUS - 5,
					color2);
		}
	}

	drawNumVel();
	drawNumrpm();

	_tft.fillCircle(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, 14, 0x0290);
	_tft.fillCircle(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, 12, TFT_BLACK);

	_tft.fillCircle(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, 14, 0x0290);
	_tft.fillCircle(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, 12, TFT_BLACK);

	_tft.setTextColor(TFT_ORANGE);
	_tft.drawCentreString("rpm", RPM_CIRCLE_CENTER_X, 60, 1);
	_tft.drawCentreString("x100", RPM_CIRCLE_CENTER_X, 70, 1);
	_tft.drawCentreString("km/h", SPEED_CIRCLE_CENTER_X - 3, 60, 1);

	drawGear('N');
}

void Dashboard::drawNumVel()
{
	unsigned short color2;
	double rad = 0.01745;
	for (int16_t i = 0; i <= SPEED_CIRCLE_SEGMENTS; i++)
	{
		if (i < 9)
		{
			color2 = TFT_WHITE;
		}
		else
		{
			color2 = purple;
		}
		if (i % 2 == 0)
		{
			_tft.setTextColor(color2);
			int16_t a = SPEED_CIRCLE_ANGLE_START + i * SPEED_CIRCLE_SEGMENT_ANGLE;
			int16_t mx = ((SPEED_CIRCLE_RADIUS - 12) * cos(rad * a)) + SPEED_CIRCLE_CENTER_X;
			int16_t my = ((SPEED_CIRCLE_RADIUS - 12) * sin(rad * a)) + SPEED_CIRCLE_CENTER_Y;
			_tft.drawCentreString(String(i * 10), mx, my, 1);
		}
	}
}

void Dashboard::drawNumrpm()
{
	unsigned short color2 = TFT_WHITE;
	double rad = 0.01745;
	for (int16_t i = 0; i <= RPM_CIRCLE_SEGMENTS; i++)
	{
		if (i % 2 == 0)
		{
			_tft.setTextColor(color2);
			int16_t a = RPM_CIRCLE_ANGLE_START + i * RPM_CIRCLE_SEGMENT_ANGLE;
			int16_t mx = ((RPM_CIRCLE_RADIUS - 12) * cos(rad * a)) + RPM_CIRCLE_CENTER_X;
			int16_t my = ((RPM_CIRCLE_RADIUS - 12) * sin(rad * a)) + RPM_CIRCLE_CENTER_Y;
			_tft.drawCentreString(String(i * 5 / 2), mx, my, 1);
		}
	}
}

void Dashboard::drawArc(int16_t cx, int16_t cy, int16_t r, int16_t deg_start, int16_t deg_end,
						uint32_t color)
{
	float sx, sy;
	uint16_t x, y;

	for (int16_t i = deg_start; i <= deg_end; i++)
	{
		sx = cos(i * DEG2RAD);
		sy = sin(i * DEG2RAD);
		x = round(sx * r) + cx;
		y = round(sy * r) + cy;
		_tft.drawPixel(x, y, color);
	}
}

void Dashboard::drawSep(int16_t cx, int16_t cy, uint16_t len, int16_t deg_pos, uint16_t r,
						uint32_t color)
{
	float sx, sy;
	uint16_t x0, y0, x1, y1;

	for (int8_t i = -1; i <= 1; i++)
	{
		sx = cos((i + deg_pos) * DEG2RAD);
		sy = sin((i + deg_pos) * DEG2RAD);
		x0 = round(sx * r) + cx;
		y0 = round(sy * r) + cy;
		x1 = round(sx * (r - len)) + cx;
		y1 = round(sy * (r - len)) + cy;
		_tft.drawLine(x0, y0, x1, y1, color);
	}
}

void Dashboard::drawNeedle(int16_t cx, int16_t cy, uint16_t len, int16_t deg_pos,
						   uint16_t radius_holder, uint32_t color_needle)
{
	float sx, sy;
	uint16_t x, y, xx, yy;
	sx = cos((deg_pos)*DEG2RAD);
	sy = sin((deg_pos)*DEG2RAD);
	x = round(sx * len) + cx;
	y = round(sy * len) + cy;
	xx = -round(sx * 2) + cx;
	yy = -round(sy * 2) + cy;
	_tft.drawLine(x, y, xx, yy, color_needle);
}

// Needle function
void Dashboard::drawIndicadorFr(int16_t cx, int16_t cy, uint16_t len, int16_t deg_pos,
								uint16_t radius_holder, uint32_t color_needle,
								int16_t deg_pos_ant, uint32_t color_needle_ant, uint16_t pVelocidad)
{

	if (deg_pos == deg_pos_ant)
	{
		return;
	}
	float sx, sy;
	uint16_t x, y, xx, yy;

	// clear old needle
	sx = cos(deg_pos_ant * DEG2RAD);
	sy = sin(deg_pos_ant * DEG2RAD);
	x = round(sx * len) + cx;
	y = round(sy * len) + cy;

	xx = round(sx * (radius_holder - 2)) + cx;
	yy = round(sy * (radius_holder - 2)) + cy;

	_tft.drawLine(x, y, xx, yy, color_needle_ant);

	if (pVelocidad == 1)
	{
		drawNumVel();
	}
	if (pVelocidad == 2)
	{
		drawNumrpm();
	}

	// draw new needle
	sx = cos(deg_pos * DEG2RAD);
	sy = sin(deg_pos * DEG2RAD);
	x = round(sx * len) + cx;
	y = round(sy * len) + cy;

	xx = round(sx * (radius_holder - 2)) + cx;
	yy = round(sy * (radius_holder - 2)) + cy;

	_tft.drawLine(x, y, xx, yy, color_needle);
}

void Dashboard::drawLeftIndicator(uint32_t color)
{
	static uint32_t lastVal = TFT_BLACK;
	if (color == lastVal)
	{
		return;
	}
	lastVal = color;

	_tft.fillTriangle(INDICATOR_LEFT_CENTER_X - 4, INDICATOR_LEFT_CENTER_Y, INDICATOR_LEFT_CENTER_X,
					  INDICATOR_LEFT_CENTER_Y - 4, INDICATOR_LEFT_CENTER_X,
					  INDICATOR_LEFT_CENTER_Y + 4, color);
	_tft.fillRect(INDICATOR_LEFT_CENTER_X, INDICATOR_LEFT_CENTER_Y - 2, 4, 5, color);
}

void Dashboard::drawRightIndicator(uint32_t color)
{
	static uint32_t lastVal = TFT_BLACK;
	if (color == lastVal)
	{
		return;
	}
	lastVal = color;

	_tft.fillTriangle(INDICATOR_RIGHT_CENTER_X + 4, INDICATOR_RIGHT_CENTER_Y,
					  INDICATOR_RIGHT_CENTER_X, INDICATOR_RIGHT_CENTER_Y - 4,
					  INDICATOR_RIGHT_CENTER_X, INDICATOR_RIGHT_CENTER_Y + 4, color);
	_tft.fillRect(INDICATOR_RIGHT_CENTER_X - 4, INDICATOR_RIGHT_CENTER_Y - 2, 4, 5, color);
}

void Dashboard::drawLowBeamIndicator(uint32_t color)
{
	static uint32_t lastVal = TFT_BLACK;
	if (color == lastVal)
	{
		return;
	}
	lastVal = color;

	uint16_t x = SCREEN_WIDTH / 2 + 10;
	// uint16_t y = SCREEN_HEIGHT - 20;
	// Fredy
	uint16_t y = INSTRUMENTOS_Y;

	uint8_t cols = sizeof(LOW_BEAM_ICON[0]);
	uint8_t rows = sizeof(LOW_BEAM_ICON) / cols;
	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
		{
			if (LOW_BEAM_ICON[i][j])
			{
				_tft.drawPixel(x + j, y + i, color);
			}
		}
	}
}

void Dashboard::drawHighBeamIndicator(uint32_t color)
{
	static uint32_t lastVal = TFT_BLACK;
	if (color == lastVal)
	{
		return;
	}
	lastVal = color;

	uint16_t x = SCREEN_WIDTH / 2 - 7;
	// uint16_t y = SCREEN_HEIGHT - 20;
	// Fredy
	uint16_t y = INSTRUMENTOS_Y;

	uint8_t cols = sizeof(HIGH_BEAM_ICON[0]);
	uint8_t rows = sizeof(HIGH_BEAM_ICON) / cols;
	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
		{
			if (HIGH_BEAM_ICON[i][j])
			{
				_tft.drawPixel(x + j, y + i, color);
			}
		}
	}
}

void Dashboard::drawFogLightIndicator(uint32_t color)
{
	static uint32_t lastVal = TFT_BLACK;
	if (color == lastVal)
	{
		return;
	}
	lastVal = color;

	uint16_t x = SCREEN_WIDTH / 2 - 24;
	// uint16_t y = SCREEN_HEIGHT - 20;
	// Fredy
	uint16_t y = INSTRUMENTOS_Y;

	uint8_t cols = sizeof(FOG_LIGHT_ICON[0]);
	uint8_t rows = sizeof(FOG_LIGHT_ICON) / cols;
	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
		{
			if (FOG_LIGHT_ICON[i][j])
			{
				_tft.drawPixel(x + j, y + i, color);
			}
		}
	}
}

void Dashboard::drawSpeed(uint16_t value)
{
	static uint16_t lastVal = 0xFFFF;
	if (value == lastVal)
	{
		return;
	}

	int16_t last_deg =
		map(lastVal, SPEED_MIN, SPEED_MAX, SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END);
	int16_t deg = map(value, SPEED_MIN, SPEED_MAX, SPEED_CIRCLE_ANGLE_START, SPEED_CIRCLE_ANGLE_END);
	// Speed needle
	drawIndicadorFr(SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y, SPEED_CIRCLE_RADIUS - 9, deg, 16,
					needleColor, last_deg, TFT_BLACK, 1);

	_tft.setTextColor(TFT_BLACK);
	_tft.drawCentreString(String(lastVal), SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y - 4, 1);

	_tft.setTextColor(TFT_WHITE);
	_tft.drawCentreString(String(value), SPEED_CIRCLE_CENTER_X, SPEED_CIRCLE_CENTER_Y - 4, 1);

	lastVal = value;
}

void Dashboard::drawRPM(uint16_t value)
{
	static uint16_t lastVal = 0xFFFF;
	if (value == lastVal)
	{
		return;
	}

	int16_t last_deg = map(lastVal, RPM_MIN, RPM_MAX, RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END);
	int16_t deg = map(value, RPM_MIN, RPM_MAX, RPM_CIRCLE_ANGLE_START, RPM_CIRCLE_ANGLE_END);
	// RPM needle
	drawIndicadorFr(RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y, RPM_CIRCLE_RADIUS - 9, deg, 16,
					needleColor, last_deg, TFT_BLACK, 2);

	lastVal = value;
}

void Dashboard::drawFuel(uint16_t value)
{
	return;

	static uint16_t lastVal = 0xFFFF;
	if (value == lastVal)
	{
		return;
	}

	int16_t last_deg =
		map(lastVal, FUEL_MIN, FUEL_MAX, FUEL_CIRCLE_ANGLE_START, FUEL_CIRCLE_ANGLE_END);
	int16_t deg = map(value, FUEL_MIN, FUEL_MAX, FUEL_CIRCLE_ANGLE_START, FUEL_CIRCLE_ANGLE_END);
	drawNeedle(FUEL_CIRCLE_CENTER_X, FUEL_CIRCLE_CENTER_Y, FUEL_CIRCLE_RADIUS - 4, last_deg, 2,
			   TFT_BLACK);
	drawNeedle(FUEL_CIRCLE_CENTER_X, FUEL_CIRCLE_CENTER_Y, FUEL_CIRCLE_RADIUS - 4, deg, 2, TFT_WHITE);

	lastVal = value;
}

void Dashboard::drawAdBlue(uint16_t value)
{
	return;
	static uint16_t lastVal = 0xFFFF;
	if (value == lastVal)
	{
		return;
	}

	int16_t last_deg =
		map(lastVal, ADBLUE_MIN, ADBLUE_MAX, ADBLUE_CIRCLE_ANGLE_START, ADBLUE_CIRCLE_ANGLE_END);
	int16_t deg =
		map(value, ADBLUE_MIN, ADBLUE_MAX, ADBLUE_CIRCLE_ANGLE_START, ADBLUE_CIRCLE_ANGLE_END);
	drawNeedle(ADBLUE_CIRCLE_CENTER_X, ADBLUE_CIRCLE_CENTER_Y, ADBLUE_CIRCLE_RADIUS - 4, last_deg, 2,
			   TFT_BLACK);
	drawNeedle(ADBLUE_CIRCLE_CENTER_X, ADBLUE_CIRCLE_CENTER_Y, ADBLUE_CIRCLE_RADIUS - 4, deg, 2,
			   TFT_WHITE);

	lastVal = value;
}

void Dashboard::drawPump(uint16_t x, uint16_t y, uint32_t color)
{
	uint8_t cols = sizeof(PUMP_ICON[0]);
	uint8_t rows = sizeof(PUMP_ICON) / cols;
	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
		{
			if (PUMP_ICON[i][j])
			{
				_tft.drawPixel(x + j, y + i, color);
			}
		}
	}
}

void Dashboard::drawGear(char gear)
{
	static char lastVal = ' ';
	if (gear == lastVal)
	{
		return;
	}
	char str[2] = "\0";
	str[0] = gear;

	_tft.setTextColor(TFT_BLACK);
	_tft.drawCentreString(String((lastVal)), RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y - 7, 2);

	_tft.setTextColor(TFT_WHITE);
	_tft.drawCentreString(str, RPM_CIRCLE_CENTER_X, RPM_CIRCLE_CENTER_Y - 7, 2);
	lastVal = gear;
}

void Dashboard::drawVolt(float volt, float threshold)
{
	static float lastVal = '1';
	if (abs(volt - lastVal) < 0.1)
		return; // Prevent display from being triggered too frequently

	_tft.setTextColor(TFT_BLACK);
	_tft.drawCentreString(String(lastVal) + "V", 33, 70, 1); // Clear old content

	// Set color
	if (volt <= threshold)
		_tft.setTextColor(TFT_RED);
	else if (volt <= threshold + 0.5)
		_tft.setTextColor(TFT_ORANGE);
	else
		_tft.setTextColor(TFT_WHITE);

	_tft.drawCentreString(String(volt) + "V", 33, 70, 1); // Show new content

	lastVal = volt;
}

// Draw Frevic logo
void Dashboard::drawLogo(int16_t x, int16_t y, const uint16_t *bitmap, int16_t w, int16_t h)
{
	int row, col, buffidx = 0;
	for (row = x; row < h; row++)
	{ // For each scanline...
		for (col = y; col < w; col++)
		{ // For each pixel...
			_tft.drawPixel(col, row, pgm_read_word(bitmap + buffidx));
			buffidx++;
		}
	}
}

*/

#endif
// End of Vrevic dashboard