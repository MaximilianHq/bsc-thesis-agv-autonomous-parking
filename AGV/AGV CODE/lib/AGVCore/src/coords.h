#pragma once
#include <types.h>
#include <Arduino.h>

void agv2dwm(AgvState &state, const Position &PosRelAgv){// Transformerar en punkt relativt AGV till DWM/världskoordinater
state.pos.x = PosRelAgv.x*cos(state.theta) + PosRelAgv.y*sin(state.theta) + state.pos.x;
state.pos.y = - PosRelAgv.x*sin(state.theta) + PosRelAgv.y*cos(state.theta) + state.pos.y;
}


void dwm2agv(AgvState &state){ // Ger dwm position i agv koordinater, dvs tar hänsyn till dwm:s position på agv och agv:s riktning
float offset = 50; // mm, avståndet från dwm till agv:s mittpunkt
state.pos.x = state.pos.x - offset*cos(state.theta);
state.pos.y = state.pos.y - offset*sin(state.theta);

}