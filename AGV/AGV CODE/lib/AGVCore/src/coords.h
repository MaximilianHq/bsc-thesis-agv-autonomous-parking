#pragma once
#include <Arduino.h>
#include <types.h>

// REP 103: body frame uses x forward, y left, z up.
// Yaw is measured from +x and grows counter-clockwise.
void rel2agv(AgvState &state, const Position &PosRelAgv)
{ // Transformerar en punkt i AGV-kroppsramen till världskoordinater
    state.pos.x = PosRelAgv.x * cosf(state.theta) - PosRelAgv.y * sinf(state.theta) + state.pos.x;
    state.pos.y = PosRelAgv.x * sinf(state.theta) + PosRelAgv.y * cosf(state.theta) + state.pos.y;
}

void abs2agv(AgvState &state, float offset)
{ // Flyttar DWM-tag position till AGV-center med offset längs AGV:s +x-axel.
    state.pos.x = state.pos.x - offset * cosf(state.theta);
    state.pos.y = state.pos.y - offset * sinf(state.theta);
}

void abs2agv(Position &p, float ang, float offset)
{ // Flyttar DWM-tag position till AGV-center med offset längs AGV:s +x-axel.
    p.x = p.x - offset * cosf(ang);
    p.y = p.y - offset * sinf(ang);
}
