#ifndef WATERPUMP_H
#define WATERPUMP_H

#include "GPIOClass.h"
using namespace std;

class WaterPump
{
public:
	WaterPump();
	WaterPump(string mux_1, string mux_2);
	void forward();
	void back();
	void stop();
	void brake();
private:
	GPIOClass* muxIn1;
	GPIOClass* muxIn2;
};

#endif