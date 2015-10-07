#include "WaterPump.h"


WaterPump::WaterPump()
{
	/* Choose channels*/
	/* Use ch20 for mux input 1 by default */
	muxIn1 = new GPIOClass("20");
	/* Use ch21 for mux input 2 by default */
	muxIn2 = new GPIOClass("21");

	/* Export them*/
	muxIn1->export_gpio();
	muxIn2->export_gpio();

	/* Set both channels as output */
	muxIn1->setdir_gpio("out");
	muxIn2->setdir_gpio("out");
}

WaterPump::WaterPump(string rpiCh_muxIn1, string rpiCh_muxIn2)
{
	/* Choose channels*/
	/* Use raspberry pi channel for mux input 1 */
	muxIn1 = new GPIOClass("rpiCh_muxIn1");
	/* Use raspberry pi channel for mux input 2 */
	muxIn2 = new GPIOClass("rpiCh_muxIn2");

	/* Export them */
	muxIn1->export_gpio();
	muxIn2->export_gpio();

	/* Set both channels as output */
	muxIn1->setdir_gpio("out");
	muxIn2->setdir_gpio("out");
}

/* Set the pump in forward mode */
void WaterPump::forward()
{
	muxIn1->setval_gpio("1");
	muxIn2->setval_gpio("0");
};
/* Set the pump in reverse mode */
void WaterPump::back()
{
	muxIn1->setval_gpio("0");
	muxIn2->setval_gpio("1");
};
/* Set the pump in stop mode */
void WaterPump::stop()
{
	muxIn1->setval_gpio("0");
	muxIn2->setval_gpio("0");
};
/* Set the pump in break mode */
void WaterPump::brake()
{
	muxIn1->setval_gpio("1");
	muxIn2->setval_gpio("1");
};
