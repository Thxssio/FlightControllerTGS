

#ifndef __PULSE_POSITION_IMXRT_H__
#define __PULSE_POSITION_IMXRT_H__
 
#if defined(__IMXRT1062__)

#include <Arduino.h>

#define FlightControllerTGS_MAXCHANNELS 16

class FlightControllerTGSBase
{
public:

protected:
	static FlightControllerTGSBase *list[10];
	virtual void isr() = 0;

	typedef struct {
		uint8_t 		pin;
		uint8_t			channel;
		volatile IMXRT_TMR_t* tmr;
		volatile uint32_t *clock_gate_register;
		uint32_t 		clock_gate_mask;
		IRQ_NUMBER_t	interrupt;
		void     		(*isr)();
		volatile uint32_t	*select_input_register; // Which register controls the selection
		const uint32_t		select_val;	// Value for that selection
	} TMR_Hardware_t;

	static const TMR_Hardware_t hardware[];
	static const uint8_t _hardware_count;

	// static class functions

	static void isrTimer1();
	static void isrTimer2();
	static void isrTimer3();
	static void isrTimer4();
	static inline void checkAndProcessTimerCHInPending(uint8_t index, volatile IMXRT_TMR_CH_t *tmr_ch);
};


class FlightControllerTGSOutput : public FlightControllerTGSBase
{
public:
	FlightControllerTGSOutput(void);
	FlightControllerTGSOutput(int polarity);
	bool begin(uint8_t txPin); // txPin can be 6,9,10,11,12,13,14,15,18,19
	bool begin(uint8_t txPin, uint32_t _framePin);
	bool write(uint8_t channel, float microseconds);

private:
	uint8_t outPolarity = 1;  // Polarity rising
	uint8_t inPolarity = 1;
	
	volatile uint8_t framePinMask;
	uint32_t state, total_channels, total_channels_buffer, pulse_remaining,
			 current_channel, framePin = 255;
	volatile uint32_t ticks;
	
	uint32_t pulse_width[FlightControllerTGS_MAXCHANNELS + 1];
	uint32_t pulse_buffer[FlightControllerTGS_MAXCHANNELS + 1];

	// member variables...
	uint16_t idx_channel;
	virtual void isr();
};


class FlightControllerTGSInput : public FlightControllerTGSBase
{
public:
	FlightControllerTGSInput(void);
	FlightControllerTGSInput(int polarity);
	bool begin(uint8_t rxPin); // rxPin can be 6,9,10,11,12,13,14,15,18,19
	int available(void);
	float read(uint8_t channel);
	
private:
	uint32_t pulse_width[FlightControllerTGS_MAXCHANNELS+1];
	uint32_t pulse_buffer[FlightControllerTGS_MAXCHANNELS+1];
	uint32_t prev;
	uint8_t write_index;
	uint8_t total_channels;
	uint8_t outPolarity = 1;  // Polarity rising
	uint8_t inPolarity = 1;
	volatile uint32_t ticks, overflow_count;
	volatile bool overflow_inc, available_flag;
	static uint8_t channelmask;
	// member variables...
	uint16_t idx_channel;
	virtual void isr();
};

#endif
#endif