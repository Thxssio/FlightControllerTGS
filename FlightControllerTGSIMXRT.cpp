

#if defined(__IMXRT1062__)

#include "Arduino.h"
#include "FlightControllerTGSIMXRT.h"


#ifdef DEBUG_IO_PINS
#define DBGdigitalWriteFast(pin, state) digitalWriteFast(pin, state);
#else
    inline void  DBGdigitalWriteFast(uint8_t pin, uint8_t state) {};
#endif


#define TX_MINIMUM_SIGNAL   300.0


#define TX_MAXIMUM_SIGNAL  2500.0


#define TX_DEFAULT_SIGNAL  1500.0

#define TX_MINIMUM_SPACE   5000.0


#define TX_MINIMUM_FRAME  20000.0

#define TX_PULSE_WIDTH      100.0

#define RX_MINIMUM_SPACE   3500.0

#define CLOCKS_PER_MICROSECOND (150./4) 
#define TX_MINIMUM_SPACE_CLOCKS   (uint32_t)(TX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)
#define TX_MINIMUM_FRAME_CLOCKS   (uint32_t)(TX_MINIMUM_FRAME * CLOCKS_PER_MICROSECOND)
#define TX_PULSE_WIDTH_CLOCKS     (uint32_t)(TX_PULSE_WIDTH * CLOCKS_PER_MICROSECOND)
#define TX_DEFAULT_SIGNAL_CLOCKS  (uint32_t)(TX_DEFAULT_SIGNAL * CLOCKS_PER_MICROSECOND)
#define RX_MINIMUM_SPACE_CLOCKS   (uint32_t)(RX_MINIMUM_SPACE * CLOCKS_PER_MICROSECOND)

#define CTRL_SET TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_LENGTH |TMR_CTRL_OUTMODE(2)
#define CTRL_CLEAR TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_LENGTH |TMR_CTRL_OUTMODE(1)
#define FlightControllerTGS_MAXCHANNELS 16

FlightControllerTGSBase * FlightControllerTGSBase::list[10];

const FlightControllerTGSBase::TMR_Hardware_t FlightControllerTGSBase::hardware[] = {
	{ 6,1, &IMXRT_TMR4, &CCM_CCGR6, CCM_CCGR6_QTIMER4(CCM_CCGR_ON), IRQ_QTIMER4, &FlightControllerTGSInput::isrTimer4, nullptr, 0},
	{ 9,2, &IMXRT_TMR4, &CCM_CCGR6, CCM_CCGR6_QTIMER4(CCM_CCGR_ON), IRQ_QTIMER4, &FlightControllerTGSInput::isrTimer4, nullptr, 0},
    {10,0, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &FlightControllerTGSInput::isrTimer1, nullptr, 0},
    {11,2, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &FlightControllerTGSInput::isrTimer1, nullptr, 0},
    {12,1, &IMXRT_TMR1, &CCM_CCGR6, CCM_CCGR6_QTIMER1(CCM_CCGR_ON), IRQ_QTIMER1, &FlightControllerTGSInput::isrTimer1, nullptr, 0},
    {13,0, &IMXRT_TMR2, &CCM_CCGR6, CCM_CCGR6_QTIMER2(CCM_CCGR_ON), IRQ_QTIMER2, &FlightControllerTGSInput::isrTimer2, &IOMUXC_QTIMER2_TIMER0_SELECT_INPUT, 1 },
    {14,2, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &FlightControllerTGSInput::isrTimer3, &IOMUXC_QTIMER3_TIMER2_SELECT_INPUT, 1 },
    {15,3, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &FlightControllerTGSInput::isrTimer3, &IOMUXC_QTIMER3_TIMER3_SELECT_INPUT, 1 },
    {18,1, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &FlightControllerTGSInput::isrTimer3, &IOMUXC_QTIMER3_TIMER1_SELECT_INPUT, 0 },
    {19,0, &IMXRT_TMR3, &CCM_CCGR6, CCM_CCGR6_QTIMER3(CCM_CCGR_ON), IRQ_QTIMER3, &FlightControllerTGSInput::isrTimer3, &IOMUXC_QTIMER3_TIMER0_SELECT_INPUT, 1 }
};

const uint8_t FlightControllerTGSBase::_hardware_count =  (sizeof(FlightControllerTGSBase::hardware)/sizeof(FlightControllerTGSBase::hardware[0]));

inline void FlightControllerTGSBase::checkAndProcessTimerCHInPending(uint8_t index, volatile IMXRT_TMR_CH_t *tmr_ch) {
 	if (((tmr_ch->CSCTRL & (TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF1EN)) == (TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF1EN)) 
			|| ((tmr_ch->SCTRL & (TMR_SCTRL_IEF | TMR_SCTRL_IEFIE)) == (TMR_SCTRL_IEF | TMR_SCTRL_IEFIE))) {

 		if 	(list[index]) {
 			list[index]->isr();
 		} else {

 			tmr_ch->CSCTRL &= ~TMR_CSCTRL_TCF1;
			tmr_ch->SCTRL &= ~TMR_SCTRL_IEF;
 		}
 	}
}


void FlightControllerTGSBase::isrTimer1()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(2, &IMXRT_TMR1.CH[0]);
	checkAndProcessTimerCHInPending(3, &IMXRT_TMR1.CH[2]);
	checkAndProcessTimerCHInPending(4, &IMXRT_TMR1.CH[1]);
	asm volatile ("dsb");  
	DBGdigitalWriteFast(2, LOW);
}

void FlightControllerTGSBase::isrTimer2()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(5, &IMXRT_TMR2.CH[0]);
	asm volatile ("dsb");  
	DBGdigitalWriteFast(2, LOW);

}

void FlightControllerTGSBase::isrTimer3()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(6, &IMXRT_TMR3.CH[2]);
	checkAndProcessTimerCHInPending(7, &IMXRT_TMR3.CH[3]);
	checkAndProcessTimerCHInPending(8, &IMXRT_TMR3.CH[1]);
	checkAndProcessTimerCHInPending(9, &IMXRT_TMR3.CH[0]);
	asm volatile ("dsb"); 
	DBGdigitalWriteFast(2, LOW);
}
void FlightControllerTGSBase::isrTimer4()
{
	DBGdigitalWriteFast(2, HIGH);
	checkAndProcessTimerCHInPending(0, &IMXRT_TMR4.CH[1]);
	checkAndProcessTimerCHInPending(1, &IMXRT_TMR4.CH[2]);
	asm volatile ("dsb");  
	DBGdigitalWriteFast(2, LOW);
}

FlightControllerTGSOutput::FlightControllerTGSOutput(void)
{
	pulse_width[0] = TX_MINIMUM_FRAME_CLOCKS;
	for (int i=1; i <= FlightControllerTGS_MAXCHANNELS; i++) {
		pulse_width[i] = TX_DEFAULT_SIGNAL_CLOCKS;
	}
}

FlightControllerTGSOutput::FlightControllerTGSOutput(int polarity)
{
	pulse_width[0] = TX_MINIMUM_FRAME_CLOCKS;
	for (int i=1; i <= FlightControllerTGS_MAXCHANNELS; i++) {
		pulse_width[i] = TX_DEFAULT_SIGNAL_CLOCKS;
	}
	if (polarity == FALLING) {
		outPolarity = 0;
	} else {
		outPolarity = 1;
	}
}

bool FlightControllerTGSOutput::begin(uint8_t txPin)
{
	return begin(txPin, 255);
}

bool FlightControllerTGSOutput::begin(uint8_t txPin, uint32_t _framePin)
{
#ifdef DEBUG_OUTPUT
	Serial.println(txPin);
#endif
	for (idx_channel = 0; idx_channel < _hardware_count; idx_channel++) {
		if (hardware[idx_channel].pin == txPin) break; 
	}
	if (idx_channel == _hardware_count) return false;

	// make sure the appropriate clock gate is enabled.
	*hardware[idx_channel].clock_gate_register |= hardware[idx_channel].clock_gate_mask;

	uint8_t channel =  hardware[idx_channel].channel;
	volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
	
	tmr_ch->CTRL = 0; // stop
	tmr_ch->CNTR = 0;
	tmr_ch->LOAD = 0;

  //framePin = 2;   // optional select a framePin
	if (_framePin < NUM_DIGITAL_PINS) {
		framePin = _framePin;
		pinMode(framePin,OUTPUT);
		digitalWriteFast(framePin,HIGH);
	}
	
	tmr_ch->COMP1 = 200;  // first time
	state = 0;
	tmr_ch->CMPLD1 = TX_PULSE_WIDTH_CLOCKS;
	//TMR1_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_OPS to make falling
	if(outPolarity == 0){
	  tmr_ch->SCTRL = TMR_SCTRL_OEN | TMR_SCTRL_OPS;
	} else {
	  tmr_ch->SCTRL = TMR_SCTRL_OEN ;
	}

	tmr_ch->CSCTRL = TMR_CSCTRL_CL1(1);
	attachInterruptVector(hardware[idx_channel].interrupt, hardware[idx_channel].isr);
	tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);  // clear
	tmr_ch->CSCTRL |= TMR_CSCTRL_TCF1EN;  // enable interrupt
	NVIC_SET_PRIORITY(hardware[idx_channel].interrupt, 32);
	NVIC_ENABLE_IRQ(hardware[idx_channel].interrupt);
	tmr_ch->CTRL =  CTRL_SET;

	list[idx_channel] = this;
	
	//set Mux for Tx Pin - all timers on ALT1
	*(portConfigRegister(txPin)) = 1;

	return true;
}

bool FlightControllerTGSOutput::write(uint8_t channel, float microseconds)
{
  uint32_t i, sum, space, clocks, num_channels;

  if (channel < 1 || channel > FlightControllerTGS_MAXCHANNELS) return false;
  if (microseconds < TX_MINIMUM_SIGNAL || microseconds > TX_MAXIMUM_SIGNAL) return false;
  clocks = microseconds * CLOCKS_PER_MICROSECOND;
  num_channels = total_channels;
  if (channel > num_channels) num_channels = channel;
  sum = clocks;
  for (i = 1; i < channel; i++) sum += pulse_width[i];
  for (i = channel + 1; i <= num_channels; i++) sum += pulse_width[i];
  if (sum < TX_MINIMUM_FRAME_CLOCKS - TX_MINIMUM_SPACE_CLOCKS) {
    space = TX_MINIMUM_FRAME_CLOCKS - sum;
  } else {
    if (framePin < NUM_DIGITAL_PINS) {
      space = TX_PULSE_WIDTH_CLOCKS;
    } else {
      space = TX_MINIMUM_SPACE_CLOCKS;
    }
  }
  __disable_irq();
  pulse_width[0] = space;
  pulse_width[channel] = clocks;
  total_channels = num_channels;
  __enable_irq();
  return true;
}


void FlightControllerTGSOutput::isr() 
{
	DBGdigitalWriteFast(3, HIGH);
  uint8_t channel = hardware[idx_channel].channel;
  volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
  ticks++;

  // Clear out the Compare match interrupt. 
  tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);
  
  if (state == 0) {
    // pin was just set high, schedule it to go low
	
   tmr_ch->COMP1 = tmr_ch->CMPLD1 = TX_PULSE_WIDTH_CLOCKS;
    tmr_ch->CTRL =  CTRL_CLEAR;
    state = 1;
  } else {
    // pin just went low
    uint32_t width, channel;
    if (state == 1) {
      channel = current_channel;
      if (channel == 0) {
        total_channels_buffer = total_channels;
        for (uint32_t i = 0; i <= total_channels_buffer; i++) {
          pulse_buffer[i] = pulse_width[i];
        }
      }
      width = pulse_buffer[channel] - TX_PULSE_WIDTH_CLOCKS;
      if (++channel > total_channels_buffer) {
        channel = 0;
      }
      if (framePin < NUM_DIGITAL_PINS) {
        if (channel == 1) {
          digitalWriteFast(framePin,HIGH);
        } else {
          digitalWriteFast(framePin,LOW);
        }
      }
      current_channel = channel;
    } else {
      width = pulse_remaining;
    }
    if (width <= 60000) {
      tmr_ch->COMP1 = tmr_ch->CMPLD1 = width;
      tmr_ch->CTRL =  CTRL_SET; // set on compare match & interrupt
      state = 0;
    } else {
      tmr_ch->COMP1 =tmr_ch->CMPLD1 = 58000;
      tmr_ch->CTRL =  CTRL_CLEAR; // clear on compare match & interrupt
      pulse_remaining = width - 58000;
      state = 2;
    }
  }

	DBGdigitalWriteFast(3, LOW);
}


//-----------------------------------------------------------------------------
// FlightControllerTGSOutput 
//-----------------------------------------------------------------------------

FlightControllerTGSInput::FlightControllerTGSInput(void)
{
	outPolarity = 1;
}

FlightControllerTGSInput::FlightControllerTGSInput(int polarity)
{
	if (polarity == FALLING) {
		outPolarity = 0;
	} else {
		outPolarity = 1;
	}
}

bool FlightControllerTGSInput::begin(uint8_t rxPin)
{
	for (idx_channel = 0; idx_channel < _hardware_count; idx_channel++) {
		if (hardware[idx_channel].pin == rxPin) break; 
	}
	if (idx_channel == _hardware_count) return false;

	// make sure the appropriate clock gate is enabled.
	*hardware[idx_channel].clock_gate_register |= hardware[idx_channel].clock_gate_mask;

	uint8_t channel =  hardware[idx_channel].channel;
	volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];

	tmr_ch->CTRL = 0; // stop
	tmr_ch->CNTR = 0;
	tmr_ch->LOAD = 0;
	tmr_ch->CSCTRL = 0;
	tmr_ch->LOAD = 0;  // start val after compare
	tmr_ch->COMP1 = 0xffff;  // count up to this val, interrupt,  and start again
	tmr_ch->CMPLD1 = 0xffff;

	if(outPolarity == 0){
	  tmr_ch->SCTRL = TMR_SCTRL_CAPTURE_MODE(1) | TMR_SCTRL_IPS;  //falling
	} else {
		tmr_ch->SCTRL = TMR_SCTRL_CAPTURE_MODE(1);  //rising
	}

	attachInterruptVector(hardware[idx_channel].interrupt, hardware[idx_channel].isr);
#if 1
	// uses timer match condition. 
	tmr_ch->SCTRL |= TMR_SCTRL_IEFIE;  // enable compare interrupt as well as overflow
	tmr_ch->CSCTRL = TMR_CSCTRL_TCF1EN;  // enable capture interrupt
#else	
	// tries to use overflow condition
	tmr_ch->SCTRL |= TMR_SCTRL_IEFIE | TMR_SCTRL_TOFIE;  // enable compare interrupt as well as overflow
	// tmr_ch->CSCTRL = TMR_CSCTRL_TCF1EN;  // enable capture interrupt
#endif
	NVIC_SET_PRIORITY(hardware[idx_channel].interrupt, 32);
	NVIC_ENABLE_IRQ(hardware[idx_channel].interrupt);
	
	tmr_ch->CTRL =  TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_SCS(channel) | TMR_CTRL_LENGTH ; // prescale
	//tmr_ch->CTRL =  TMR_CTRL_CM(1) | TMR_CTRL_PCS(8 + 2) | TMR_CTRL_SCS(channel) ; // prescale
	
	list[idx_channel] = this;
	
#ifdef DEBUG_OUTPUT
	Serial.printf("FlightControllerTGSInput::begin pin:%d idx: %d CH:%d SC:%x CSC:%x\n", rxPin, idx_channel, channel, tmr_ch->SCTRL, tmr_ch->CSCTRL); Serial.flush();
	//set Mux for Tx Pin - all timers on ALT1
	Serial.printf("Select Input Regster: %x %d\n", (uint32_t)hardware[idx_channel].select_input_register, hardware[idx_channel].select_val); Serial.flush();
#endif
	if (hardware[idx_channel].select_input_register) {
		*hardware[idx_channel].select_input_register = hardware[idx_channel].select_val;
#ifdef DEBUG_OUTPUT
		Serial.println("Select Input completed");Serial.flush();
#endif
	}
	*(portConfigRegister(rxPin)) = 1 | 0x10;

#ifdef DEBUG_OUTPUT
	Serial.printf("  CP1:%x CP2:%x CAPT:%x LOAD:%x \n", (uint16_t)tmr_ch->COMP1, (uint16_t)tmr_ch->COMP2,
		(uint16_t)tmr_ch->CAPT, (uint16_t)tmr_ch->LOAD); 
	Serial.flush();
	Serial.printf("  HOLD:%x CNTR:%x CTRL:%x  SCTRL:%x\n", (uint16_t)tmr_ch->HOLD, (uint16_t)tmr_ch->CNTR, (uint16_t)tmr_ch->CTRL, (uint32_t)tmr_ch->SCTRL); 
	Serial.flush();

	Serial.printf(" CMPLD1:%x, CMPLD2:%x, FILT:%x DMA:%x ENBL:%x\n",tmr_ch->CMPLD1,
		tmr_ch->CMPLD2, tmr_ch->FILT, tmr_ch->DMA, tmr_ch->ENBL);
	Serial.flush();
#endif


	
	return true;
}

int FlightControllerTGSInput::available(void)
{
	uint32_t total;
	bool flag;

	__disable_irq();
	flag = available_flag;
	total = total_channels;
	__enable_irq();
	if (flag) return total;
	return -1;
}

float FlightControllerTGSInput::read(uint8_t channel)
{
	uint32_t total, index, value=0;

	if (channel == 0) return 0.0;
	index = channel - 1;
	__disable_irq();
	total = total_channels;
	if (index < total) value = pulse_buffer[index];
	if (channel >= total) available_flag = false;
	__enable_irq();
	return (float)value / (float)CLOCKS_PER_MICROSECOND;
}


void FlightControllerTGSInput::isr() {  // capture and compare
  DBGdigitalWriteFast(4, HIGH);
  uint8_t channel = hardware[idx_channel].channel;
  volatile IMXRT_TMR_CH_t *tmr_ch = &hardware[idx_channel].tmr->CH[channel];
  
#if 1
  // uses match
  // tries to use overflow 
  if (tmr_ch->CSCTRL & TMR_CSCTRL_TCF1) { // compare rollover
    tmr_ch->CSCTRL &= ~(TMR_CSCTRL_TCF1);  // clear
    overflow_count++;
    overflow_inc = true;
  }
#else 
  // tries to use overflow 
  if (tmr_ch->SCTRL & TMR_SCTRL_TOF) { // compare rollover
    tmr_ch->SCTRL &= ~(TMR_SCTRL_TOF);  // clear
    overflow_count++;
    overflow_inc = true;
  }
#endif  
  if (tmr_ch->SCTRL & TMR_SCTRL_IEF) { // capture
    uint32_t val, count;
    tmr_ch->SCTRL &= ~(TMR_SCTRL_IEF);  // clear
    val = tmr_ch->CAPT;
    count = overflow_count;
    if (val > 0xE000 && overflow_inc) count--;
    val |= (count << 16);
    count = val - prev;
    prev = val;
    if (count >= RX_MINIMUM_SPACE_CLOCKS) {
      if (write_index < 255) {
        for (int i = 0; i < write_index; i++) {
          pulse_buffer[i] = pulse_width[i];
        }
        total_channels = write_index;
        available_flag = true;
      }
      write_index = 0;
    } else {
      if (write_index < FlightControllerTGS_MAXCHANNELS) {
        pulse_width[write_index++] = count;
      }
    }
  }
  ticks++;
  asm volatile ("dsb");  // wait for clear  memory barrier
  overflow_inc = false;
  DBGdigitalWriteFast(4, LOW);
}

#endif

