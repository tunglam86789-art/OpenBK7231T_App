
#include "../obk_config.h"

#if ENABLE_DRIVER_IRREMOTEESP
// drv_ir_new.cpp - IRremoteESP8266
extern "C" {
	// these cause error: conflicting declaration of 'int bk_wlan_mcu_suppress_and_sleep(unsigned int)' with 'C' linkage
#include "../new_common.h"

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../obk_config.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_generic.h"
#include "../hal/hal_hwtimer.h"

#if PLATFORM_BEKEN
#include "include.h"
#include "arm_arch.h"
#elif PLATFORM_LN882H || PLATFORM_LN8825
#define delay_ms OS_MsDelay
#elif PLATFORM_RTL8710B
	int __wrap_atoi(const char* str);
	char* _strncpy(char* dest, const char* src, size_t count);
	int _sscanf_patch(const char* buf, const char* fmt, ...);
//#undef sscanf
#endif

// why can;t I call this?
#include "../mqtt/new_mqtt.h"

	unsigned long ir_counter = 0;
	uint8_t gEnableIRSendWhilstReceive = 0;
	uint32_t gIRProtocolEnable = 0xFFFFFFFF;
	// 0 == active low.  1 = active hi
	uint8_t gIRPinPolarity = 0;

	extern int my_strnicmp(const char* a, const char* b, int len);
	extern unsigned int g_timeMs;
}


#include "drv_ir.h"

//#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE 1
#undef read
#undef write
#undef send
//#define PROGMEM


//#define NO_LED_FEEDBACK_CODE 1

//typedef unsigned char uint_fast8_t;
typedef unsigned short uint16_t;

#define __FlashStringHelper char

// dummy functions
#if PLATFORM_BEKEN
void noInterrupts() { }
void interrupts() { }
void delay(int n) { }
void delayMicroseconds(int n) { }
unsigned long millis()
{
	return 0;
}
unsigned long micros()
{
	return 0;
}
#else
void noInterrupts() { taskENTER_CRITICAL(); }
void interrupts() { taskEXIT_CRITICAL(); }
void delay(int n) { delay_ms(n); }
void delayMicroseconds(int n) { HAL_Delay_us(n); }
unsigned long millis()
{
	return g_timeMs;
}
unsigned long micros()
{
	return g_timeMs * 1000;
}
#endif


class Print {
public:
	void println(const char *p) {
		return;
	}
	void print(...) {
		return;
	}
};

Print Serial;




#define EXTERNAL_IR_TIMER_ISR

//////////////////////////////////////////
// our external timer interrupt stuff
// this will have already been done
#define TIMER_RESET_INTR_PENDING


// #  if defined(ISR)
// #undef ISR
// #  endif
// #define ISR void IR_ISR

// THIS function is defined in src/libraries/IRremoteESP8266/src/IRrecv.cpp
extern "C" void DRV_IR_ISR(void* arg);
extern void IR_ISR(float period_us);

static int8_t ir_chan = -1;
static float ir_periodus = 50;

void timerConfigForReceive() {
	// nothing here`
}

void _timerConfigForReceive() {
	ir_counter = 0;

	ir_chan = HAL_RequestHWTimer(ir_periodus, &ir_periodus, DRV_IR_ISR, NULL);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer %u, %.2f us period", ir_chan, ir_periodus);
}

static void timer_enable() {
}
static void timer_disable() {
}
static void _timer_enable() {
	HAL_HWTimerStart(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer enabled %u", ir_chan);
}
static void _timer_disable() {
	HAL_HWTimerStop(ir_chan);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"ir timer disabled %u", ir_chan);
}

#define TIMER_ENABLE_RECEIVE_INTR timer_enable();
#define TIMER_DISABLE_RECEIVE_INTR timer_disable();

//////////////////////////////////////////

class SpoofIrReceiver {
public:
	static void restartAfterSend() {

	}
};

SpoofIrReceiver IrReceiver;

#include "../libraries/IRremoteESP8266/src/IRremoteESP8266.h"
#include "../libraries/IRremoteESP8266/src/IRsend.h"
#include "../libraries/IRremoteESP8266/src/IRrecv.h"
#include "../libraries/IRremoteESP8266/src/IRutils.h"
#ifdef ENABLE_IRAC
#include "../libraries/IRremoteESP8266/src/IRac.h"
#endif
#include "../libraries/IRremoteESP8266/src/IRproto.h"
#include "../libraries/IRremoteESP8266/src/digitalWriteFast.h"

// override aspects of sending for our own interrupt driven sends
// basically, IRsend calls mark(us) and space(us) to send.
// we simply note the numbers into a rolling buffer, assume the first is a mark()
// and then every 50us service the rolling buffer, changing the PWM from 0 duty to 50% duty
// appropriately.
#define SEND_MAXBITS 128

class myIRsend : public IRsend {
public:
	myIRsend(uint_fast8_t aSendPin) :IRsend(aSendPin) {
		sendPin = aSendPin;
		our_us = 0;
		our_ms = 0;
		resetsendqueue();
	}
	~myIRsend() { }


	uint32_t millis() {
		return our_ms;
	}

	void delay(long int ms) {
		// add a pure delay to our queue
		space(ms * 1000);
	}

	uint16_t mark(uint16_t aMarkMicros) {
		// sends a high for aMarkMicros
		uint32_t newtimein = (timein + 1) % (SEND_MAXBITS * 2);
		if (newtimein != timeout) {
			// store mark bits in highest +ve bit of count
			times[timein] = aMarkMicros | 0x10000000;
			timein = newtimein;
			timecount++;
			timecounttotal++;
		}
		else {
			overflows++;
		}
		return 1;
	}

	void space(uint32_t aMarkMicros) {
		// sends a low for aMarkMicros
		uint32_t newtimein = (timein + 1) % (SEND_MAXBITS * 2);
		if (newtimein != timeout) {
			times[timein] = aMarkMicros;
			timein = newtimein;
			timecount++;
			timecounttotal++;
		}
		else {
			overflows++;
		}
	}

	void enableIROut(uint32_t freq, uint8_t duty=50) {
		//uint_fast8_t aFrequencyKHz
		if (freq < 1000)  // Were we given kHz? Supports the old call usage.
			freq *= 1000;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"enableIROut %d freq %d duty",(int)freq, (int)duty);
		if(duty<1)
			duty=1;
		pwmduty = duty;

		HAL_PIN_PWM_Start(sendPin, freq);
		//HAL_PIN_PWM_Update(sendPin, duty);
	}

	void resetsendqueue() {
		// sends a low for aMarkMicros
		timein = timeout = 0;
		timecount = 0;
		overflows = 0;
		currentsendtime = 0;
		currentbitval = 0;
		timecounttotal = 0;
	}
	int32_t times[SEND_MAXBITS * 2]; // enough for 128 bits
	unsigned short timein;
	unsigned short timeout;
	unsigned short timecount;
	unsigned short overflows;
	uint32_t timecounttotal;

	int32_t getsendqueue() {
		int32_t val = 0;
		if (timein != timeout) {
			val = times[timeout];
			timeout = (timeout + 1) % (SEND_MAXBITS * 2);
			timecount--;
		}
		return val;
	}
	int currentsendtime;
	int currentbitval;

	uint8_t sendPin;
	uint32_t pwmduty;

	uint32_t our_ms;
	float our_us;
};

// our send/receive instances
myIRsend *pIRsend = NULL;
IRrecv *ourReceiver = NULL;

static uint16_t *gLastRaw = NULL;
static uint16_t gLastRawLen = 0;
static bool gIRTxWasBusy = false;
static bool IR_ValueInRange(
    const uint32_t value,
    const uint32_t minimum,
    const uint32_t maximum
) {
    return value >= minimum && value <= maximum;
}

static uint32_t IR_GetRawUsecs(
    const decode_results *results,
    const uint16_t index
) {
    return (uint32_t)results->rawbuf[index] * kRawTick;
}

static uint32_t IR_Abs32(const int32_t value) {
    return value < 0
        ? (uint32_t)(-value)
        : (uint32_t)value;
}

/*
 * Gom dữ liệu từ một khung COOLIX.
 *
 * Mỗi byte thật được theo sau bởi byte đảo.
 * Với mỗi bit:
 *   dataSpace > inverseSpace  => bit 1
 *   dataSpace < inverseSpace  => bit 0
 *
 * Không bắt buộc từng timing phải hoàn hảo.
 * Cặp timing quá méo sẽ được bỏ qua và lấy từ khung lặp khác.
 */
static bool IR_AccumulateCoolixFrame(
    const decode_results *results,
    const uint16_t start,
    int32_t bitSum[24],
    int32_t bitBest[24],
    uint8_t bitSamples[24]
) {
    if (!results || !results->rawbuf) {
        return false;
    }

    // 2 header + 96 data timings + 1 footer.
    if ((uint32_t)start + 98u >= results->rawlen) {
        return false;
    }

    const uint32_t headerMark =
        IR_GetRawUsecs(results, start);

    const uint32_t headerSpace =
        IR_GetRawUsecs(results, start + 1);

    if (!IR_ValueInRange(headerMark, 2500, 6500) ||
        !IR_ValueInRange(headerSpace, 2500, 6500)) {
        return false;
    }

    const uint32_t footerMark =
        IR_GetRawUsecs(results, start + 98);

    if (!IR_ValueInRange(footerMark, 80, 1600)) {
        return false;
    }

    uint8_t saneMarks = 0;

    // Kiểm tra tổng thể 48 MARK, không loại vì chỉ một MARK xấu.
    for (uint16_t rawBit = 0; rawBit < 48; rawBit++) {
        const uint16_t markIndex =
            start + 2 + rawBit * 2;

        const uint32_t mark =
            IR_GetRawUsecs(results, markIndex);

        if (IR_ValueInRange(mark, 80, 1600)) {
            saneMarks++;
        }
    }

    if (saneMarks < 36) {
        return false;
    }

    int32_t localDelta[24] = {0};
    bool localValid[24] = {false};
    uint8_t usableBits = 0;

    for (uint16_t group = 0; group < 3; group++) {
        for (uint16_t bit = 0; bit < 8; bit++) {
            const uint16_t outputBit =
                group * 8 + bit;

            const uint16_t dataRawBit =
                group * 16 + bit;

            const uint16_t inverseRawBit =
                dataRawBit + 8;

            const uint16_t dataSpaceIndex =
                start + 3 + dataRawBit * 2;

            const uint16_t inverseSpaceIndex =
                start + 3 + inverseRawBit * 2;

            const uint32_t dataSpace =
                IR_GetRawUsecs(results, dataSpaceIndex);

            const uint32_t inverseSpace =
                IR_GetRawUsecs(results, inverseSpaceIndex);

            // Chỉ bỏ đúng cặp timing quá vô lý.
            if (!IR_ValueInRange(dataSpace, 80, 3200) ||
                !IR_ValueInRange(inverseSpace, 80, 3200)) {
                continue;
            }

            const int32_t delta =
                (int32_t)dataSpace -
                (int32_t)inverseSpace;

            // Hai timing gần bằng nhau thì cặp này không đủ tin cậy.
            if (IR_Abs32(delta) < 120) {
                continue;
            }

            localDelta[outputBit] = delta;
            localValid[outputBit] = true;
            usableBits++;
        }
    }

    // Khung phải cung cấp được phần lớn số bit.
    if (usableBits < 14) {
        return false;
    }

    // Chỉ nhập dữ liệu sau khi xác nhận đây là một khung hợp lệ.
    for (uint16_t bit = 0; bit < 24; bit++) {
        if (!localValid[bit]) {
            continue;
        }

        const int32_t delta = localDelta[bit];

        bitSum[bit] += delta;
        bitSamples[bit]++;

        if (IR_Abs32(delta) >
            IR_Abs32(bitBest[bit])) {
            bitBest[bit] = delta;
        }
    }

    return true;
}

/*
 * Khôi phục COOLIX từ UNKNOWN hoặc COOLIX48 nhận nhầm.
 *
 * - Quét mọi khung COOLIX trong rawbuf.
 * - Gom các khung lặp.
 * - Mỗi bit lấy tổng độ tin cậy của tất cả khung.
 * - Khi tổng bị triệt tiêu, dùng mẫu riêng có độ chênh lớn nhất.
 */
static bool IR_TryDecodeCoolixFallback(
    decode_results *results
) {
    if (!results ||
        !results->rawbuf ||
        results->rawlen < 99) {
        return false;
    }

    if (results->decode_type != decode_type_t::UNKNOWN &&
        results->decode_type != decode_type_t::COOLIX48) {
        return false;
    }

    /*
     * COOLIX48 đôi khi chính là COOLIX thường đã được thư viện
     * đọc thành 48 bit. Nếu ba cặp byte đảo hoàn hảo thì đổi thẳng.
     */
    if (results->decode_type == decode_type_t::COOLIX48 &&
        results->bits == 48) {

        const uint64_t raw48 = results->value;

        const uint8_t b0 = (uint8_t)(raw48 >> 40);
        const uint8_t b1 = (uint8_t)(raw48 >> 32);
        const uint8_t b2 = (uint8_t)(raw48 >> 24);
        const uint8_t b3 = (uint8_t)(raw48 >> 16);
        const uint8_t b4 = (uint8_t)(raw48 >> 8);
        const uint8_t b5 = (uint8_t)raw48;

        if (b1 == (uint8_t)(b0 ^ 0xFFu) &&
            b3 == (uint8_t)(b2 ^ 0xFFu) &&
            b5 == (uint8_t)(b4 ^ 0xFFu)) {

            const uint32_t code =
                ((uint32_t)b0 << 16) |
                ((uint32_t)b2 << 8) |
                (uint32_t)b4;

            results->decode_type = decode_type_t::COOLIX;
            results->bits = 24;
            results->value = code;
            results->address = 0;
            results->command = 0;
            results->repeat = false;

            return true;
        }
    }

    int32_t bitSum[24] = {0};
    int32_t bitBest[24] = {0};
    uint8_t bitSamples[24] = {0};
    uint8_t frameCount = 0;

    /*
     * rawbuf[0] là khoảng nghỉ trước tín hiệu.
     * MARK luôn ở vị trí lẻ: 1, 3, 5...
     * Quét vị trí lẻ để không nhầm GAP + header thành header.
     */
    for (uint16_t start = 1;
         (uint32_t)start + 98u < results->rawlen;
         start += 2) {

        if (IR_AccumulateCoolixFrame(
                results,
                start,
                bitSum,
                bitBest,
                bitSamples)) {
            frameCount++;
        }
    }

    if (frameCount == 0) {
        return false;
    }

    uint32_t code = 0;

    for (uint16_t bit = 0; bit < 24; bit++) {
        if (bitSamples[bit] == 0) {
            return false;
        }

        int32_t decision = bitSum[bit];

        /*
         * Nếu hai khung có một khung méo ngược dấu làm tổng gần 0,
         * lấy cặp timing riêng có độ tách lớn nhất.
         */
        if (IR_Abs32(decision) < 180) {
            decision = bitBest[bit];
        }

        if (IR_Abs32(decision) < 120) {
            return false;
        }

        code <<= 1;

        if (decision > 0) {
            code |= 1u;
        }
    }

    /*
     * Toàn bộ mã Funiki đã thu thực tế đều thuộc họ COOLIX 0xBxxxxx.
     * Điều kiện này ngăn nhiễu dài bị ép nhầm thành COOLIX.
     */
    if ((code & 0xF00000u) != 0xB00000u) {
        return false;
    }

    results->decode_type = decode_type_t::COOLIX;
    results->bits = 24;
    results->value = code;
    results->address = 0;
    results->command = 0;
    results->repeat = false;

    return true;
}
// this is our ISR.
// it is called every 50us, so we need to work on making it as efficient as possible.
extern "C" void DRV_IR_ISR(void* arg)
{
	int sending = 0;
	if (pIRsend) {
		pIRsend->our_us += ir_periodus;
		if (pIRsend->our_us > 1000) {
			pIRsend->our_ms++;
			pIRsend->our_us -= 1000;
		}

		int pinval = 0;
		if (pIRsend->currentsendtime) {
			sending = 1;
			pIRsend->currentsendtime -= ir_periodus;
			if (pIRsend->currentsendtime <= 0) {
				int32_t remains = pIRsend->currentsendtime;
				int32_t newtime = pIRsend->getsendqueue();
				if (0 == newtime) {
					// if it was the last one
					pIRsend->currentsendtime = 0;
					pIRsend->currentbitval = 0;
				}
				else {
					// we got a new time
					// store mark bits in highest +ve bit of count
					pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
					pIRsend->currentsendtime = (newtime & 0xfffffff);
					// adjust the us value to keep the running accuracy
					// and avoid a running error?
					// note remains is -ve
					pIRsend->currentsendtime += remains;
				}
			}
		}
		else {
			int32_t newtime = pIRsend->getsendqueue();
			if (!newtime) {
				pIRsend->currentsendtime = 0;
				pIRsend->currentbitval = 0;
			}
			else {
				sending = 1;
				pIRsend->currentsendtime = (newtime & 0xfffffff);
				pIRsend->currentbitval = (newtime & 0x10000000) ? 1 : 0;
			}
		}
		pinval = pIRsend->currentbitval;

		uint32_t duty = pIRsend->pwmduty;
		if (!pinval) {
			if (gIRPinPolarity) {
				duty = 50;
			}
			else {
				duty = 0;
			}
		}
		HAL_PIN_PWM_Update(pIRsend->sendPin, duty);
	}

	// is someone really wants rx and TX at the same time, then allow it.
	if (gEnableIRSendWhilstReceive) {
		sending = 0;
	}

	// don't receive if we are currently sending
	if (ourReceiver && !sending){
		IR_ISR(ir_periodus);
	}
	ir_counter++;
}

static int IR_HexNibble(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

// Send Tuya learned IR code.
// Usage: IRSendTuyaRaw 061106111f023b...
extern "C" commandResult_t IR_Send_TuyaRaw_Cmd(
	const void *context,
	const char *cmd,
	const char *args_in,
	int cmdFlags
) {
	if (!args_in || !args_in[0]) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSendTuyaRaw expects a Tuya hex code");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	if (!pIRsend) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSendTuyaRaw: IR sender is not running");
		return CMD_RES_ERROR;
	}

	// Skip leading spaces.
	while (*args_in == ' ' || *args_in == '\t') {
		args_in++;
	}

	size_t hexLen = strlen(args_in);

	// Each timing is stored by Tuya as four hex characters:
	// low byte first, then high byte. Example 0611 => 0x1106 => 4358 us.
	if (hexLen == 0 || (hexLen % 4) != 0) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSendTuyaRaw: invalid length %d, must be divisible by 4",
			(int)hexLen);
		return CMD_RES_BAD_ARGUMENT;
	}

	size_t timingCount = hexLen / 4;

	// myIRsend has a 256-entry rolling queue, with one entry reserved.
	if (timingCount > ((SEND_MAXBITS * 2) - 1)) {
		ADDLOG_ERROR(LOG_FEATURE_IR,
			(char *)"IRSendTuyaRaw: too many timings %d, maximum %d",
			(int)timingCount,
			(int)((SEND_MAXBITS * 2) - 1));
		return CMD_RES_BAD_ARGUMENT;
	}

	pIRsend->resetsendqueue();
	pIRsend->enableIROut(38000, 33);

	for (size_t i = 0; i < timingCount; i++) {
		const char *s = args_in + (i * 4);

		int h0 = IR_HexNibble(s[0]);
		int h1 = IR_HexNibble(s[1]);
		int h2 = IR_HexNibble(s[2]);
		int h3 = IR_HexNibble(s[3]);

		if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0) {
			pIRsend->resetsendqueue();
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IRSendTuyaRaw: invalid hex at position %d",
				(int)(i * 4));
			return CMD_RES_BAD_ARGUMENT;
		}

		uint16_t lowByte  = (uint16_t)((h0 << 4) | h1);
		uint16_t highByte = (uint16_t)((h2 << 4) | h3);
		uint16_t usec = (uint16_t)(lowByte | (highByte << 8));

		if (usec == 0) {
			pIRsend->resetsendqueue();
			ADDLOG_ERROR(LOG_FEATURE_IR,
				(char *)"IRSendTuyaRaw: zero timing at index %d",
				(int)i);
			return CMD_RES_BAD_ARGUMENT;
		}

		// Tuya sequence begins with a MARK, then alternates MARK/SPACE.
		if ((i & 1) == 0) {
			pIRsend->mark(usec);
		}
		else {
			pIRsend->space(usec);
		}
	}

	ADDLOG_INFO(LOG_FEATURE_IR,
		(char *)"IRSendTuyaRaw queued %d timings at 38kHz",
		(int)timingCount);

	return CMD_RES_OK;
}
extern "C" commandResult_t IR_Replay_Last_Cmd(
    const void *context,
    const char *cmd,
    const char *args_in,
    int cmdFlags
) {
    if (!pIRsend) {
        ADDLOG_ERROR(
            LOG_FEATURE_IR,
            (char *)"IRReplayLast: IR sender is not running"
        );
        return CMD_RES_ERROR;
    }

    if (!gLastRaw || gLastRawLen == 0) {
        ADDLOG_ERROR(
            LOG_FEATURE_IR,
            (char *)"IRReplayLast: no captured IR data"
        );
        return CMD_RES_ERROR;
    }

    const uint16_t maxTimings = (SEND_MAXBITS * 2) - 1;

    if (gLastRawLen > maxTimings) {
        ADDLOG_ERROR(
            LOG_FEATURE_IR,
            (char *)"IRReplayLast: RAW too long %d, maximum %d",
            (int)gLastRawLen,
            (int)maxTimings
        );
        return CMD_RES_BAD_ARGUMENT;
    }

    // Không cho nhấn thêm khi tín hiệu trước vẫn đang phát.
    if (pIRsend->timecount != 0 || pIRsend->currentsendtime != 0) {
        ADDLOG_ERROR(
            LOG_FEATURE_IR,
            (char *)"IRReplayLast: IR sender is busy"
        );
        return CMD_RES_ERROR;
    }

    // Bảo đảm hàng đợi sạch trước khi đưa RAW mới vào.
    pIRsend->resetsendqueue();

    pIRsend->sendRaw(gLastRaw, gLastRawLen, 38);

    if (pIRsend->overflows) {
        ADDLOG_ERROR(
            LOG_FEATURE_IR,
            (char *)"IRReplayLast: send queue overflow"
        );
        pIRsend->resetsendqueue();
        return CMD_RES_ERROR;
    }

    ADDLOG_INFO(
        LOG_FEATURE_IR,
        (char *)"IRReplayLast queued %d raw timings at 38kHz",
        (int)gLastRawLen
    );

    return CMD_RES_OK;
}
extern "C" commandResult_t IR_Send_Cmd(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	char args[128];
	strncpy(args, args_in, sizeof(args) - 1);
	args[sizeof(args) - 1] = 0;

	// split arg at hyphen;
	char *p = args;
	while (*p && (*p != '-') && (*p != ' ')) {
		p++;
	}

	if ((*p != '-') && (*p != ' ')) {
		// try to decode "new" format, separated by comma
		// the format is bits,0xDATA[,repeat]
		char *p = args;
		while (*p && (*p != ',')) {
			p++;
		}
		if(*p==',')
		{
			*p='\0';
			decode_type_t protocol = strToDecodeType(args);
			p++;
			char *_bits=p;
			while (*p && (*p != ',')) {
				p++;
			}
			if(*p==',')
			{
				*p='\0';
				uint16_t bits = (uint16_t)strtol(_bits,NULL,10);
				p++;
				if(bits<=64)
				{
					char *_data=p;
					uint64_t data =  strtoll(_data,&p,16);
					if(protocol!=decode_type_t::UNKNOWN)
					{
						int repeats=1;
						if(*p==',')
							repeats=strtol(p+1,NULL,10);
						
						if( pIRsend->send(protocol,data,bits,repeats) )
						{
							pIRsend->delay(100);
						
							ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send %s: protocol %d bits %d data 0x%llX repeats %d", args, (int)protocol, (int)bits, (long long int)data, (int)repeats);
							return CMD_RES_OK;
						} else {
							ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR can't send %s: protocol %d bits data 0x%llX repeats %d", args, (int)protocol, (long long int)data, (int)repeats);
							return CMD_RES_BAD_ARGUMENT;
						}
					}
				} else {
					// TODO: implement longer protocols
					ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend currently only protocol with up to 64bits are supported", args);
					return CMD_RES_BAD_ARGUMENT;
				}
			} 
		}
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend cmnd not valid [%s] not like [NEC-0-1A] or [NEC 0 1A 1] or [NEC,bits,0xDATA,[repeat]]", args);
 		return CMD_RES_BAD_ARGUMENT;
	}

	*p='\0';
	decode_type_t protocol = strToDecodeType(args);
	if(hasACState(protocol))
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend can't send AC commands", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	p++;
	int addr = strtol(p, &p, 16);
	if ((*p != '-') && (*p != ' ')) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRSend cmnd not valid [%s] not like [NEC-0-1A] or [NEC 0 1A 1].", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	p++;
	int command = strtol(p, &p, 16);

	int repeats = 0;

	if ((*p == '-') || (*p == ' ')) {
		p++;
		repeats = strtol(p, &p, 16);
	}

	if (pIRsend) {
		bool success = true;  // Assume success.

		switch(protocol)
		{
			case decode_type_t::RC5:
				pIRsend->sendRC5((uint64_t)pIRsend->encodeRC5(addr,command));
				break;
			case decode_type_t::RC6:
				pIRsend->sendRC6((uint64_t)pIRsend->encodeRC6(addr,command));
				break;
			case decode_type_t::NEC:
				pIRsend->sendNEC((uint64_t)pIRsend->encodeNEC(addr,command));
				break;
			case decode_type_t::PANASONIC:
				pIRsend->sendPanasonic((uint16_t)addr,(uint32_t)command);
				break;
			case decode_type_t::JVC:
				pIRsend->sendJVC((uint64_t)pIRsend->encodeJVC(addr,command));
				break;
			case decode_type_t::SAMSUNG:
				pIRsend->sendSAMSUNG((uint64_t)pIRsend->encodeSAMSUNG(addr,command));
				break;
			case decode_type_t::LG:
				pIRsend->sendLG((uint64_t)pIRsend->encodeLG(addr,command));
				break;
			default:
				ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IR send %s protocol not supported", args);
				return CMD_RES_ERROR;
				break;
		};

		// add a 100ms delay after command
		// NOTE: this is NOT a delay here.  it adds 100ms 'space' in the TX queue
		pIRsend->delay(100);

		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR send %s protocol %d addr 0x%X cmd 0x%X repeats %d", args, (int)protocol, (int)addr, (int)command, (int)repeats);
		return CMD_RES_OK;
	}
	else {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR NOT send (no IRsend running) %s protocol %d addr 0x%X cmd 0x%X repeats %d", args, (int)protocol, (int)addr, (int)command, (int)repeats);
	}
	return CMD_RES_ERROR;
}

extern "C" commandResult_t IR_Enable(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in || !args_in[0]) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IREnable expects arguments");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	char args[128];
	strncpy(args, args_in, sizeof(args)-1);
	args[sizeof(args)-1] = 0;
	char *p = args;
	int enable = 1;
	if (!my_strnicmp(p, "RXTX", 4)) {
		p += 4;
		if (*p == ' ') {
			p++;
			if (*p) {
				enable = atoi(p);
			}
		}
		gEnableIRSendWhilstReceive = enable;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable RX whilst TX enable set %d", enable);
		return CMD_RES_OK;
	}

	if (!my_strnicmp(p, "invert", 6)) {
		// default normal.
		enable = 0;
		p += 6;
		if (*p == ' ') {
			p++;
			if (*p) {
				enable = atoi(p);
			}
		}
		gIRPinPolarity = enable;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable invert set %d", enable);
		return CMD_RES_OK;
	}


	// find length of first arg.
	while (*p && (*p != ' ')) {
		p++;
	}

	//int numProtocols = sizeof(ProtocolNames)/sizeof(*ProtocolNames);
	#if 0 // number of protocols now is 125 
	// TODO: reimpleemnt this using bigger mask
	int numProtocols = 0;
	int ournamelen = (p - args);
	int protocol = -1;
	for (int i = 0; i < numProtocols; i++) {
		const char *name = "Unknown"; //= ProtocolNames[i];
		int namelen = strlen(name);
		if (!my_strnicmp(name, args, namelen) && (ournamelen == namelen)) {
			protocol = i;
			break;
		}
	}
	if (*p == ' ') {
		p++;
		if (*p) {
			enable = atoi(p);
		}
	}

	uint32_t thisbit = (1 << protocol);
	if (protocol < 0) {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable invalid protocol %s", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	else {
		//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable found protocol %s(%d), enable %d from %s, bitmask 0x%08X", ProtocolNames[protocol], protocol, enable, p, thisbit);
	}
	if (enable) {
		gIRProtocolEnable = gIRProtocolEnable | thisbit;
	}
	else {
		gIRProtocolEnable = gIRProtocolEnable & (~thisbit);
	}
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IREnable Protocol mask now 0x%08X", gIRProtocolEnable);
	#endif //TODO
	return CMD_RES_OK;
}


extern "C" commandResult_t IR_Param(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in || !args_in[0]) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam expects two arguments");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	if(!ourReceiver)
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam: IR receiver disabled");
		return CMD_RES_BAD_ARGUMENT;
	}

	// Set higher if you get lots of random short UNKNOWN messages when nothing
	// should be sending a message.
	// Set lower if you are sure your setup is working, but it doesn't see messages
	// from your device. (e.g. Other IR remotes work.)
	// NOTE: Set this value very high to effectively turn off UNKNOWN detection.	
	int kMinUnknownSize = 12;

	// How much percentage lee way do we give to incoming signals in order to match
	// it?
	// e.g. +/- 25% (default) to an expected value of 500 would mean matching a
	//      value between 375 & 625 inclusive.
	// Note: Default is 25(%). Going to a value >= 50(%) will cause some protocols
	//       to no longer match correctly. In normal situations you probably do not
	//       need to adjust this value. Typically that's when the library detects
	//       your remote's message some of the time, but not all of the time.
	int kTolerancePercentage = 25;  // kTolerance is normally 25%

	int res = sscanf(args_in, "%d %d", &kMinUnknownSize, &kTolerancePercentage);

	if(res!=2)
	{
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRParam invalid parameters %s", args_in);
		return CMD_RES_BAD_ARGUMENT;
	}
	ourReceiver->setUnknownThreshold(kMinUnknownSize);
	ourReceiver->setTolerance(kTolerancePercentage);

	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IRParam MinUnknownSize: %d  Noice tolerance: %d%%", kMinUnknownSize,kTolerancePercentage);
	return CMD_RES_OK;
}



#ifdef ENABLE_IRAC
extern "C" commandResult_t IR_AC_Cmd(const void *context, const char *cmd, const char *args_in, int cmdFlags) {
	if (!args_in) return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	char args[64];
	strncpy(args, args_in, sizeof(args) - 1);
	args[sizeof(args) - 1] = 0;

	// split arg at hyphen;
	char *p = args;
	while (*p && (*p != '-') && (*p != ' ')) {
		p++;
	}
	int ournamelen = (p - args);
	if ((*p != '-') && (*p != ' ')) {
		ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRAC cmnd not valid [%s] ", args);
		return CMD_RES_BAD_ARGUMENT;
	}
	//	decode_type_t protocol = strToDecodeType(args);

	ADDLOG_ERROR(LOG_FEATURE_IR, (char *)"IRAC cmnd not implemented yet", args);

	return CMD_RES_OK;
}
#endif //ENABLE_IRAC


// test routine to start IR RX and TX
// currently fixed pins for testing.
extern "C" void DRV_IR_Init() {
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from extern C CPP");

	int pin = -1; //9;// PWM3/25
	int txpin = -1; //24;// PWM3/25
	bool pup = true;

	// allow user to change them
	pin = PIN_FindPinIndexForRole(IOR_IRRecv, pin);
	if(pin == -1)
	{
		pin = PIN_FindPinIndexForRole(IOR_IRRecv_nPup, pin);
		if(pin >= 0) pup = false;
	}
	txpin = PIN_FindPinIndexForRole(IOR_IRSend, txpin);

	if (ourReceiver){
	     IRrecv *temp = ourReceiver;
	     ourReceiver = NULL;
	     delete temp;
	 }
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: recv pin %i", pin);
	if ((pin >= 0) || (txpin >= 0)) {
	}
	else {
		_timer_disable();
	}

	if (pin >= 0) {
		// setup IRrecv pin as input
		//bk_gpio_config_input_pup((GPIO_INDEX)pin); // enabled by enableIRIn

		//TODO: we should specify buffer size (now set to 1024), timeout (now 90ms) and tolerance 
		 ourReceiver = new IRrecv(pin);
		 ourReceiver->enableIRIn(pup);
	}

	if (pIRsend) {
		myIRsend *pIRsendTemp = pIRsend;
		pIRsend = NULL;
		delete pIRsendTemp;
	}

	if (txpin > 0) {
		// is this pin capable of PWM?
		if (HAL_PIN_CanThisPinBePWM(txpin)) {
			uint32_t pwmfrequency = 38000;
			myIRsend *pIRsendTemp = new myIRsend((uint_fast8_t)txpin);
			pIRsendTemp->resetsendqueue();
			pIRsendTemp->enableIROut(pwmfrequency, 50);

			pIRsend = pIRsendTemp;

			//cmddetail:{"name":"IRSend","args":"[PROT-ADDR-CMD-REP]",
			//cmddetail:"descr":"Sends IR commands in the form PROT-ADDR-CMD-REP, e.g. NEC-1-1A-0",
			//cmddetail:"fn":"IR_Send_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IRSend", IR_Send_Cmd, NULL);
            CMD_RegisterCommand("IRSendTuyaRaw", IR_Send_TuyaRaw_Cmd, NULL);
			CMD_RegisterCommand("IRReplayLast", IR_Replay_Last_Cmd, NULL);
			//cmddetail:{"name":"IRAC","args":"[TODO]",
			//cmddetail:"descr":"Sends IR commands for HVAC control (TODO)",
			//cmddetail:"fn":"IR_AC_Cmd","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			#ifdef ENABLE_IRAC
			CMD_RegisterCommand("IRAC", IR_AC_Cmd, NULL);
			#endif //ENABLE_IRAC
			//cmddetail:{"name":"IREnable","args":"[Str][1or0]",
			//cmddetail:"descr":"Enable/disable aspects of IR.  IREnable RXTX 0/1 - enable Rx whilst Tx.  IREnable [protocolname] 0/1 - enable/disable a specified protocol",
			//cmddetail:"fn":"IR_Enable","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IREnable",IR_Enable, NULL);
			//cmddetail:{"name":"IRParam","args":"[MinSize] [Noise Threshold]",
			//cmddetail:"descr":"Set minimal size of the message and noise threshold",
			//cmddetail:"fn":"IR_Param","file":"driver/drv_ir_new.cpp","requires":"ENABLE_DRIVER_IRREMOTEESP (IRremoteESP8266)",
			//cmddetail:"examples":""}
			CMD_RegisterCommand("IRParam",IR_Param, NULL);
		}
	}
	if ((pin >= 0) || (txpin >= 0)) {
		// both tx and rx need the interrupt
		_timerConfigForReceive();
		delay_ms(10);
		_timer_enable();
	}
}

extern "C" void DRV_IR_Deinit()
{
	_timer_disable();
	HAL_HWTimerDeinit(ir_chan);
}

void dump(decode_results *results) {
	// Dumps out the decode_results structure.
	// Call this after IRrecv::decode()
	ADDLOG_INFO(LOG_FEATURE_IR, resultToHumanReadableBasic(results).c_str());

	#ifdef ENABLE_IRAC
	if (hasACState(results->decode_type))
	{
		ADDLOG_INFO(LOG_FEATURE_IR, IRAcUtils::resultAcToString(results).c_str());
	}
	#endif
}



////////////////////////////////////////////////////
// this polls the IR receive to see if there was any IR received
extern "C" void DRV_IR_RunFrame() {
	// Debug-only check to see if the timer interrupt is running
	if (ir_counter) {
		//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR counter: %u", ir_counter);
	}
	if (pIRsend) {
    if (pIRsend->overflows) {
        ADDLOG_DEBUG(
            LOG_FEATURE_IR,
            (char *)"##### IR send overflows %d",
            (int)pIRsend->overflows
        );

        pIRsend->resetsendqueue();
        gIRTxWasBusy = false;

        if (ourReceiver) {
            ourReceiver->resume();
            ADDLOG_INFO(
                LOG_FEATURE_IR,
                (char *)"IR RX resumed after TX overflow"
            );
        }
    }
    else {
        const bool txBusy =
            (pIRsend->timecount != 0) ||
            (pIRsend->currentsendtime != 0);

        if (txBusy) {
            gIRTxWasBusy = true;
        }
        else if (gIRTxWasBusy) {
            gIRTxWasBusy = false;

            if (ourReceiver) {
                ourReceiver->resume();
                ADDLOG_INFO(
                    LOG_FEATURE_IR,
                    (char *)"IR RX resumed after TX completed"
                );
            }
        }
    }
}
	if (ourReceiver) {
		decode_results results;
		if (ourReceiver->decode(&results)) {
			if (IR_TryDecodeCoolixFallback(&results)) {
    ADDLOG_INFO(
        LOG_FEATURE_IR,
        (char *)"IR COOLIX fallback recovered: 0x%06lX",
        (unsigned long)results.value
    );
}
			// TODO: find a better way?
			String proto_name = typeToString(results.decode_type, results.repeat).c_str();

			#if 0 // TODO: implement different masking
			if (!(gIRProtocolEnable & (1 << (int)results.decode_type))) {
				ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR decode ignore masked protocol %s (%d) - mask 0x%08X", proto_name.c_str(), (int)results.decode_type, gIRProtocolEnable);
			}
			#endif

			//dump(&results);
			// 'UNKNOWN' protocol is by default disabled in flags
			// This is because I am getting a lot of 'UNKNOWN' spam with no IR signals in room
			if (((results.decode_type != decode_type_t::UNKNOWN) ||
				(results.decode_type == decode_type_t::UNKNOWN && CFG_HasFlag(OBK_FLAG_IR_ALLOW_UNKNOWN))) //&&
				// only process if this protocol is enabled.  all by default.
				//(gIRProtocolEnable & (1 << (int)results.decode_type)
				) {
				String lastIrReceived = String((int)results.decode_type, 16) + "," + resultToHexidecimal(&results);

				if (!hasACState(results.decode_type))
					lastIrReceived += "," + String((int)results.bits);
				else
					ADDLOG_INFO(LOG_FEATURE_IR, "Received AC code:%s",proto_name.c_str());

				char out[128];

				int repeat = results.repeat?0:1; // not sure how to deal with this

				if (results.decode_type == decode_type_t::UNKNOWN) {
					//snprintf(out, sizeof(out), "IR_RAW 0x%lX %d", (unsigned long)results.decodedRawData, repeat);
					snprintf(out, sizeof(out), "IR %s %s", "Unknown", lastIrReceived.c_str());
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
				}
				else if (!hasACState(results.decode_type)) {
					snprintf(out, sizeof(out), "IR %s %lX %lX %d", proto_name.c_str(), (long int)results.address, (long int)results.command, repeat);
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
					// show new format too
					snprintf(out, sizeof(out), "IR %s,%d,%s", proto_name.c_str(), (int)results.bits, resultToHexidecimal(&results).c_str());
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
				} else {
					#ifdef ENABLE_IRAC
					String description = IRAcUtils::resultAcToString(&results);
					ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IRAC %s", description.c_str());
					#endif //ENABLE_IRAC
				}
				// if user wants us to publish every received IR data, do it now
				if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED)) {

					// another flag required?
					int publishrepeats = 1;

					if (publishrepeats || !repeat) {
						//ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR MQTT publish %s", out);

						uint32_t counter_in = ir_counter;
						MQTT_PublishMain_StringString("ir", lastIrReceived.c_str(), 0);
						uint32_t counter_dur = ((ir_counter - counter_in) * 50) / 1000;
						ADDLOG_INFO(LOG_FEATURE_IR, (char *)"IR MQTT publish %s took %dms", out, counter_dur);
					}
					else {
						ADDLOG_INFO(LOG_FEATURE_IR, (char *)out);
					}
				}

				if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED_IN_JSON)) {
					// {"IrReceived":{"Protocol":"RC_5","Bits":0x1,"Data":"0xC"}}
					//
					String _data=resultToHexidecimal(&results);
					snprintf(out, sizeof(out), "{\"IrReceived\":{\"Protocol\":\"%s\",\"Bits\":%i,\"Data\":\"%s\"}}",
						proto_name.c_str(), (int)results.bits, _data.c_str());
					MQTT_PublishMain_StringString("RESULT", out, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
				}

				if (results.decode_type != decode_type_t::UNKNOWN) {
					snprintf(out, sizeof(out), "%X", results.command);
					int tgType = 0;
					switch (results.decode_type)
					{
					case decode_type_t::NEC:
						tgType = CMD_EVENT_IR_NEC;
						break;
					case decode_type_t::SAMSUNG:
						tgType = CMD_EVENT_IR_SAMSUNG;
						break;
					case decode_type_t::SHARP:
						tgType = CMD_EVENT_IR_SHARP;
						break;
					case decode_type_t::RC5:
						tgType = CMD_EVENT_IR_RC5;
						break;
					case decode_type_t::RC6:
						tgType = CMD_EVENT_IR_RC6;
						break;
					case decode_type_t::SONY:
						tgType = CMD_EVENT_IR_SONY;
						break;
					default:
						break;
					}

					// we should include repeat here?
					// e.g. on/off button should not toggle on repeats, but up/down probably should eat them.
					uint32_t counter_in = ir_counter;
					EventHandlers_FireEvent2(tgType, results.address, results.command);
					uint32_t counter_dur = ((ir_counter - counter_in) * 50) / 1000;
					ADDLOG_DEBUG(LOG_FEATURE_IR, (char *)"IR fire event took %dms", counter_dur);
				}
	            } else {
                ADDLOG_INFO(LOG_FEATURE_IR, "Received Unknown IR ");
            }

ADDLOG_INFO(
    LOG_FEATURE_IR,
    (char *)"IR RX diag: rawlen=%d overflow=%d type=%d bits=%d",
    (int)results.rawlen,
    results.overflow ? 1 : 0,
    (int)results.decode_type,
    (int)results.bits
);

// Buffer thu đã đầy: bỏ khung lỗi, không chuyển thành RAW.
if (results.overflow) {
    if (results.decode_type == decode_type_t::COOLIX) {
        ADDLOG_INFO(
            LOG_FEATURE_IR,
            (char *)"IR RX overflow: COOLIX recovered, RAW not saved"
        );
    }
    else {
        ADDLOG_INFO(
            LOG_FEATURE_IR,
            (char *)"IR RX overflow: no valid COOLIX, RAW discarded"
        );
    }

    ourReceiver->resume();
    return;
}
// Xóa RAW cũ.
if (gLastRaw) {
    delete[] gLastRaw;
    gLastRaw = NULL;
    gLastRawLen = 0;
}

// Chỉ chuyển đổi khung hợp lệ.
gLastRawLen = getCorrectedRawLength(&results);
gLastRaw = resultToRawArray(&results);

if (!gLastRaw || gLastRawLen == 0) {
    gLastRawLen = 0;

    ADDLOG_INFO(
        LOG_FEATURE_IR,
        (char *)"IR capture conversion failed"
    );
}
else {
    ADDLOG_INFO(
        LOG_FEATURE_IR,
        (char *)"Captured corrected raw IR: %d timings",
        (int)gLastRawLen
    );
}
ourReceiver->resume();
        }
    }
}
#ifdef TEST_CPP
// routines to test C++
class cpptest2 {
public:
	int initialised;
	cpptest2() {
		// remove else static class may kill us!!!ADDLOG_INFO(LOG_FEATURE_IR, "Log from Class constructor");
		initialised = 42;
	};
	~cpptest2() {
		initialised = 24;
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from Class destructor");
	}

	void print() {
		ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from Class %d", initialised);
	}
};

cpptest2 staticclass;

void cpptest() {
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from CPP");
	cpptest2 test;
	test.print();
	cpptest2 *test2 = new cpptest2();
	test2->print();
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"Log from static class (is it initialised?):");
	staticclass.print();
}
#endif

#endif

