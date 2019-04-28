
#include <TimerOne.h>
#include "SimpleModbusSlave.h"
#include <EEPROM.h>

#pragma region Pins
#define CLOSED 3
#define OPENED 4
#define DIR 5
#define DISABLE 6
#define STEP 7	
#pragma endregion
//
//
//
#pragma region Modbus
#define ID 1      // адрес ведомого
#define TxEnablePin 2	   // Tx/Rx пин RS485
#define baudrate 19200  // скорость обмена по последовательному интерфейсу. (UART)
#define timeout 1000  // Длительность ожидание ответа (таймаут modbus)
#define polling 200   // скорость опроса по modbus
#define retry_count 10    // количесво запросов modbus до ошибки и останова обмена
#pragma endregion

#define STATE_WAITING 0
#define STATE_OPENED 1
#define STATE_CLOSED 2
#define STATE_OPENING 3
#define STATE_CLOSING 4
#define STATE_ERROR 5
#define STATE_SAVE_SETTINGS 6

#define CMD_OPEN 1
#define CMD_CLOSE 2
#define CMD_STOP 3
#define CMD_RESET 4
#define CMD_SAVE 5

#define MAX_STEPS 65535
#define MAX_CHUNK_LENGTH 2000
#define MAX_CHUNK_DELAY 65535

enum
{
	//Просто добавьте или удалите регистры. Первый регистр начинается по адресу 0
	cmd,          //  команда адр. 0
	steps,          //  число шагов адр. 1
	chunkLength,	// число импульсов в пачке	
	errors,			// ошибки	
	state,
	chunkDelay,
	HOLDING_REGS_SIZE // Это не удалять размер массива HOLDING_REGS.
	//общее количество регистров для функции 3 и 16 разделяет тотже самый массив регистров
	//т.е. то же самое адресное пространство
};


unsigned int holdingRegs[HOLDING_REGS_SIZE]; // функции 3 и 16 массив регистров

struct var {
public:
	short value;
	int hAddr;
	int lAddr;

	bool Save() {
		if (hAddr == lAddr)
			return false;
		if (value <= 0)
			return false;
		byte hi = highByte(value);
		byte lo = lowByte(value);
		EEPROM.write(hAddr, hi);
		EEPROM.write(lAddr, lo);
		return true;
	}
	void Load() {
		byte hi = EEPROM.read(hAddr);
		byte lo = EEPROM.read(lAddr);
		v = word(hi, lo);
		if (v < 65535)
			value = v;
		else
			value = 0;
	}
private:
	int v;

};

static var _steps;
var _chunkLength;
var _chunkDelay;

unsigned long prevMillis = 0;

void setup() {
	// инициализируем пины концевиков
	pinMode(OPENED, INPUT);
	pinMode(CLOSED, INPUT);
	// инициализируем управляющие пины
	pinMode(DIR, OUTPUT);
	pinMode(STEP, OUTPUT);
	pinMode(DISABLE, OUTPUT);
	//Timer1.initialize(500000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
	//Timer1.attachInterrupt(modbusUpdate);
	//pinMode(12, OUTPUT);
	//pinMode(11, OUTPUT);
	//pinMode(10, OUTPUT);
	/*parameters(HardwareSerial* SerialPort,long baudrate,unsigned char byteFormat,unsigned char ID,
	unsigned char transmit enable pin,unsigned int holding registers size,unsigned int* holding register array)
	SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
	SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
	SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
	SERIAL_8N1 option*/

	_steps.hAddr = 1;
	_steps.lAddr = 0;
	_steps.Load();

	_chunkLength.hAddr = 5;
	_chunkLength.lAddr = 4;
	_chunkLength.Load();

	_chunkDelay.hAddr = 7;
	_chunkDelay.lAddr = 6;
	_chunkDelay.Load();

	modbus_configure(&Serial, baudrate, SERIAL_8N1, ID, TxEnablePin, HOLDING_REGS_SIZE, holdingRegs);
	modbus_update_comms(baudrate, SERIAL_8N1, 1);
}



bool opened = false;
bool closed = false;

bool opening = false;
bool closing = false;

int _cmd = 0;
int _state = STATE_WAITING;

int _chunkStep = 0;
int _stepCount = 0;

int _errors = 0;



bool isFirst = false;

void loop() {
	updateKeys();
	if (_errors != 0) {
		_cmd = 0;
		_state = STATE_ERROR;
	}

	modbusUpdate();
	switch (_state) {
	case STATE_WAITING:
		_stepCount = 0;
		_chunkStep = 0;
		digitalWrite(STEP, LOW);
		if (_cmd == CMD_OPEN)
			_state = STATE_OPENING;
		else if (_cmd == CMD_CLOSE)
			_state = STATE_CLOSING;
		else if (_cmd == CMD_SAVE)
			_state = STATE_SAVE_SETTINGS;
		if (opened) {
			_state = STATE_OPENED;
		}
		if (closed) {
			_state = STATE_CLOSED;
		}
		break;
	case STATE_OPENED:
		_stepCount = 0;
		if (_cmd == CMD_CLOSE)
			_state = STATE_CLOSING;
		if (_cmd == CMD_SAVE)
			_state = STATE_SAVE_SETTINGS;
		break;
	case STATE_CLOSED:
		_stepCount = 0;
		if (_cmd == CMD_OPEN)
			_state = STATE_OPENING;
		if (_cmd == CMD_SAVE)
			_state = STATE_SAVE_SETTINGS;
		break;
	case STATE_OPENING:
		if (_cmd == CMD_STOP) {
			_cmd = 0;
			_state = STATE_WAITING;
		}

		if (opened) {
			_cmd = 0;
			_state = STATE_WAITING;
		}

		if (_stepCount >= _steps.value) {
			_cmd = 0;
			_state = STATE_WAITING;
			
		}
		else {
			if (!opening) {
				Timer1.initialize(_chunkDelay.value * 100); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
				Timer1.attachInterrupt(openValveLittleBit);
				opening = true;
			}
			
			//openValveLittleBit();			
		}
		break;
	case STATE_CLOSING:
		if (_cmd == CMD_STOP) {
			_cmd = 0;
			_state = STATE_WAITING;
		}

		if (closed) {
			_cmd = CMD_STOP;
			_state = STATE_WAITING;
		}

		if (_stepCount >= _steps.value) {
			
			_cmd = CMD_STOP;
			_state = STATE_WAITING;
		}
		else {			
			if (!closing) {
				Timer1.initialize(_chunkDelay.value * 100); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
				Timer1.attachInterrupt(closeValveLittleBit);
				closing = true;
			}
		}
		break;
	case STATE_ERROR:
		_stepCount = 0;
		_chunkStep = 0;
		if (_cmd == CMD_RESET) {
			_errors = 0;
			_state = STATE_WAITING;
			_cmd = 0;
		}

		break;
	case STATE_SAVE_SETTINGS:
		if (!_steps.Save()) {
			_errors = _errors | 0x4;
			_state = STATE_ERROR;
			_cmd = 0;
			break;
		}

		if (!_chunkLength.Save()) {
			_errors = _errors | 0x10;
			_state = STATE_ERROR;
			_cmd = 0;
			break;
		}
		if (!_chunkDelay.Save()) {
			_errors = _errors | 0x20;
			_state = STATE_ERROR;
			_cmd = 0;
			break;
		}
		_cmd = 0;
		_state = STATE_WAITING;
		break;
	default: break;
	}
}

bool pulse = false;

void openValveLittleBit() {

		digitalWrite(DIR, HIGH);
		if (_state != STATE_OPENING) {
			Timer1.detachInterrupt();
			opening = false;
		}
		if (!pulse) {
			digitalWrite(STEP, HIGH);
			_chunkStep++;
			if (_chunkStep >= _chunkLength.value) {
				_stepCount++;
				_chunkStep = 0;
			}
		}
		else {
			digitalWrite(STEP, LOW);
		}
		pulse = !pulse;

}

void closeValveLittleBit() {
		digitalWrite(DIR, LOW);
		if (_state != STATE_CLOSING) {
			Timer1.detachInterrupt();
			closing = false;
		}
		if (!pulse) {
			digitalWrite(STEP, HIGH);
			_chunkStep++;
			if (_chunkStep >= _chunkLength.value) {
				_stepCount++;
				_chunkStep = 0;
			}
		}
		else {
			digitalWrite(STEP, LOW);
		}
		pulse = !pulse;


}

void modbusUpdate() {
	short prevSteps = 0;
	short prevChunkLength = 0;
	short prevChunkDelay = 0;

	modbus_update(); // запуск обмена по Modbus

	if (holdingRegs[steps] < MAX_STEPS) {
		if (holdingRegs[steps] != prevSteps)
		{
			if (holdingRegs[steps] > 0)
				_steps.value = holdingRegs[steps];
		}
		else {
			holdingRegs[steps] = _steps.value;
		}
		prevSteps = holdingRegs[steps];
	}
	else {
		_steps.value = 0;
		_errors = _errors | 0x02;
	}

	if (holdingRegs[chunkLength] < MAX_CHUNK_LENGTH)
	{

		if (holdingRegs[chunkLength] != prevChunkLength)
		{
			if (holdingRegs[chunkLength] > 0)
				_chunkLength.value = holdingRegs[chunkLength];
		}
		else {
			holdingRegs[chunkLength] = _chunkLength.value;
		}
		prevChunkLength = holdingRegs[chunkLength];
	}
	else {
		_chunkLength.value = 0;
		_errors = _errors | 0x02;
	}

	if (holdingRegs[chunkDelay] < MAX_CHUNK_DELAY)
	{

		if (holdingRegs[chunkDelay] != prevChunkDelay)
		{
			if (holdingRegs[chunkDelay] > 0)
				_chunkDelay.value = holdingRegs[chunkDelay];
		}
		else {
			holdingRegs[chunkDelay] = _chunkDelay.value;
		}
		prevChunkDelay = holdingRegs[chunkDelay];
	}
	else {
		_chunkDelay.value = 0;
		_errors = _errors | 0x02;
	}

	if (holdingRegs[cmd] != 0) {
		_cmd = holdingRegs[cmd];
		holdingRegs[cmd] = 0;
	}
	holdingRegs[errors] = _errors;
	holdingRegs[state] = _state;

}

void updateKeys() {
	//Концевики нормально замкнутые
	opened = digitalRead(OPENED);
	closed = digitalRead(CLOSED);


	if (opened && closed) {
		_errors = _errors | 0x01; //Возможно обрыв
	}
}

