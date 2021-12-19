#include "Serial_Comm.h"

/**
 * @brief Read one byte from Serial and cast it to a Command
 * 
 * @return Command 
 */
Command read_command(HardwareSerial &serial) {
  return (Command) serial.read();
}

/**
 * @brief Wait for the number of bytes to be available on Serial
 * 
 * @param num_bytes 
 * @param timeout_ms 
 */
void wait_for_bytes(HardwareSerial &serial, int num_bytes, unsigned long timeout_ms) {
  uint32_t startTime = millis();
	// Wait for incoming bytes or exit if timeout
	while ((serial.available() < num_bytes) && (millis() - startTime < timeout_ms)){
    // NOOP.
  }
}

void read_signed_bytes(HardwareSerial &serial, int8_t* buffer, size_t n)
{
	size_t i = 0;
	int c;
	while (i < n)
	{
		c = serial.read();
		if (c < 0) break;
		*buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
		i++;
	}
}

/**
 * @brief Read one byte
 * 
 * @return int8_t 
 */
int8_t read_i8(HardwareSerial &serial) {
  wait_for_bytes(serial, 1, 500);
  return (int8_t) serial.read();
}

/**
 * @brief Read two bytes and convert to signed 16bit integer
 * 
 * @return int16_t 
 */
int16_t read_i16(HardwareSerial &serial) {
  int8_t buffer[2];
	wait_for_bytes(serial, 2, 500);
	read_signed_bytes(serial, buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

/**
 * @brief Read four bytes and convert to signed 32bit integer
 * 
 * @return int32_t 
 */
int32_t read_i32(HardwareSerial &serial) {
  int8_t buffer[4];
	wait_for_bytes(serial, 4, 200); // Wait for 4 bytes with a timeout of 200 ms
	read_signed_bytes(serial, buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);

}

/**
 * @brief Write one byte corresponding to a Command
 * 
 * @param cmd 
 */
void write_command(HardwareSerial &serial, Command cmd) {
  uint8_t* c = (uint8_t*) &cmd;
  serial.write(c, sizeof(uint8_t));
}

/**
 * @brief Write a signed 8bit integer
 * 
 * @param num 
 */
void write_i8(HardwareSerial &serial, int8_t num) {
  serial.write(num);
}

/**
 * @brief Write a signed 16bit integer
 * 
 * @param num 
 */
void write_i16(HardwareSerial &serial, int16_t num) {
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

/**
 * @brief Write a signed 32bit integer
 * 
 * @param num 
 */
void write_i32(HardwareSerial &serial, int32_t num) {
  int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
