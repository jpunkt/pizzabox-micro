#include <Arduino.h>
#include "Commands.h"

/**
 * @brief Read one byte from Serial and cast it to a Command
 * 
 * @return Command 
 */
Command read_command(HardwareSerial &serial);

/**
 * @brief Wait for the number of bytes to be available on Serial
 * 
 * @param num_bytes 
 * @param timeout_ms 
 */
void wait_for_bytes(HardwareSerial &serial, int num_bytes, unsigned long timeout_ms);

/**
 * @brief Read one byte
 * 
 * @return int8_t 
 */
int8_t read_i8(HardwareSerial &serial);

/**
 * @brief Read two bytes and convert to signed 16bit integer
 * 
 * @return int16_t 
 */
int16_t read_i16(HardwareSerial &serial);

/**
 * @brief Read four bytes and convert to signed 32bit integer
 * 
 * @return int32_t 
 */
int32_t read_i32(HardwareSerial &serial);

/**
 * @brief Write one byte corresponding to a Command
 * 
 * @param cmd 
 */
void write_command(HardwareSerial &serial, Command cmd);

/**
 * @brief Write a signed 8bit integer
 * 
 * @param num 
 */
void write_i8(HardwareSerial &serial, int8_t num);

/**
 * @brief Write a signed 16bit integer
 * 
 * @param num 
 */
void write_i16(HardwareSerial &serial, int16_t num);

/**
 * @brief Write a signed 32bit integer
 * 
 * @param num 
 */
void write_i32(HardwareSerial &serial, int32_t num);
