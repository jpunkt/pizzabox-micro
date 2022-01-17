#ifndef STATES_H
#define STATES_H

/*-------- State Definitions --------*/

/**
 * @brief STATE 0. Power-on self test
 * 
 */
void state_post();

/**
 * @brief STATE 1. Zero the scrolls and check if they are present
 * 
 */
void state_zero();

/**
 * @brief STATE 2. Initialize sensor callbacks
 * 
 */
void state_init_callbacks();

/**
 * @brief STATE 3. Wait for Raspberry Pi HELO and serial handshake
 * 
 * set RPI_HELO2 pin high
 * wait for RPI_HELO1 pin to go low (answer from pi)
 */
void state_wait_serial();

/**
 * @brief MAIN STATE. Wait for serial commands and execute them
 * 
 */
void state_serial_com();

/**
 * @brief ERROR STATE. There is no recovery except power cycling.
 * 
 */
void state_error();


/*-------- Transitions --------*/

/**
 * @brief Transition from STATE 0 to STATE 1.
 * 
 * @return true 
 * @return false 
 */
bool transition_post_zero();

/**
 * @brief Transition from STATE 1 to STATE 2
 * 
 * @return true 
 * @return false 
 */
bool transition_zero_init();

/**
 * @brief Transition from STATE 2 to STATE 3
 * 
 * @return true 
 * @return false 
 */
bool transition_init_wait();

/**
 * @brief Transition from STATE 3 to MAIN STATE
 * 
 * @return true 
 * @return false 
 */
bool transition_wait_sercom();

#endif