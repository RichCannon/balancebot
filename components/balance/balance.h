#pragma once

// Initialize the robot hardware (SPI bus, MPU sensor, motors).
// Call once before balance_start().
void balance_init(void);

// Start the balance control task.
void balance_start(void);
