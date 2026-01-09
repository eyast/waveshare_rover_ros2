/*
 * Storage Module
 * 
 * Abstracts access to the onboard storage for configurations.
 * 
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <Preferences.h>

void preferences_init();

void calib_set_flag(bool enabled);
bool calib_get_flag();

void streamboot_set_flag(bool enabled);
bool streamboot_get_flag();

#endif