/*
 * version.h
 *
 *  Created on: 2022年1月11日
 *      Author: chi
 */

#ifndef VERSION_H_
#define VERSION_H_

static const char rcVersionMagic[] = "1.0.0";

const char* rc_get_version() { return rcVersionMagic; }

#endif /* VERSION_H_ */
