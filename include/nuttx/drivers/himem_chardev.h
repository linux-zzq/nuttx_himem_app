/****************************************************************************
 * include/nuttx/drivers/himem_chardev.h
 * 
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
****************************************************************************/

#ifndef __INCLUDE_NUTTX_DRIVER_HIMEM_CHARDEV_H
#define __INCLUDE_NUTTX_DRIVER_HIMEM_CHARDEV_H

#include <nuttx/list.h>
#include <nuttx/himem/himem.h>

#ifdef __cplusplus
extern "C" {
#endif

int himem_chardev_init(void);
int himem_chardev_exit(void);
int himem_chardev_register(char *name, size_t size);
int himem_chardev_unregister(char *name);

#ifdef __cplusplus
}
#endif

#endif
