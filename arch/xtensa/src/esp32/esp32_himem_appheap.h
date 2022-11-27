/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem_appheap.h
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

#ifndef __INCLUDE_NUTTX_HIMEM_APPHEAP_H
#define __INCLUDE_NUTTX_HIMEM_APPHEAP_H

#include <stddef.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
void esp_app_heap_add_region(void);
size_t esp_app_heap_reserved_area_size(void);

#endif /* __INCLUDE_NUTTX_HIMEM_APPHEAP_H */
