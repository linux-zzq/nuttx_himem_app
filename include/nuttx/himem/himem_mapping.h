/****************************************************************************
 * include/nuttx/himem/himem_mapping.h
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

#ifndef __INCLUDE_NUTTX_HIMEM_HIMEM_MAPPING_H
#define __INCLUDE_NUTTX_HIMEM_HIMEM_MAPPING_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int esp_himem_app_map_start(void *id);
int esp_himem_app_map_stop(void *id);
uint8_t *esp_himem_app_map_create(void *id, size_t size);
int esp_himem_app_map_append(void *id, size_t size);
void esp_himem_app_map_destroy(void *id);
ssize_t esp_himem_app_pread(void *id, void *buf, size_t count, void *app_ptr);
ssize_t esp_himem_app_pwrite(void *id, const void *buf,
                             size_t count, void *app_ptr);

#endif /* __INCLUDE_NUTTX_HIMEM_HIMEM_MAPPING_H */

