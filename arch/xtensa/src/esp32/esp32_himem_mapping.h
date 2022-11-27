/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem_mapping.h
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

#ifndef __INCLUDE_NUTTX_HIMEM_MAPPING_H
#define __INCLUDE_NUTTX_HIMEM_MAPPING_H

#include <nuttx/config.h>
#include <nuttx/himem/himem.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
void esp_himem_app_map(struct esp_himem_par *param);
void esp_himem_app_unmap(struct esp_himem_par *param);
size_t esp_himem_mapping_reserved_area_size(void);

#endif /* __INCLUDE_NUTTX_HIMEM_MAPPING_H */
