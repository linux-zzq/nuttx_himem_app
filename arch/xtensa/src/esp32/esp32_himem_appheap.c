/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem_appheap.c
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

#include <nuttx/mm/mm.h>
#include <nuttx/himem/himem.h>
#include "esp32_spiram.h"
#include "hardware/esp32_soc.h"
#include "esp32_himem.h"
#ifdef CONFIG_ESP32_HIMEM_APP_HEAP
#include "esp32_himem_appheap.h"
#endif

#ifdef CONFIG_ESP32_HIMEM_MAPPING
#include "esp32_himem_mapping.h"
#endif

#ifdef CONFIG_ESP32_HIMEM_APP_HEAP_BLOCKS
#define ESP32_APP_HEAP_RESERVE  (CONFIG_ESP32_HIMEM_APP_HEAP_BLOCKS)
#else
#define ESP32_APP_HEAP_RESERVE  (8)
#endif

#ifdef CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
#  define SPIRAM_BANKSWITCH_RESERVE CONFIG_SPIRAM_BANKSWITCH_RESERVE
#else
#  define SPIRAM_BANKSWITCH_RESERVE 0
#endif

static struct mm_heap_s *g_app_heap;

void esp_app_heap_add_region(void)
{
    void *start = (void *)(SOC_EXTRAM_DATA_LOW +
                           (128 - ESP32_APP_HEAP_RESERVE) * CACHE_BLOCKSIZE);

#  ifdef CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
    start -= esp_himem_reserved_area_size();
#  endif

#  ifdef CONFIG_ESP32_HIMEM_MAPPING
    start -= esp_himem_mapping_reserved_area_size();
#endif

    g_app_heap = mm_initialize("app",
                               start,
                               ESP32_APP_HEAP_RESERVE * CACHE_BLOCKSIZE);
}

void *esp_app_malloc(size_t size)
{
    return mm_malloc(g_app_heap, size);
}

void *esp_app_realloc(void *oldmem, size_t size)
{
    return mm_realloc(g_app_heap, oldmem, size);
}

void esp_app_free(void *mem)
{
    mm_free(g_app_heap, mem);
}

size_t esp_app_heap_reserved_area_size(void)
{
    return CACHE_BLOCKSIZE * ESP32_APP_HEAP_RESERVE;
}
