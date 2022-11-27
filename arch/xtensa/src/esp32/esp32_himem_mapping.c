/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem_mapping.c
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

#include <nuttx/list.h>
#include <nuttx/mm/mm.h>
#include <nuttx/himem/himem.h>
#include <stdio.h>
#include "sched/sched.h"
#include "esp32_spiram.h"
#include "esp32_himem.h"
#include "hardware/esp32_soc.h"

#define RESERVE_WRITEBACK_CACHE_BLOCK   (4)
#define HIMEM_APP_VIRT_BASE     \
    (SOC_EXTRAM_DATA_LOW + (CACHE_BLOCKSIZE * RESERVE_WRITEBACK_CACHE_BLOCK))
#define HIMEM_APP_VIRT_BLOCK_START \
    ((HIMEM_APP_VIRT_BASE - SOC_EXTRAM_DATA_LOW) / CACHE_BLOCKSIZE)
#define HIMEM_APP_PHYS_BLOCK_START  (PHYS_HIMEM_BLOCKSTART)

#define HIMEM_APP_VIRT_SIZE_MAX     (2 * 1024 * 1024) // MAX Map 2MB
#define HIMEM_APP_RESERVE_BLOCK     (128 - SPIRAM_BANKSWITCH_RESERVE - 1)
#define HIMEM_APP_SIZE_MASK (CACHE_BLOCKSIZE - 1)
#define HIMEM_APP_SIZE_ALIGN(size)  ((size + HIMEM_APP_SIZE_MASK) & ~HIMEM_APP_SIZE_MASK)

static sem_t s_app_map_sem = SEM_INITIALIZER(1);

struct esp_himem_app
{
    struct list_node node;
    struct esp_himem_par param;
    void *id;
};

static struct list_node g_himem_app_list =
    LIST_INITIAL_VALUE(g_himem_app_list);

static void set_bank(int virt_bank, int phys_bank, int ct)
{
  int r;

  r = cache_sram_mmu_set(0, 0, SOC_EXTRAM_DATA_LOW + CACHE_BLOCKSIZE *
                         virt_bank, phys_bank * CACHE_BLOCKSIZE, 32, ct);
  DEBUGASSERT(r == 0);
  r = cache_sram_mmu_set(1, 0, SOC_EXTRAM_DATA_LOW + CACHE_BLOCKSIZE *
                         virt_bank, phys_bank * CACHE_BLOCKSIZE, 32, ct);
  DEBUGASSERT(r == 0);

  UNUSED(r);
}

static struct esp_himem_app *esp_himem_app_search_by_id(void* id)
{
    struct esp_himem_app *handle;
    
    list_for_every_entry(&g_himem_app_list, handle, 
                         struct esp_himem_app, node) {
        if (handle->id == id)
            return handle;
    }

    return NULL;
}

void esp_himem_app_map(struct esp_himem_par *param)
{
    esp_himem_rangedata_t *range;
    esp_himem_handle_t handle;
    int i;

    if (!param || !param->range)
        return;

    range = param->range;
    handle = param->handle;
    esp_spiram_writeback_cache();
    for (i = 0; i < range->block_ct; i++)
    {
        set_bank(HIMEM_APP_VIRT_BLOCK_START + i,
                 handle->block[i] + HIMEM_APP_PHYS_BLOCK_START, 1);
    }

    return;
}

void esp_himem_app_unmap(struct esp_himem_par *param)
{
    esp_himem_rangedata_t *range;
    int i;

    if (!param || !param->range)
        return;

    range = param->range;
    esp_spiram_writeback_cache();
    for (i = 0; i < range->block_ct; i++)
    {
        set_bank(HIMEM_APP_VIRT_BLOCK_START + i,
                 RESERVE_WRITEBACK_CACHE_BLOCK + i, 1);
    }

    return;
}

uint8_t *esp_himem_app_map_create(void *id, size_t size)
{
    struct esp_himem_app *appHandle;
    struct esp_himem_app *findHandle;
    struct esp_himem_par *param;
    esp_himem_rangedata_t *r;
    int ret;
    irqstate_t flags;

    size = HIMEM_APP_SIZE_ALIGN(size);
    
    if (size > HIMEM_APP_VIRT_SIZE_MAX)
        return NULL;
    
    appHandle = kmm_malloc(sizeof(struct esp_himem_app));
    if (!appHandle)
        return NULL;
    
    memset(appHandle, 0x00, sizeof(struct esp_himem_app));
    list_initialize(&appHandle->node);
    param = &appHandle->param;

    param->memfree = size;
    ret = esp_himem_alloc(param->memfree, &(param->handle));
    if (ret < 0)
        goto Err1;
    
    r = kmm_malloc(sizeof(esp_himem_rangedata_t) * 1);
    if (!r) {
        ret = -ENOMEM;
        goto Err2;
    }

    flags = enter_critical_section();

    findHandle = esp_himem_app_search_by_id(id);
    if (findHandle) {
        leave_critical_section(flags);
        return NULL;
    }

    r->block_ct = size / CACHE_BLOCKSIZE;
    r->block_start = 0;

    param->range = r;
    appHandle->id = id;

    esp_himem_app_map(param);
    memset((void *)HIMEM_APP_VIRT_BASE, 0x00, size);
    esp_himem_app_unmap(param);
    
    list_add_tail(&g_himem_app_list, &appHandle->node);

    leave_critical_section(flags);
    
    return (uint8_t *)HIMEM_APP_VIRT_BASE;
 Err2:
    esp_himem_free(param->handle);
 Err1:
    kmm_free(appHandle);

    return NULL;
}

int esp_himem_app_map_append(void *id, size_t size)
{
    struct esp_himem_app *appHandle;
    struct esp_himem_par *param;
    int ret;

    size = HIMEM_APP_SIZE_ALIGN(size);
    if (size > HIMEM_APP_VIRT_SIZE_MAX)
        return -EINVAL;
    
    appHandle = esp_himem_app_search_by_id(id);
    if (!appHandle)
        return -ENODEV;
    
    param = &appHandle->param;
    if (size <= param->memfree)
        return -EINVAL;
    
    ret = esp_himem_realloc(size, param->handle, &(param->handle));
    if (ret == OK) {
        param->memfree = size;
        param->range->block_ct = size / CACHE_BLOCKSIZE;
    }
    
    return ret;
}

void esp_himem_app_map_destroy(void *id)
{
    irqstate_t flags;
    struct tcb_s *rtcb;
    struct esp_himem_app *appHandle;
    struct esp_himem_par *param;

    flags = enter_critical_section();
    
    appHandle = esp_himem_app_search_by_id(id);
    if (!appHandle) {
        leave_critical_section(flags);
        return;
    }
    
    rtcb = this_task();
    esp_himem_app_unmap(rtcb->himem_param);
    rtcb->himem_param = NULL;

    param = &appHandle->param;
    list_delete(&appHandle->node);
    
    leave_critical_section(flags);
        
    kmm_free(param->range);     
    esp_himem_free(param->handle);
    kmm_free(appHandle);
}

static int esp_himem_app_map_control(void *id, bool isMap)
{
    irqstate_t flags;
    struct esp_himem_app *appHandle;
    struct tcb_s *rtcb;
    int ret = 0;

    flags = enter_critical_section();
    rtcb = this_task();
    if ((isMap && rtcb->himem_param) ||
        (!isMap && !rtcb->himem_param)){
        leave_critical_section(flags);
        return 0;
    }

    appHandle = esp_himem_app_search_by_id(id);
    if (appHandle) {
        if (isMap) {
            rtcb->himem_param = &appHandle->param;          
            esp_himem_app_map(rtcb->himem_param);
        } else {
            esp_himem_app_unmap(rtcb->himem_param);
            rtcb->himem_param = NULL;
        }
    } else {
        ret = -ENODEV;
    }

    leave_critical_section(flags);

    return ret;
}

int esp_himem_app_map_start(void *id)
{
    return esp_himem_app_map_control(id, true);
}

int esp_himem_app_map_stop(void *id)
{
    return esp_himem_app_map_control(id, false);
}

static ssize_t esp_himem_app_pread_pwrite(int is_write, void *id, void *buf,
                                          size_t count, void *app_ptr)
{
    struct esp_himem_app *appHandle;
    esp_himem_handle_t handle;
    int start_virt_bank;
    int start_phys_bank;
    uint32_t offset;
    size_t cp_size;
    void *addr;
    int i;

    appHandle = esp_himem_app_search_by_id(id);
    if (!appHandle) {
        return -EINVAL;
    }

    handle = appHandle->param.handle;
    if (!handle) {
        return -EINVAL;
    }

    start_virt_bank =
        ((uint32_t)app_ptr - HIMEM_APP_VIRT_BASE) / CACHE_BLOCKSIZE;
    if (start_virt_bank < 0) {
        return -EFAULT;
    }
    if (handle->block_ct < start_virt_bank) {
        return -EFAULT;
    }

    start_phys_bank = handle->block[start_virt_bank];
    offset = ((uint32_t)app_ptr - HIMEM_APP_VIRT_BASE) % CACHE_BLOCKSIZE;
    addr = (void *)(HIMEM_APP_RESERVE_BLOCK *
                    CACHE_BLOCKSIZE + SOC_EXTRAM_DATA_LOW);
    addr += offset;
    for (i=0; i<count; ) {
        if (count < CACHE_BLOCKSIZE - offset)
            cp_size = count;
        else
            cp_size = CACHE_BLOCKSIZE - offset;
        nxsem_wait(&s_app_map_sem);
        set_bank(HIMEM_APP_RESERVE_BLOCK,
                 start_phys_bank + HIMEM_APP_PHYS_BLOCK_START, 1);
        
        if (is_write){
            memcpy(addr, buf, cp_size);
            esp_spiram_writeback_cache();
        } else {
            memcpy(buf, addr, cp_size);
        }
        addr = (void *)(HIMEM_APP_RESERVE_BLOCK *
                        CACHE_BLOCKSIZE + SOC_EXTRAM_DATA_LOW);
        buf += cp_size;
        i += cp_size;
        offset = 0;
        start_virt_bank++;
        if (i < count) {
            if (handle->block_ct < start_virt_bank) {
                nxsem_post(&s_app_map_sem);
                return -EFAULT;
            }
            start_phys_bank = handle->block[start_virt_bank];
        }

        nxsem_post(&s_app_map_sem);
    }

    return count;
}

ssize_t esp_himem_app_pread(void *id, void *buf, size_t count, void *app_ptr)
{
    return esp_himem_app_pread_pwrite(0, id, buf, count, app_ptr);
}

ssize_t esp_himem_app_pwrite(void *id, const void *buf,
                             size_t count, void *app_ptr)
{
    return esp_himem_app_pread_pwrite(1, id, (void *)buf, count, app_ptr);
}

size_t esp_himem_mapping_reserved_area_size(void)
{
    /* only one block size is supported  */
    return (1 * CACHE_BLOCKSIZE);
}
