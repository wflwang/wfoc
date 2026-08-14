/*
 * conf_general.h - Global Configuration Manager
 *
 * Owns the live mc_configuration_t and app_configuration_t instances and
 * provides load/save accessors. Persistence is stubbed for now (flash
 * layout not yet wired); the stubs return false so callers fall back to
 * the defaults populated by conf_default.c.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef CONF_GENERAL_H
#define CONF_GENERAL_H

#include "conf/datatypes.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Global Configuration Instances                                            */
/* ======================================================================== */
/* Single live copies used by the whole firmware. */
extern mc_configuration_t  mc_conf;
extern app_configuration_t app_conf;

/* ======================================================================== */
/* Default Value Population                                                  */
/* ======================================================================== */
/* Fill *conf with board-derived defaults. Defined in conf_default.c. */
void conf_default_mc(mc_configuration_t *conf);
void conf_default_app(app_configuration_t *conf);

/* ======================================================================== */
/* Lifecycle                                                                 */
/* ======================================================================== */
/* Load configuration: defaults first, then attempt a flash read (stubbed).
 * Call once at startup, before any motor or app subsystem. */
void conf_general_init(void);

/* Persist current configuration to flash (stubbed). */
void conf_general_save(void);

/* ======================================================================== */
/* Per-Structure Accessors                                                   */
/* ======================================================================== */
/* Read from flash into *conf. Returns true on success, false if no stored
 * configuration exists (caller should use defaults). Stubbed. */
bool conf_general_read_mc_conf(mc_configuration_t *conf);
bool conf_general_read_app_conf(app_configuration_t *conf);

/* Write *conf to flash and update the live instance. Returns true on
 * success. Stubbed (no-ops the flash write, but mirrors into the live copy). */
bool conf_general_write_mc_conf(const mc_configuration_t *conf);
bool conf_general_write_app_conf(const app_configuration_t *conf);

#ifdef __cplusplus
}
#endif

#endif /* CONF_GENERAL_H */
