/**
 * @file  STM32_BMPX_conf.h
 * @brief Configuration overridable STM32_BMPX — surcharger via -D ou
 *        en définissant AVANT d'inclure STM32_BMPX.h
 */
#ifndef STM32_BMPX_CONF_H
#define STM32_BMPX_CONF_H

/** @brief Activer les messages de debug (table strings en flash)
 *  @note  Décommenter pour activer, ou passer -DBMPX_DEBUG_ENABLE via le Makefile/CubeIDE.
 *         Désactivé par défaut pour minimiser l'empreinte flash en production.
 */
/* #define BMPX_DEBUG_ENABLE */

/** @brief Timeout I2C par défaut en ms */
#ifndef BMPX_DEFAULT_TIMEOUT_MS
#define BMPX_DEFAULT_TIMEOUT_MS   100U
#endif

/** @brief Timeout d'attente mesure en ms (datasheet BMP180 §3.4: max 25.5ms) */
#ifndef BMPX_WAIT_TIMEOUT_MS
#define BMPX_WAIT_TIMEOUT_MS      50U
#endif

/** @brief Seuil d'erreurs consécutives avant signalement critique */
#ifndef BMPX_MAX_CONSECUTIVE_ERRORS
#define BMPX_MAX_CONSECUTIVE_ERRORS  3U
#endif

/** @brief Intervalle de déclenchement TriggerEvery par défaut (ms) */
#ifndef BMPX_DEFAULT_SAMPLE_INTERVAL_MS
#define BMPX_DEFAULT_SAMPLE_INTERVAL_MS  2000U
#endif

#endif /* STM32_BMPX_CONF_H */
