/**
  ******************************************************************************
  * @file    STM32_BMPX.h
  * @brief   Driver STM32 HAL pour capteurs BMP085/BMP180/BMP280 (Bosch)
  * @author  STM32_LIB_STYLE_GUIDE
  * @version 0.9.0
  * @date    2026-02-11
  * @copyright Libre sous licence MIT.
  ******************************************************************************
  * @attention
  *
  * Capteurs de pression barométrique + température (I2C)
  * Support BMP085, BMP180, BMP280 avec auto-détection
  *
  ******************************************************************************
  */

#ifndef STM32_BMPX_H // Vérifie si le fichier d'en-tête n'a pas déjà été inclus
#define STM32_BMPX_H // Définit une macro pour éviter les inclusions multiples

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ============================================================================ */

#include "main.h"       ///< HAL multi-famille STM32 via CubeMX (contient stm32l4xx_hal.h ou stm32f4xx_hal.h selon projet)
#include "STM32_BMPX_conf.h" ///< Configuration surchargeable de la librairie
#include <stdbool.h>    ///< Type bool, true, false

/* ============================================================================
 * Exported constants
 * ============================================================================ */

/** @defgroup BMPX_Version Library version
  * @{
  */
#define BMPX_LIB_VERSION_MAJOR 0
#define BMPX_LIB_VERSION_MINOR 9
#define BMPX_LIB_VERSION_PATCH 0

/* Adressage I2C profil DUAL_ADDRESS */
#define BMP_I2C_ADDR_PRIMARY_7B   0x77  ///< Adresse I2C 7-bit primaire   (SDO → GND, défaut)
#define BMP_I2C_ADDR_SECONDARY_7B 0x76  ///< Adresse I2C 7-bit secondaire (SDO → VDD)


/* ============================================================================
 * Exported types
 * ============================================================================ */

/**
  * @{
  */
#define BMP_ULTRALOWPOWER 0  ///< Mode ultra faible consommation (OSS=0, 1 sample)
#define BMP_STANDARD 1       ///< Mode standard (défaut, OSS=1, 1 sample)
#define BMP_HIGHRES 2        ///< Mode haute résolution (OSS=2, 2 samples)
#define BMP_ULTRAHIGHRES 3   ///< Mode ultra haute résolution (OSS=3, 4 samples)

#define BMP_DEFAULT_MODE BMP_STANDARD    ///< Mode de démarrage par défaut


/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMP Status codes
 */
typedef enum
{
  BMP_OK = 0,                   //!< Opération réussie
  BMP_ERR_I2C,                  //!< Erreur de communication I2C (TX ou RX)
  BMP_ERR_CHIP_ID,              //!< ID de puce incorrect lu
  BMP_ERR_CAL_READ,             //!< Erreur lors de la lecture des données de calibration
  BMP_ERR_NULL_PTR,             //!< Pointeur NULL fourni en argument
  BMP_ERR_INVALID_PARAM,        //!< Paramètre invalide fourni (ex: mode, pression/altitude négative)
  BMP_ERR_BUSY,                 //!< Contexte occupé (mesure async déjà en cours / données non prêtes)
  BMP_ERR_TIMEOUT,              //!< Timeout HAL I2C
  BMP_ERR_MATH,                 //!< Erreur de calcul interne (ex: division par zéro)
  BMP_ERR_NOT_INITIALIZED,       //!< Handle non initialisé (Init non appelé)
  BMP_ERR_NOT_CONFIGURED         //!< Pré-requis HW/CubeMX absents
} BMP_Status;

/**
 * @brief BMP Sensor type
 */
typedef enum
{
  BMP_SENSOR_UNKNOWN = 0,
  BMP_SENSOR_BMP085_180 = 1,
  BMP_SENSOR_BMP280 = 2
} BMP_SensorType;

/**
 * @brief BMP Sensor instance structure
 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;  //!< Pointeur vers le handle I2C HAL (standard CubeMX)
  uint8_t i2cAddr7bit;          //!< Adresse I2C 7 bits
  uint8_t oversampling;         //!< Mode d'oversampling configuré

  BMP_SensorType sensorType;  //!< Type de capteur (BMP085/180 ou BMP280)

  // Données de calibration lues depuis le capteur
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2;
  int16_t mb, mc, md;

  // --- BMP280 calibration (NVM) ---
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  int32_t t_fine;

  volatile BMP_Status last_error;         ///< Dernier code d'erreur (diagnostic sans printf)
  bool                initialized;        ///< true après BMP_InitWithAddress() réussi
  volatile uint8_t    consecutive_errors; ///< Compteur d'erreurs consécutives (reset à 0 si OK)
  uint32_t            last_hal_error;     ///< Code HAL_I2C_GetError() lors de la dernière erreur
} BMP_Handle_t; /**< sizeof(BMP_Handle_t) ≈ 40 octets — handle principal du capteur BMP */

// --- BMP280 : configuration optionnelle ---
// Notes :
// - Cette API n'est valide que si bmp->sensorType == BMP_SENSOR_BMP280.
// - Les champs utilisent les valeurs brutes du datasheet BMP280.

// Oversampling (osrs_t / osrs_p)
#define BMP280_OSRS_SKIPPED 0u
#define BMP280_OSRS_X1      1u
#define BMP280_OSRS_X2      2u
#define BMP280_OSRS_X4      3u
#define BMP280_OSRS_X8      4u
#define BMP280_OSRS_X16     5u

// Filter (IIR)
#define BMP280_FILTER_OFF 0u
#define BMP280_FILTER_2   1u
#define BMP280_FILTER_4   2u
#define BMP280_FILTER_8   3u
#define BMP280_FILTER_16  4u

// Standby time (t_sb)
#define BMP280_STANDBY_0P5_MS   0u
#define BMP280_STANDBY_62P5_MS  1u
#define BMP280_STANDBY_125_MS   2u
#define BMP280_STANDBY_250_MS   3u
#define BMP280_STANDBY_500_MS   4u
#define BMP280_STANDBY_1000_MS  5u
#define BMP280_STANDBY_2000_MS  6u
#define BMP280_STANDBY_4000_MS  7u

// Mode
#define BMP280_MODE_SLEEP   0u
#define BMP280_MODE_FORCED  1u
#define BMP280_MODE_NORMAL  3u

typedef struct
{
  uint8_t osrs_t;   //!< Oversampling température (0..5)
  uint8_t osrs_p;   //!< Oversampling pression (0..5)
  uint8_t filter;   //!< Filtre IIR (0..4)
  uint8_t standby;  //!< Standby time t_sb (0..7)
  uint8_t mode;     //!< Mode (0=sleep, 1=forced, 3=normal)
} BMP280_Config;

/* ============================================================================
 * Exported functions prototypes
 * ============================================================================ */

/**
 * @brief  Convertit un code BMP_Status en chaîne de caractères.
 * @note   Retourne "" si BMPX_DEBUG_ENABLE n'est pas défini (body conditionnel dans le .c).
 * @param  status  Code retour à convertir
 * @retval Pointeur vers une chaîne statique constante (jamais NULL)
 */
const char *BMP_StatusToString(BMP_Status status);

/**
 * @brief  Convertit un type de capteur en chaîne de caractères.
 * @note   Retourne "" si BMPX_DEBUG_ENABLE n'est pas défini (body conditionnel dans le .c).
 * @param  sensor_type  Type de capteur
 * @retval Pointeur vers une chaîne statique constante (jamais NULL)
 */
const char *BMP_SensorTypeToString(BMP_SensorType sensor_type);

/**
 * @brief  Détecte automatiquement l'adresse I2C du capteur (scan 0x76 et 0x77).
 * @note   Lit le registre chip_id (0xD0) : accepte 0x55 (BMP085/180) et 0x58 (BMP280).
 * @param  hi2c       Handle I2C HAL (non NULL)
 * @param  addr7b_out Pointeur pour recevoir l'adresse 7-bit détectée (non NULL)
 * @retval BMP_OK           Détection réussie — *addr7b_out rempli
 * @retval BMP_ERR_NULL_PTR Pointeur NULL
 * @retval BMP_ERR_CHIP_ID  Aucun capteur détecté
 */
BMP_Status BMP_DetectAddress7b(I2C_HandleTypeDef *hi2c, uint8_t *addr7b_out);

/**
 * @brief  Initialise le capteur BMP avec une adresse I2C explicite.
 * @note   Détecte le type (0x55=BMP085/180, 0x58=BMP280), lit la calibration.
 *         Pour BMP280 : reset + calibration + configuration ctrl_meas/config.
 * @param  bmp              Handle BMP (non NULL)
 * @param  hi2c             Handle I2C HAL (non NULL)
 * @param  i2c_address_7bit Adresse 7-bit (BMP_I2C_ADDR_PRIMARY_7B ou SECONDARY_7B)
 * @param  mode             Mode oversampling (BMP_ULTRALOWPOWER..BMP_ULTRAHIGHRES)
 * @retval BMP_OK                  Succès
 * @retval BMP_ERR_NULL_PTR        Pointeur NULL
 * @retval BMP_ERR_INVALID_PARAM   Mode invalide
 * @retval BMP_ERR_CHIP_ID         chip_id non reconnu
 * @retval BMP_ERR_CAL_READ        Erreur lecture calibration
 * @retval BMP_ERR_I2C             Erreur bus I2C
 */
BMP_Status BMP_InitWithAddress(BMP_Handle_t *bmp, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_7bit, uint8_t mode);

/**
 * @brief  Initialise le capteur BMP avec auto-détection d'adresse.
 * @note   Appelle BMP_DetectAddress7b puis BMP_InitWithAddress.
 * @param  bmp   Handle BMP (non NULL)
 * @param  hi2c  Handle I2C HAL (non NULL)
 * @param  mode  Mode oversampling (BMP_ULTRALOWPOWER..BMP_ULTRAHIGHRES)
 * @retval BMP_OK         Succès
 * @retval BMP_ERR_CHIP_ID Aucun capteur détecté
 * @retval (voir BMP_InitWithAddress)
 */
BMP_Status BMP_InitAuto(BMP_Handle_t *bmp, I2C_HandleTypeDef *hi2c, uint8_t mode);

/** @brief Alias Init conf.au pattern commun — auto-détection + mode par défaut */
#define BMP_Init(bmp, hi2c_ptr) BMP_InitAuto((bmp), (hi2c_ptr), BMP_DEFAULT_MODE)

/**
 * @brief  Remet le handle à zéro (dissocie l'I2C, efface toutes les erreurs).
 * @note   Ne libère aucune mémoire. À appeler avant une ré-initialisation.
 * @param  bmp  Handle BMP (non NULL)
 * @retval BMP_OK / BMP_ERR_NULL_PTR
 */
BMP_Status BMP_DeInit(BMP_Handle_t *bmp);

/**
 * @brief  Configure manuellement l'adresse I2C du handle.
 * @param  bmp            Handle BMP (non NULL)
 * @param  i2c_address_7bit Adresse 7-bit (0x76 ou 0x77)
 * @retval BMP_OK / BMP_ERR_NULL_PTR / BMP_ERR_INVALID_PARAM
 */
BMP_Status BMP_SetAddress(BMP_Handle_t *bmp, uint8_t i2c_address_7bit);
BMP_Status BMP_UsePrimaryAddress(BMP_Handle_t *bmp);
BMP_Status BMP_UseSecondaryAddress(BMP_Handle_t *bmp);
uint8_t BMP_GetAddress(const BMP_Handle_t *bmp);

/**
 * @brief  Lit la température (bloquant).
 * @param  bmp       Handle BMP initialisé (non NULL)
 * @param  temp_degC Pointeur de résultat en °C (non NULL)
 * @retval BMP_OK / BMP_ERR_NOT_INITIALIZED / BMP_ERR_I2C / BMP_ERR_MATH
 */
BMP_Status BMP_ReadTemperature(BMP_Handle_t *bmp, float *temp_degC);

/**
 * @brief  Lit la pression (bloquant).
 * @param  bmp         Handle BMP initialisé (non NULL)
 * @param  pressure_Pa Pointeur de résultat en Pa (non NULL)
 * @retval BMP_OK / BMP_ERR_NOT_INITIALIZED / BMP_ERR_I2C / BMP_ERR_MATH
 */
BMP_Status BMP_ReadPressure(BMP_Handle_t *bmp, int32_t *pressure_Pa);

/**
 * @brief  Lit température et pression en une seule opération (bloquant).
 * @param  bmp         Handle BMP initialisé (non NULL)
 * @param  temp_degC   Pointeur température en °C (non NULL)
 * @param  pressure_Pa Pointeur pression en Pa (non NULL)
 * @retval BMP_OK / BMP_ERR_NOT_INITIALIZED / BMP_ERR_I2C / BMP_ERR_MATH
 */
BMP_Status BMP_ReadAll(BMP_Handle_t *bmp, float *temp_degC, int32_t *pressure_Pa);

/**
 * @brief  Calcule l'altitude depuis la pression au sol et la pression courante.
 * @param  pressure_Pa         Pression mesurée en Pa
 * @param  sealevelPressure_Pa Pression au niveau de la mer en Pa (ex: 101325.0f)
 * @param  altitude_m          Pointeur de résultat en mètres (non NULL)
 * @retval BMP_OK / BMP_ERR_NULL_PTR / BMP_ERR_INVALID_PARAM
 */
BMP_Status BMP_CalculateAltitude(int32_t pressure_Pa, float sealevelPressure_Pa, float *altitude_m);

/**
 * @brief  Calcule la pression au niveau de la mer depuis pression+altitude connue.
 * @param  pressure_Pa  Pression mesurée en Pa
 * @param  altitude_m   Altitude en mètres
 * @retval Pression niveau de la mer en Pa
 */
int32_t BMP_CalculateSeaLevelPressure(int32_t pressure_Pa, float altitude_m);

/* API BMP280 optionnelle */

/**
 * @brief  Configure le BMP280 (oversampling, filtre IIR, standby, mode).
 * @note   Valide uniquement si bmp->sensorType == BMP_SENSOR_BMP280.
 *         Met le capteur en Sleep avant écriture, puis restaure le mode.
 * @param  bmp  Handle BMP280 initialisé (non NULL)
 * @param  cfg  Configuration à appliquer (non NULL)
 * @retval BMP_OK / BMP_ERR_NULL_PTR / BMP_ERR_INVALID_PARAM / BMP_ERR_I2C
 */
BMP_Status BMP280_SetConfig(BMP_Handle_t *bmp, const BMP280_Config *cfg);

/**
 * @brief  Déclenche une mesure Forced et lit T+P (bloquant).
 * @note   Valide uniquement si bmp->sensorType == BMP_SENSOR_BMP280.
 * @param  bmp         Handle BMP280 initialisé (non NULL)
 * @param  temp_degC   Pointeur température en °C (non NULL)
 * @param  pressure_Pa Pointeur pression en Pa (non NULL)
 * @param  timeout_ms  Timeout attente mesure (0 = 100ms par défaut)
 * @retval BMP_OK / BMP_ERR_NOT_INITIALIZED / BMP_ERR_INVALID_PARAM / BMP_ERR_I2C / BMP_ERR_TIMEOUT
 */
BMP_Status BMP280_ReadAll_OneShot(BMP_Handle_t *bmp, float *temp_degC, int32_t *pressure_Pa, uint32_t timeout_ms);

/* ============================================================================
 * API ASYNCHRONE (IT uniquement) - Gain CPU lors de l'attente conversion
 * @note DMA non implémenté : bus I2C partagé multi-capteurs (arbitrage
 *       impossible) et trames courtes (BMP280=6B, BMP085/180=2+3B).
 *       IT suffit et cohabite avec les autres capteurs sur le même bus.
 * ============================================================================ */

/**
 * @brief Mode de transport I2C asynchrone
 * @note  POLLING est géré par l'API synchrone (BMP_ReadAll), pas ici.
 * @deprecated DMA supprimé — IT uniquement (bus I2C partagé).
 */
typedef enum {
    BMP_ASYNC_TRANSPORT_IT  = 0   /**< Mode interruption (seul mode supporté) */
} BMP_AsyncTransport;

/**
 * @brief BMP Async state machine
 */
typedef enum {
    BMP_ASYNC_IDLE = 0,        /**< Inactif */
    BMP_ASYNC_READ_TEMP_TX,    /**< Envoi commande lecture température */
    BMP_ASYNC_WAIT_TEMP,       /**< Attente conversion température */
    BMP_ASYNC_READ_TEMP_RX,    /**< Lecture données température */
    BMP_ASYNC_READ_PRESS_TX,   /**< Envoi commande lecture pression */
    BMP_ASYNC_WAIT_PRESS,      /**< Attente conversion pression */
    BMP_ASYNC_READ_PRESS_RX,   /**< Lecture données pression */
    BMP_ASYNC_DONE,            /**< Mesure complète */
    BMP_ASYNC_ERROR            /**< Erreur */
} BMP_AsyncState;

/**
 * @brief BMP données résultats
 */
typedef struct {
    float temperature;      /**< Température en °C */
    int32_t pressure;       /**< Pression en Pa */
} BMP_Data;

/**
 * @brief Callbacks utilisateur (contexte main loop via Process())
 */
typedef void (*BMP_Async_OnDataReadyCb)(void *user_ctx, const BMP_Data *data, BMP_Status status);
typedef void (*BMP_Async_OnErrorCb)(void *user_ctx, BMP_Status status);

/**
 * @brief Callbacks IRQ-safe (contexte interruption, ultra-court)
 */
typedef void (*BMP_Async_OnIrqDataReadyCb)(void *user_ctx);
typedef void (*BMP_Async_OnIrqErrorCb)(void *user_ctx);

/**
 * @brief Structure contexte asynchrone BMP
 */
typedef struct {
    BMP_Handle_t *bmp;
    
    volatile BMP_AsyncState state;
    volatile BMP_Status last_status;
    
    uint32_t conv_deadline_ms;       /**< Deadline attente conversion */
    uint32_t i2c_deadline_ms;        /**< Deadline timeout I2C */
    
    uint8_t temp_rx_buf[3];          /**< Buffer température brute */
    uint8_t press_rx_buf[3];         /**< Buffer pression brute */
    uint8_t bmp280_rx_buf[6];        /**< Buffer brut BMP280: P(3)+T(3) */
    BMP_Data data;                   /**< Résultats calculés */
    
    /* Flags et callbacks */
    volatile bool data_ready_flag;
    volatile bool error_flag;
    bool notify_data_pending;
    bool notify_error_pending;
    
    volatile uint8_t    async_busy;         ///< 1 = transfert IT en cours (set dans ReadAll_IT, clear dans Tick)
    uint32_t            last_hal_error;    ///< Code HAL_I2C_GetError() lors de la dernière erreur I2C
    uint32_t            sample_interval_ms; ///< Intervalle TriggerEvery (ms) — init via BMP_Async_Init()

    BMP_Async_OnDataReadyCb on_data_ready;
    BMP_Async_OnErrorCb on_error;
    BMP_Async_OnIrqDataReadyCb on_irq_data_ready;
    BMP_Async_OnIrqErrorCb on_irq_error;
    void *user_ctx;     ///< Contexte utilisateur (appelé depuis Process — main loop)
    void *irq_user_ctx; ///< Contexte IRQ-safe (appelé depuis callbacks HAL IT)
} BMP_Async;
typedef BMP_Async BMP_Async_t; //!< Alias compat

/* Fonctions API asynchrone */

/**
 * @brief Initialise le contexte asynchrone
 */
void BMP_Async_Init(BMP_Async *ctx, BMP_Handle_t *bmp);

/**
 * @brief Réinitialise la machine d'état async sans perdre les callbacks
 * @param ctx Contexte async (non NULL)
 * @note Préserve : bmp, hi2c, tous les callbacks, user_ctx.
 *       Remet : state=IDLE, flags=false, buffers=0, deadlines=0.
 *       Utile pour récupération d'erreur sans re-appeler Init+SetCallbacks.
 * @pre  ctx non NULL
 * @post ctx->state == BMP_ASYNC_IDLE
 * @post ctx->hbmp->async_busy == 0  (libère le guard polling)
 */
void BMP_Async_Reset(BMP_Async *ctx);

/**
 * @brief Configure les callbacks utilisateur (appel depuis Process)
 */
void BMP_Async_SetCallbacks(BMP_Async *ctx,
                            BMP_Async_OnDataReadyCb on_data_ready,
                            BMP_Async_OnErrorCb on_error,
                            void *user_ctx);

/**
 * @brief Configure les callbacks IRQ-safe
 * @note  irq_user_ctx est distinct de user_ctx (SetCallbacks) pour permettre
 *        des contextes différents entre main loop et ISR.
 */
void BMP_Async_SetIrqCallbacks(BMP_Async *ctx,
                               BMP_Async_OnIrqDataReadyCb on_irq_data_ready,
                               BMP_Async_OnIrqErrorCb on_irq_error,
                               void *irq_user_ctx);

/**
 * @brief Déclenche une lecture asynchrone en mode IT
 * @note  IT uniquement — DMA non implémenté (bus I2C partagé multi-capteurs).
 * @param ctx  Contexte async (non NULL)
 * @retval BMP_OK si démarré, BMP_ERR_BUSY / BMP_ERR_NULL_PTR / BMP_ERR_I2C sinon
 * @pre  ctx->hbmp->initialized == true  (BMP_Init() réussi)
 * @pre  NVIC activé pour le périphérique I2C utilisé
 * @pre  HAL_GetTick() monotone (uint32_t, wraparound géré par soustraction non signée)
 * @post ctx->state transitoire après déclenchement (ou ASYNC_IDLE si HAL_BUSY — normal sur bus partagé)
 */
BMP_Status BMP_ReadAll_IT(BMP_Async *ctx);

/**
 * @brief Machine à états asynchrone (appeler dans main loop)
 */
void BMP_Async_Process(BMP_Async_t *ctx, uint32_t now_ms);

/**
 * @brief Vérifie si contexte est inactif
 */
bool BMP_Async_IsIdle(const BMP_Async_t *ctx);

/**
 * @brief Vérifie si données disponibles
 */
bool BMP_Async_HasData(const BMP_Async_t *ctx);

/**
 * @brief Récupère les dernières données
 */
BMP_Status BMP_Async_GetData(BMP_Async_t *ctx, BMP_Data *out);

/**
 * @brief Vérification flag data_ready
 */
bool BMP_Async_DataReadyFlag(const BMP_Async_t *ctx);

/**
 * @brief Vérification flag error
 */
bool BMP_Async_ErrorFlag(const BMP_Async_t *ctx);

/**
 * @brief Clear les flags
 */
void BMP_Async_ClearFlags(BMP_Async_t *ctx);

/**
 * @brief Résultat court d'un appel BMP_Async_Tick()
 */
typedef enum {
  BMP_TICK_IDLE       = 0, /**< Contexte inactif */
  BMP_TICK_BUSY       = 1, /**< Mesure en cours */
  BMP_TICK_DATA_READY = 2, /**< Donnée disponible dans *data_out */
  BMP_TICK_ERROR      = 3  /**< Erreur — contexte réinitialisé */
} BMP_TickResult;

/**
 * @brief Avance la FSM async et retourne un résultat actionnable
 * @param ctx      Contexte async (non NULL)
 * @param now_ms   Timestamp HAL_GetTick()
 * @param data_out Pointeur pour recevoir la mesure (peut être NULL)
 * @retval BMP_TickResult
 */
BMP_TickResult BMP_Async_Tick(BMP_Async_t *ctx, uint32_t now_ms, BMP_Data *data_out);

/**
 * @brief  Déclenche une mesure IT à intervalle régulier
 * @param ctx     Contexte async (non NULL)
 * @param now_ms  Timestamp HAL_GetTick()
 * @param last_ms Pointeur vers le timestamp de la dernière mesure (mis à jour)
 * @note  La période est lue depuis `ctx->sample_interval_ms`
 *        (initialisé par BMP_Async_Init() avec BMPX_DEFAULT_SAMPLE_INTERVAL_MS).
 * @retval BMP_OK si déclenchée ou période non écoulée, BMP_ERR_BUSY sinon
 */
BMP_Status BMP_Async_TriggerEvery(BMP_Async_t *ctx, uint32_t now_ms,
                  uint32_t *last_ms);

/* Callbacks HAL à appeler depuis stm32l4xx_it.c */

/**
 * @brief À appeler depuis HAL_I2C_MasterTxCpltCallback()
 * @pre  Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_MasterTxCpltCallback)
 */
void BMP_Async_OnI2CMasterTxCplt(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief À appeler depuis HAL_I2C_MasterRxCpltCallback()
 */
void BMP_Async_OnI2CMasterRxCplt(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief À appeler depuis HAL_I2C_ErrorCallback()
 */
void BMP_Async_OnI2CError(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* STM32_BMPX_H */
