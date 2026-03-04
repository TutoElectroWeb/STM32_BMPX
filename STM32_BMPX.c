/**
  ******************************************************************************
  * @file    STM32_BMPX.c
  * @brief   Implementation du driver STM32 HAL pour capteurs BMP085/BMP180/BMP280
  * @author  STM32_LIB_STYLE_GUIDE
  * @version 0.9.0
  * @date    2026-02-11
  * @copyright Libre sous licence MIT.
  ******************************************************************************
  */

#include "STM32_BMPX.h"
#include <math.h>       ///< powf() pour calcul altitude

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef BMPX_DEBUG_ENABLE
const char* BMP_StatusToString(BMP_Status status)
{
    switch (status) {
        case BMP_OK:                  return "BMP_OK";
        case BMP_ERR_I2C:             return "BMP_ERR_I2C";
        case BMP_ERR_CHIP_ID:         return "BMP_ERR_CHIP_ID";
        case BMP_ERR_CAL_READ:        return "BMP_ERR_CAL_READ";
        case BMP_ERR_NULL_PTR:        return "BMP_ERR_NULL_PTR";
        case BMP_ERR_INVALID_PARAM:   return "BMP_ERR_INVALID_PARAM";
        case BMP_ERR_BUSY:            return "BMP_ERR_BUSY";
        case BMP_ERR_TIMEOUT:         return "BMP_ERR_TIMEOUT";
        case BMP_ERR_MATH:            return "BMP_ERR_MATH";
        case BMP_ERR_NOT_INITIALIZED: return "BMP_ERR_NOT_INITIALIZED";
        case BMP_ERR_NOT_CONFIGURED:  return "BMP_ERR_NOT_CONFIGURED";
        default:                      return "BMP_ERR_UNKNOWN";
    }
}

const char* BMP_SensorTypeToString(BMP_SensorType sensor_type)
{
    switch (sensor_type) {
        case BMP_SENSOR_BMP280:     return "BMP280";
        case BMP_SENSOR_BMP085_180: return "BMP085/BMP180";
        default:                    return "UNKNOWN";
    }
}
#else /* BMPX_DEBUG_ENABLE not defined — stubs retournant "" (évite linker error) */
const char* BMP_StatusToString(BMP_Status status)     { (void)status;      return ""; }
const char* BMP_SensorTypeToString(BMP_SensorType st) { (void)st;          return ""; }
#endif /* BMPX_DEBUG_ENABLE */

BMP_Status BMP_DetectAddress7b(I2C_HandleTypeDef *i2cdev, uint8_t *addr7b_out)
{
    if (i2cdev == NULL || addr7b_out == NULL) {
        return BMP_ERR_NULL_PTR;
    }

    const uint8_t addr_candidates_7b[] = { BMP_I2C_ADDR_SECONDARY_7B, BMP_I2C_ADDR_PRIMARY_7B };
    for (uint32_t i = 0; i < (sizeof(addr_candidates_7b) / sizeof(addr_candidates_7b[0])); ++i) {
        const uint16_t addr_8b = (uint16_t)(addr_candidates_7b[i] << 1);
        uint8_t reg = 0xD0u;
        uint8_t chip_id = 0u;

        if (HAL_I2C_Master_Transmit(i2cdev, addr_8b, &reg, 1u, 50u) != HAL_OK) {
            continue;
        }
        if (HAL_I2C_Master_Receive(i2cdev, addr_8b, &chip_id, 1u, 50u) != HAL_OK) {
            continue;
        }

        if (chip_id == 0x55u || chip_id == 0x58u) {
            *addr7b_out = addr_candidates_7b[i];
            return BMP_OK;
        }
    }

    return BMP_ERR_CHIP_ID;
}

// Registres de données de calibration
#define BMP_CAL_AC1 0xAA //!< R   Données de calibration (16 bits)
#define BMP_CAL_AC2 0xAC //!< R   Données de calibration (16 bits)
#define BMP_CAL_AC3 0xAE //!< R   Données de calibration (16 bits)
#define BMP_CAL_AC4 0xB0 //!< R   Données de calibration (16 bits)
#define BMP_CAL_AC5 0xB2 //!< R   Données de calibration (16 bits)
#define BMP_CAL_AC6 0xB4 //!< R   Données de calibration (16 bits)
#define BMP_CAL_B1 0xB6  //!< R   Données de calibration (16 bits)
#define BMP_CAL_B2 0xB8  //!< R   Données de calibration (16 bits)
#define BMP_CAL_MB 0xBA  //!< R   Données de calibration (16 bits)
#define BMP_CAL_MC 0xBC  //!< R   Données de calibration (16 bits)
#define BMP_CAL_MD 0xBE  //!< R   Données de calibration (16 bits)

// Commandes
#define BMP_CHIP_ID_REG 0xD0     //!< Registre contenant l'ID de la puce (devrait être 0x55)
#define BMP_CONTROL 0xF4         //!< Registre de contrôle
#define BMP_TEMPDATA 0xF6        //!< Registre des données de température
#define BMP_PRESSUREDATA 0xF6    //!< Registre des données de pression
#define BMP_READTEMPCMD 0x2E     //!< Commande pour lire la température
#define BMP_READPRESSURECMD 0x34 //!< Commande pour lire la pression

// Délais de conversion en ms selon le mode
#define BMP_TEMP_CONVERSION_DELAY 5
#define BMP_PRES_CONVERSION_DELAY_ULP 5
#define BMP_PRES_CONVERSION_DELAY_STD 8
#define BMP_PRES_CONVERSION_DELAY_HR 14
#define BMP_PRES_CONVERSION_DELAY_UHR 26
#define BMP_I2C_TIMEOUT BMPX_DEFAULT_TIMEOUT_MS ///< Timeout I2C (ms) — configurable dans STM32_BMPX_conf.h

// --- BMP280 (datasheet BST-BMP280-DS001) ---
#define BMP280_CHIP_ID 0x58
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CALIB00 0x88
#define BMP280_RESET_CMD 0xB6

// --- Fonctions statiques internes ---

// Fonctions de gestion I2C
// Retourne BMP_OK en cas de succès, ou un code d'erreur BMP_Status
static BMP_Status readBytes(BMP_Handle_t *bmp, uint8_t regAddr, uint8_t *buffer, uint8_t len)
{
    if (bmp == NULL || bmp->hi2c == NULL || buffer == NULL) {
        return BMP_ERR_NULL_PTR;
    }

    if (HAL_I2C_Master_Transmit(bmp->hi2c, (uint16_t)(bmp->i2cAddr7bit << 1), &regAddr, 1, BMP_I2C_TIMEOUT) != HAL_OK) {
        bmp->last_error = BMP_ERR_I2C;
        return BMP_ERR_I2C;
    }
    if (HAL_I2C_Master_Receive(bmp->hi2c, (uint16_t)(bmp->i2cAddr7bit << 1), buffer, len, BMP_I2C_TIMEOUT) != HAL_OK) {
        bmp->last_error = BMP_ERR_I2C;
        return BMP_ERR_I2C;
    }
    return BMP_OK;
}

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *value
static BMP_Status read8(BMP_Handle_t *bmp, uint8_t regAddr, uint8_t *value) // Lit un octet depuis un registre
{
    // Passe maintenant le handle bmp à readBytes
    return readBytes(bmp, regAddr, value, 1);
}

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *value
static BMP_Status read16(BMP_Handle_t *bmp, uint8_t regAddr, uint16_t *value) // Lit deux octets depuis un registre
{
    if (bmp == NULL || value == NULL) return BMP_ERR_NULL_PTR;
    uint8_t retbuf[2]; // Tampon pour stocker les deux octets
    BMP_Status status = readBytes(bmp, regAddr, retbuf, 2);
    if (status != BMP_OK) {
        return status; // Erreur
    }
    *value = ((uint16_t)retbuf[0] << 8) | retbuf[1]; // Combine les octets en un entier 16 bits (MSB first)
    return BMP_OK; // Succès
}

// Retourne BMP_OK en cas de succès
static BMP_Status write8(BMP_Handle_t *bmp, uint8_t regAddr, uint8_t data) // Écrit un octet dans un registre
{
    uint8_t tBuf[2]; // Tampon pour l'adresse et la donnée
    if (bmp == NULL) return BMP_ERR_NULL_PTR;
    tBuf[0] = regAddr; // Adresse du registre
    tBuf[1] = data; // Donnée à écrire
    if (HAL_I2C_Master_Transmit(bmp->hi2c, (uint16_t)(bmp->i2cAddr7bit << 1), tBuf, 2, BMP_I2C_TIMEOUT) != HAL_OK) {
        bmp->last_error = BMP_ERR_I2C;
        return BMP_ERR_I2C; // Erreur
    }
    return BMP_OK; // Succès
}

static uint16_t le_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t le_i16(const uint8_t *p)
{
  return (int16_t)le_u16(p);
}

static BMP_Status bmp280_wait_nvm(BMP_Handle_t *bmp, uint32_t timeout_ms)
{
  if (bmp == NULL) return BMP_ERR_NULL_PTR;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms) {
    uint8_t st = 0;
    BMP_Status s = read8(bmp, BMP280_REG_STATUS, &st);
    if (s != BMP_OK) return s;
    if ((st & 0x01u) == 0u) return BMP_OK; // im_update==0
    HAL_Delay(2);
  }
  bmp->last_error = BMP_ERR_TIMEOUT;
  return BMP_ERR_TIMEOUT;
}

static BMP_Status bmp280_wait_measuring(BMP_Handle_t *bmp, uint32_t timeout_ms)
{
  if (bmp == NULL) return BMP_ERR_NULL_PTR;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms) {
    uint8_t st = 0;
    BMP_Status s = read8(bmp, BMP280_REG_STATUS, &st);
    if (s != BMP_OK) return s;
    if ((st & 0x08u) == 0u) return BMP_OK; // measuring==0
    HAL_Delay(2);
  }
  bmp->last_error = BMP_ERR_TIMEOUT;
  return BMP_ERR_TIMEOUT;
}

static BMP_Status bmp280_read_calib(BMP_Handle_t *bmp)
{
  if (bmp == NULL) return BMP_ERR_NULL_PTR;
  uint8_t b[24];
  BMP_Status s = readBytes(bmp, BMP280_REG_CALIB00, b, sizeof(b));
  if (s != BMP_OK) return s;

  bmp->dig_T1 = le_u16(&b[0]);
  bmp->dig_T2 = le_i16(&b[2]);
  bmp->dig_T3 = le_i16(&b[4]);
  bmp->dig_P1 = le_u16(&b[6]);
  bmp->dig_P2 = le_i16(&b[8]);
  bmp->dig_P3 = le_i16(&b[10]);
  bmp->dig_P4 = le_i16(&b[12]);
  bmp->dig_P5 = le_i16(&b[14]);
  bmp->dig_P6 = le_i16(&b[16]);
  bmp->dig_P7 = le_i16(&b[18]);
  bmp->dig_P8 = le_i16(&b[20]);
  bmp->dig_P9 = le_i16(&b[22]);
  bmp->t_fine = 0;
  return BMP_OK;
}

static uint8_t bmp280_map_osrs_p(uint8_t mode085)
{
  // Mapping simple de l'API BMP085/180 vers oversampling pression BMP280
  // 0: x1, 1: x2, 2: x4, 3: x16
  switch (mode085) {
    case BMP_ULTRALOWPOWER: return 1;  // x1
    case BMP_STANDARD:      return 2;  // x2
    case BMP_HIGHRES:       return 3;  // x4
    case BMP_ULTRAHIGHRES:  return 5;  // x16
    default:                return 5;
  }
}

static BMP_Status bmp280_configure(BMP_Handle_t *bmp)
{
  if (bmp == NULL) return BMP_ERR_NULL_PTR;

  // Reset
  BMP_Status s = write8(bmp, BMP280_REG_RESET, BMP280_RESET_CMD);
  if (s != BMP_OK) return s;
  HAL_Delay(5);
  s = bmp280_wait_nvm(bmp, 100);
  if (s != BMP_OK) return s;

  // Read calibration
  s = bmp280_read_calib(bmp);
  if (s != BMP_OK) {
    bmp->last_error = BMP_ERR_CAL_READ;
    return BMP_ERR_CAL_READ;
  }

  // Put to sleep before config
  BMP280_Config cfg;
  cfg.osrs_t = BMP280_OSRS_X1;
  cfg.osrs_p = bmp280_map_osrs_p(bmp->oversampling);
  cfg.filter = BMP280_FILTER_OFF;
  cfg.standby = BMP280_STANDBY_1000_MS;
  cfg.mode = BMP280_MODE_NORMAL;

  s = BMP280_SetConfig(bmp, &cfg);
  if (s != BMP_OK) return s;

  return BMP_OK;
}

BMP_Status BMP280_SetConfig(BMP_Handle_t *bmp, const BMP280_Config *cfg)
{
  if (bmp == NULL || cfg == NULL) return BMP_ERR_NULL_PTR;
  if (bmp->sensorType != BMP_SENSOR_BMP280) return BMP_ERR_INVALID_PARAM;

  if (cfg->osrs_t > BMP280_OSRS_X16) return BMP_ERR_INVALID_PARAM;
  if (cfg->osrs_p > BMP280_OSRS_X16) return BMP_ERR_INVALID_PARAM;
  if (cfg->filter > BMP280_FILTER_16) return BMP_ERR_INVALID_PARAM;
  if (cfg->standby > BMP280_STANDBY_4000_MS) return BMP_ERR_INVALID_PARAM;
  if (!((cfg->mode == BMP280_MODE_SLEEP) || (cfg->mode == BMP280_MODE_FORCED) || (cfg->mode == BMP280_MODE_NORMAL))) {
    return BMP_ERR_INVALID_PARAM;
  }

  // Datasheet recommande: passer en sleep avant de modifier config
  BMP_Status s = write8(bmp, BMP280_REG_CTRL_MEAS, (uint8_t)((cfg->osrs_t << 5) | (cfg->osrs_p << 2) | BMP280_MODE_SLEEP));
  if (s != BMP_OK) return s;

  // CONFIG : t_sb[7:5], filter[4:2]
  s = write8(bmp, BMP280_REG_CONFIG, (uint8_t)((cfg->standby << 5) | (cfg->filter << 2)));
  if (s != BMP_OK) return s;

  // CTRL_MEAS : osrs_t[7:5], osrs_p[4:2], mode[1:0]
  s = write8(bmp, BMP280_REG_CTRL_MEAS, (uint8_t)((cfg->osrs_t << 5) | (cfg->osrs_p << 2) | (cfg->mode & 0x03u)));
  if (s != BMP_OK) return s;

  return BMP_OK;
}

BMP_Status BMP280_ReadAll_OneShot(BMP_Handle_t *bmp, float *temp_degC, int32_t *pressure_Pa, uint32_t timeout_ms)
{
  if (bmp == NULL || temp_degC == NULL || pressure_Pa == NULL) return BMP_ERR_NULL_PTR;
  if (bmp->sensorType != BMP_SENSOR_BMP280) return BMP_ERR_INVALID_PARAM;
  if (timeout_ms == 0u) timeout_ms = 100u;

  // Lire ctrl_meas pour récupérer osrs_t/osrs_p en place
  uint8_t ctrl = 0;
  BMP_Status s = read8(bmp, BMP280_REG_CTRL_MEAS, &ctrl);
  if (s != BMP_OK) return s;

  uint8_t osrs_t = (ctrl >> 5) & 0x07u;
  uint8_t osrs_p = (ctrl >> 2) & 0x07u;

  // Déclenche une mesure forced (mode=01)
  s = write8(bmp, BMP280_REG_CTRL_MEAS, (uint8_t)((osrs_t << 5) | (osrs_p << 2) | BMP280_MODE_FORCED));
  if (s != BMP_OK) return s;

  // Attendre fin de conversion
  s = bmp280_wait_measuring(bmp, timeout_ms);
  if (s != BMP_OK) return s;

  // Lire et compenser
    return BMP_ReadAll(bmp, temp_degC, pressure_Pa);
}

static BMP_Status bmp280_read_raw(BMP_Handle_t *bmp, int32_t *adc_T, int32_t *adc_P)
{
  if (bmp == NULL || adc_T == NULL || adc_P == NULL) return BMP_ERR_NULL_PTR;
  uint8_t b[6];
  BMP_Status s = readBytes(bmp, BMP280_REG_PRESS_MSB, b, sizeof(b));
  if (s != BMP_OK) return s;
  *adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | ((int32_t)b[2] >> 4);
  *adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | ((int32_t)b[5] >> 4);
  return BMP_OK;
}

static int32_t bmp280_comp_temp_01C(BMP_Handle_t *bmp, int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp->dig_T1 << 1))) * ((int32_t)bmp->dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp->dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->dig_T1))) >> 12) * ((int32_t)bmp->dig_T3)) >> 14;
  bmp->t_fine = var1 + var2;
  return (bmp->t_fine * 5 + 128) >> 8; // 0.01°C
}

static int32_t bmp280_comp_press_Pa(BMP_Handle_t *bmp, int32_t adc_P)
{
  int64_t var1 = (int64_t)bmp->t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)bmp->dig_P6;
  var2 = var2 + ((var1 * (int64_t)bmp->dig_P5) << 17);
  var2 = var2 + (((int64_t)bmp->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)bmp->dig_P3) >> 8) + ((var1 * (int64_t)bmp->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)bmp->dig_P1) >> 33;
  if (var1 == 0) {
    bmp->last_error = BMP_ERR_MATH;
    return 0;
  }

  int64_t p = 1048576 - (int64_t)adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)bmp->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)bmp->dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7) << 4);
  return (int32_t)(p >> 8);
}

// --- Fonctions Publiques ---

BMP_Status BMP_InitWithAddress(BMP_Handle_t *bmp, I2C_HandleTypeDef *i2cdev, uint8_t i2c_address_7bit, uint8_t mode)
{
  if (bmp == NULL || i2cdev == NULL) {
    // Ne peut pas stocker l'erreur dans bmp->last_error car bmp peut être NULL
    return BMP_ERR_NULL_PTR;
  }

  bmp->hi2c = i2cdev; // Associe le périphérique I2C
  bmp->initialized       = false;
  bmp->consecutive_errors = 0U;
  bmp->last_hal_error    = 0U;
  bmp->last_error = BMP_OK; // Initialise le statut d'erreur
  bmp->sensorType = BMP_SENSOR_UNKNOWN;

    BMP_Status addr_status = BMP_SetAddress(bmp, i2c_address_7bit); // Valide et configure l'adresse I2C active
    if (addr_status != BMP_OK) {
        return addr_status;
    }

    if (mode > BMP_ULTRAHIGHRES) { // Vérifie si le mode est valide
        bmp->last_error = BMP_ERR_INVALID_PARAM;
        return BMP_ERR_INVALID_PARAM;
    }
  bmp->oversampling = mode; // Définit le niveau de suréchantillonnage

  uint8_t chipId = 0;
  BMP_Status status = read8(bmp, BMP_CHIP_ID_REG, &chipId);
  if (status != BMP_OK) {
    bmp->last_error = status;
    return status;
  }

  if (chipId == 0x55) {
    bmp->sensorType = BMP_SENSOR_BMP085_180;
  } else if (chipId == BMP280_CHIP_ID) {
    bmp->sensorType = BMP_SENSOR_BMP280;
    // Configuration BMP280 (reset + calib + ctrl/config)
    status = bmp280_configure(bmp);
    if (status != BMP_OK) {
      bmp->last_error = status;
      return status;
    }
    bmp->initialized = true;
    return BMP_OK;
  } else {
    bmp->last_error = BMP_ERR_CHIP_ID;
    return BMP_ERR_CHIP_ID;
  }

  /* Lit les données de calibration */
  // Vérifier chaque lecture de calibration pour les erreurs I2C
  // Correction: read16 prend bmp, regAddr, value*
  #define CHECK_CAL_READ(reg, var_ptr) if ((status = read16(bmp, reg, (uint16_t*)var_ptr)) != BMP_OK) { bmp->last_error = status; return status; }
  CHECK_CAL_READ(BMP_CAL_AC1, &bmp->ac1);
  CHECK_CAL_READ(BMP_CAL_AC2, &bmp->ac2);
  CHECK_CAL_READ(BMP_CAL_AC3, &bmp->ac3);
  CHECK_CAL_READ(BMP_CAL_AC4, &bmp->ac4);
  CHECK_CAL_READ(BMP_CAL_AC5, &bmp->ac5);
  CHECK_CAL_READ(BMP_CAL_AC6, &bmp->ac6);

  CHECK_CAL_READ(BMP_CAL_B1, &bmp->b1);
  // Suppression des anciennes lignes if (read16(...) != 0)
  CHECK_CAL_READ(BMP_CAL_B2, &bmp->b2);

  CHECK_CAL_READ(BMP_CAL_MB, &bmp->mb);
  CHECK_CAL_READ(BMP_CAL_MC, &bmp->mc);
  CHECK_CAL_READ(BMP_CAL_MD, &bmp->md);
  #undef CHECK_CAL_READ

  bmp->initialized = true;
  return BMP_OK; // Initialisation réussie
}

BMP_Status BMP_InitAuto(BMP_Handle_t *bmp, I2C_HandleTypeDef *i2cdev, uint8_t mode)
{
    uint8_t addr7b = 0u;
    BMP_Status detect_status = BMP_DetectAddress7b(i2cdev, &addr7b);
    if (detect_status != BMP_OK) {
        if (bmp != NULL) {
            bmp->last_error = detect_status;
        }
        return detect_status;
    }

    return BMP_InitWithAddress(bmp, i2cdev, addr7b, mode);
}

BMP_Status BMP_DeInit(BMP_Handle_t *bmp)
{
    if (bmp == NULL)
        return BMP_ERR_NULL_PTR;

    bmp->hi2c               = NULL;
    bmp->initialized        = false;
    bmp->consecutive_errors = 0U;
    bmp->last_hal_error     = 0U;
    bmp->last_error         = BMP_OK;
    bmp->sensorType         = BMP_SENSOR_UNKNOWN;
    bmp->i2cAddr7bit        = 0U;
    bmp->oversampling       = 0U;
    return BMP_OK;
}

BMP_Status BMP_SetAddress(BMP_Handle_t *bmp, uint8_t i2c_address_7bit)
{
    if (bmp == NULL) {
        return BMP_ERR_NULL_PTR;
    }

    if (i2c_address_7bit != BMP_I2C_ADDR_PRIMARY_7B &&
        i2c_address_7bit != BMP_I2C_ADDR_SECONDARY_7B) {
        bmp->last_error = BMP_ERR_INVALID_PARAM;
        return BMP_ERR_INVALID_PARAM;
    }

    bmp->i2cAddr7bit = i2c_address_7bit;
    bmp->last_error = BMP_OK;
    return BMP_OK;
}

BMP_Status BMP_UsePrimaryAddress(BMP_Handle_t *bmp)
{
    return BMP_SetAddress(bmp, BMP_I2C_ADDR_PRIMARY_7B);
}

BMP_Status BMP_UseSecondaryAddress(BMP_Handle_t *bmp)
{
    return BMP_SetAddress(bmp, BMP_I2C_ADDR_SECONDARY_7B);
}

uint8_t BMP_GetAddress(const BMP_Handle_t *bmp)
{
    if (bmp == NULL || bmp->hi2c == NULL) {
        return 0u;
    }

    return bmp->i2cAddr7bit;
}

// --- Fonctions de calcul internes (utilisent le handle) ---

// Calcule la valeur B5 pour la température. Retourne B5 ou 0 en cas d'erreur.
static int32_t computeB5(BMP_Handle_t *bmp, int32_t UT)
{
  if (bmp == NULL) {
      return 0; // Ne peut pas définir last_error (bmp NULL)
  }

  int32_t X1 = (UT - (int32_t)bmp->ac6) * ((int32_t)bmp->ac5) >> 15;
  int32_t denom = X1 + (int32_t)bmp->md;
  if (denom == 0) {
      bmp->last_error = BMP_ERR_MATH;
      return 0; // Erreur de division par zéro potentielle
  }
  int32_t X2 = ((int32_t)bmp->mc << 11) / denom;
  return X1 + X2; // Retourne la somme de X1 et X2
}

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *ut
static BMP_Status readRawTemperature(BMP_Handle_t *bmp, int32_t *ut) // Lit la température brute
{
  if (bmp == NULL || ut == NULL) {
      return BMP_ERR_NULL_PTR;
  }

  BMP_Status status = write8(bmp, BMP_CONTROL, BMP_READTEMPCMD);
  if (status != BMP_OK) return status; // Erreur I2C
  HAL_Delay(BMP_TEMP_CONVERSION_DELAY); // Attend la conversion
  uint16_t raw_temp;
  // Correction: Appel à read16 avec les bons arguments
  status = read16(bmp, BMP_TEMPDATA, &raw_temp);
  if (status != BMP_OK) return status; // Erreur I2C
  *ut = raw_temp;
  return BMP_OK; // Succès
}

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *up
static BMP_Status readRawPressure(BMP_Handle_t *bmp, int32_t *up) // Lit la pression brute
{
  uint16_t raw_pres_h;
  uint8_t  raw_pres_l;
  uint32_t delay_ms;
  // Suppression de l'ancien appel write8 incorrect

  if (bmp == NULL || up == NULL) {
      return BMP_ERR_NULL_PTR;
  }

  BMP_Status status = write8(bmp, BMP_CONTROL, BMP_READPRESSURECMD + (bmp->oversampling << 6));
  if (status != BMP_OK) return status;

  // Choix du délai en fonction du mode
  if (bmp->oversampling == BMP_ULTRALOWPOWER) // Vérifie le niveau de suréchantillonnage
    delay_ms = BMP_PRES_CONVERSION_DELAY_ULP;
  else if (bmp->oversampling == BMP_STANDARD)
    delay_ms = BMP_PRES_CONVERSION_DELAY_STD;
  // Correction: Utilisation de bmp->oversampling
  else if (bmp->oversampling == BMP_HIGHRES)
    delay_ms = BMP_PRES_CONVERSION_DELAY_HR;
  else
    delay_ms = BMP_PRES_CONVERSION_DELAY_UHR;

  HAL_Delay(delay_ms);

  status = read16(bmp, BMP_PRESSUREDATA, &raw_pres_h); // Lit les deux premiers octets de la pression
  // Correction: Appel à read16 avec les bons arguments (déjà correct ici)
  if (status != BMP_OK) return status;
  status = read8(bmp, BMP_PRESSUREDATA + 2, &raw_pres_l); // Lit le troisième octet
  // Correction: Appel à read8 avec les bons arguments (déjà correct ici)
  if (status != BMP_OK) return status;

  *up = ((uint32_t)raw_pres_h << 8) | raw_pres_l;
  *up >>= (8 - bmp->oversampling); // Ajuste selon le niveau de suréchantillonnage

  return BMP_OK; // Succès
}

// --- Fonctions de lecture publiques ---

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *temp_degC
BMP_Status BMP_ReadTemperature(BMP_Handle_t *bmp, float *temp_degC) // Calcule et retourne la température en °C
{
  if (bmp == NULL || temp_degC == NULL) {
      return BMP_ERR_NULL_PTR;
  }

  if (bmp->sensorType == BMP_SENSOR_BMP280) {
      int32_t adc_T = 0, adc_P = 0;
      BMP_Status s = bmp280_read_raw(bmp, &adc_T, &adc_P);
      if (s != BMP_OK) return s;
      int32_t t01 = bmp280_comp_temp_01C(bmp, adc_T);
      *temp_degC = (float)t01 / 100.0f;
      return BMP_OK;
  }

  int32_t UT = 0;
  int32_t B5;
  float temp; // Variable pour la température finale
  BMP_Status status;

  status = readRawTemperature(bmp, &UT); // Lit la température brute
  if (status != BMP_OK) {
      return status; // Erreur I2C ou autre
  }

  B5 = computeB5(bmp, UT); // Calcule B5
  if (bmp->last_error != BMP_OK) {
      return bmp->last_error; // Vérifie erreur dans computeB5
  }
  temp = (B5 + 8) >> 4; // Calcule la température en dixièmes de degré
  temp /= 10; // Convertit en degrés Celsius

    *temp_degC = temp;

  return BMP_OK; // Succès
}

// Calcule la pression finale à partir de la pression brute (UP) et de B5 (calculé à partir de la température brute)
// Retourne la pression en Pa.
// Note : Cette fonction NE lit PAS les valeurs brutes, elle fait juste le calcul.
// Suppression de la définition dupliquée
static int32_t calculatePressure(BMP_Handle_t *bmp, int32_t UP, int32_t B5) // Garde celle-ci
{
  if (bmp == NULL) {
      return 0; // Ne peut pas stocker l'erreur
  }

  int32_t B3, B6, X1, X2, X3, p; // Variables intermédiaires
  uint32_t B4, B7; // Variables intermédiaires non signées

  // Effectue les calculs de pression
  B6 = B5 - 4000;
  X1 = ((int32_t)bmp->b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)bmp->ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)bmp->ac1 * 4 + X3) << bmp->oversampling) + 2) / 4;

  X1 = ((int32_t)bmp->ac3 * B6) >> 13;
  X2 = ((int32_t)bmp->b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)bmp->ac4 * (uint32_t)(X3 + 32768)) >> 15;

  if (B4 == 0) { // Vérification division par zéro
      bmp->last_error = BMP_ERR_MATH;
      return 0;
  }

  B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> bmp->oversampling);

  if (B7 < 0x80000000) // Vérifie si B7 est inférieur à une limite
  {
    p = (B7 * 2) / B4; // Calcule la pression
  }
  else
  {
    p = (B7 / B4) * 2; // Calcule la pression
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4); // Ajuste la pression finale

  return p;
}

// Retourne BMP_OK en cas de succès. Met la valeur lue dans *pressure_Pa
// Fonction conservée pour compatibilité ou usage simple, mais moins efficace que BMP_ReadAll.
BMP_Status BMP_ReadPressure(BMP_Handle_t *bmp, int32_t *pressure_Pa) // Calcule et retourne la pression en Pa
{
  if (bmp == NULL || pressure_Pa == NULL) {
      return BMP_ERR_NULL_PTR;
  }

  if (bmp->sensorType == BMP_SENSOR_BMP280) {
      int32_t adc_T = 0, adc_P = 0;
      BMP_Status s = bmp280_read_raw(bmp, &adc_T, &adc_P);
      if (s != BMP_OK) return s;
      (void)bmp280_comp_temp_01C(bmp, adc_T);
      int32_t pPa = bmp280_comp_press_Pa(bmp, adc_P);
      if (bmp->last_error != BMP_OK) return bmp->last_error;
      *pressure_Pa = pPa;
      return BMP_OK;
  }

  int32_t UT, UP, B5;
  BMP_Status status;

  // Lire la température brute (nécessaire pour le calcul de pression)
  status = readRawTemperature(bmp, &UT);
  if (status != BMP_OK) {
      return status;
  }
  // Lire la pression brute
  status = readRawPressure(bmp, &UP);
  if (status != BMP_OK) {
      return status;
  }

  B5 = computeB5(bmp, UT); // Calcule B5
  if (bmp->last_error != BMP_OK) {
      return bmp->last_error; // Vérifie erreur dans computeB5
  }

  *pressure_Pa = calculatePressure(bmp, UP, B5); // Calcule la pression finale
  if (bmp->last_error != BMP_OK) {
      return bmp->last_error; // Vérifie erreur dans calculatePressure
  }

  return BMP_OK; // Succès
}

// Lit la température et la pression de manière optimisée.
// Retourne BMP_OK en cas de succès.
BMP_Status BMP_ReadAll(BMP_Handle_t *bmp, float *temp_degC, int32_t *pressure_Pa)
{
    if (bmp == NULL || temp_degC == NULL || pressure_Pa == NULL) {
        return BMP_ERR_NULL_PTR;
    }

  if (bmp->sensorType == BMP_SENSOR_BMP280) {
    int32_t adc_T = 0, adc_P = 0;
    BMP_Status s = bmp280_read_raw(bmp, &adc_T, &adc_P);
    if (s != BMP_OK) return s;
    int32_t t01 = bmp280_comp_temp_01C(bmp, adc_T);
    int32_t pPa = bmp280_comp_press_Pa(bmp, adc_P);
    if (bmp->last_error != BMP_OK) return bmp->last_error;
    *temp_degC = (float)t01 / 100.0f;
    *pressure_Pa = pPa;
    return BMP_OK;
  }

    int32_t UT, UP, B5;
    float temp;
    BMP_Status status;

    // 1. Lire la température brute
    status = readRawTemperature(bmp, &UT);
    if (status != BMP_OK) {
        return status;
    }

    // 2. Lire la pression brute
    status = readRawPressure(bmp, &UP);
    if (status != BMP_OK) {
        return status;
    }

    // 3. Calculer B5 (nécessaire pour les deux)
    // Correction: Appel à computeB5 avec le handle
    B5 = computeB5(bmp, UT);
    if (bmp->last_error != BMP_OK) {
        return bmp->last_error; // Vérifie erreur dans computeB5
    }

    // 4. Calculer la température finale
    temp = (B5 + 8) >> 4;
    temp /= 10;
    *temp_degC = temp;

    // 5. Calculer la pression finale
    *pressure_Pa = calculatePressure(bmp, UP, B5);
    if (bmp->last_error != BMP_OK) {
        return bmp->last_error; // Vérifie erreur dans calculatePressure
    }
    return BMP_OK; // Succès
}

// --- Fonctions de calcul publiques (ne dépendent pas directement du handle) ---

// Calcule l'altitude en mètres. Retourne BMP_OK ou BMP_ERR_INVALID_PARAM.
BMP_Status BMP_CalculateAltitude(int32_t pressure_Pa, float sealevelPressure_Pa, float *altitude_m)
{
  if (altitude_m == NULL) {
      return BMP_ERR_NULL_PTR;
  }

  if (pressure_Pa <= 0 || sealevelPressure_Pa <= 0) { // Vérifie les pressions valides
      *altitude_m = NAN; // Ou une autre valeur d'erreur comme -9999.0f
      return BMP_ERR_INVALID_PARAM;
  }
  // Utilise powf pour les opérations en virgule flottante
  *altitude_m = 44330.0f * (1.0f - powf((float)pressure_Pa / sealevelPressure_Pa, 0.1903f)); // Calcule l'altitude
  return BMP_OK;
}

// Calcule la pression au niveau de la mer (Pa) à partir de la pression mesurée (Pa) et de l'altitude (m)
int32_t BMP_CalculateSeaLevelPressure(int32_t pressure_Pa, float altitude_m) // Cette fonction n'est pas utilisée dans main.c pour l'instant
{
  // Utilise powf pour les opérations en virgule flottante
  // Attention : vérifier que le dénominateur ne devient pas nul ou négatif si altitude_m est très grand
  float factor = powf(1.0f - altitude_m / 44330.0f, 5.255f);
  if (factor <= 0) {
      // Retourner une valeur invalide comme 0
      return 0;
  }
  int32_t sealevel_p = (int32_t)((float)pressure_Pa / factor);
  return sealevel_p; // Calcule la pression au niveau de la mer
} // Correction: Ajout de l'accolade fermante manquante

/* ============================================================================
 * API ASYNCHRONE (IT uniquement)
 * @note DMA non implémenté : bus I2C partagé multi-capteurs (arbitrage impossible)
 *       et trames courtes (BMP280 = 6 octets, BMP085/180 = 2+3 octets).
 * ============================================================================ */

/**
 * @brief  Initialise le contexte asynchrone
 * @param  ctx  Contexte async à initialiser (non NULL)
 * @param  bmp  Handle capteur associé (non NULL)
 * @retval None
 */
void BMP_Async_Init(BMP_Async_t *ctx, BMP_Handle_t *bmp) {
    if (ctx == NULL || bmp == NULL) return;
    
    ctx->bmp = bmp;
    ctx->state = BMP_ASYNC_IDLE;
    ctx->last_status = BMP_OK;
    ctx->async_busy = 0U;
    ctx->last_hal_error = 0U;
    ctx->sample_interval_ms = BMPX_DEFAULT_SAMPLE_INTERVAL_MS;
    ctx->conv_deadline_ms = 0;
    ctx->i2c_deadline_ms = 0;
    
    ctx->data_ready_flag = false;
    ctx->error_flag = false;
    ctx->notify_data_pending = false;
    ctx->notify_error_pending = false;
    
    ctx->on_data_ready = NULL;
    ctx->on_error = NULL;
    ctx->on_irq_data_ready = NULL;
    ctx->on_irq_error = NULL;
    ctx->user_ctx = NULL;
    ctx->irq_user_ctx = NULL;
}

/**
 * @brief  Réinitialise la machine d'état async sans perdre les callbacks
 * @param  ctx  Contexte asynchrone à réinitialiser (non NULL)
 * @retval None
 */
void BMP_Async_Reset(BMP_Async_t *ctx) {
    if (ctx == NULL) return;

    ctx->state                = BMP_ASYNC_IDLE;
    ctx->last_status          = BMP_OK;
    ctx->async_busy           = 0U;
    ctx->last_hal_error       = 0U;
    ctx->conv_deadline_ms     = 0;
    ctx->i2c_deadline_ms      = 0;                // Réinitialise deadline I2C
    ctx->data_ready_flag      = false;            // Efface flag données prêtes
    ctx->error_flag           = false;            // Efface flag erreur
    ctx->notify_data_pending  = false;            // Annule notification data en attente
    ctx->notify_error_pending = false;            // Annule notification erreur en attente

    /* bmp, mode, callbacks et user_ctx sont préservés */
}

/**
 * @brief Configure les callbacks utilisateur
 */
void BMP_Async_SetCallbacks(BMP_Async_t *ctx,
                            BMP_Async_OnDataReadyCb on_data_ready,
                            BMP_Async_OnErrorCb on_error,
                            void *user_ctx) {
    if (ctx == NULL) return;
    ctx->on_data_ready = on_data_ready;
    ctx->on_error = on_error;
    ctx->user_ctx = user_ctx;
}

/**
 * @brief Configure les callbacks IRQ-safe
 */
void BMP_Async_SetIrqCallbacks(BMP_Async_t *ctx,
                               BMP_Async_OnIrqDataReadyCb on_irq_data_ready,
                               BMP_Async_OnIrqErrorCb on_irq_error,
                               void *irq_user_ctx) {
    if (ctx == NULL) return;
    ctx->on_irq_data_ready = on_irq_data_ready;
    ctx->on_irq_error = on_irq_error;
    ctx->irq_user_ctx = irq_user_ctx;
}

/**
 * @brief Calcule délai conversion pression selon mode
 */
static uint32_t BMP_GetPressureConvDelay(uint8_t mode) {
    switch (mode) {
        case BMP_ULTRALOWPOWER: return BMP_PRES_CONVERSION_DELAY_ULP;
        case BMP_STANDARD: return BMP_PRES_CONVERSION_DELAY_STD;
        case BMP_HIGHRES: return BMP_PRES_CONVERSION_DELAY_HR;
        case BMP_ULTRAHIGHRES: return BMP_PRES_CONVERSION_DELAY_UHR;
        default: return BMP_PRES_CONVERSION_DELAY_STD;
    }
}

/**
 * @brief Déclenche lecture async mode IT
 */
BMP_Status BMP_ReadAll_IT(BMP_Async_t *ctx) {
    if (ctx == NULL || ctx->bmp == NULL) {
        return BMP_ERR_NULL_PTR;
    }
    
    if (ctx->state != BMP_ASYNC_IDLE) {
        return BMP_ERR_BUSY;
    }
    
    ctx->last_status = BMP_OK;
    ctx->i2c_deadline_ms = HAL_GetTick() + BMP_I2C_TIMEOUT;

    if (ctx->bmp->sensorType == BMP_SENSOR_BMP280) {
        ctx->state = BMP_ASYNC_READ_TEMP_RX;
        HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_IT(
            ctx->bmp->hi2c, (uint16_t)(ctx->bmp->i2cAddr7bit << 1),
            BMP280_REG_PRESS_MSB, 1, ctx->bmp280_rx_buf, 6);

        if (hal_status != HAL_OK) {
            if (hal_status == HAL_BUSY) {
                ctx->state = BMP_ASYNC_IDLE;
                ctx->last_status = BMP_ERR_BUSY;
                return BMP_ERR_BUSY;
            }
            if (hal_status == HAL_TIMEOUT) {
                ctx->last_status = BMP_ERR_TIMEOUT;
                return BMP_ERR_TIMEOUT;
            }
            ctx->last_status = BMP_ERR_I2C;
            return BMP_ERR_I2C;
        }
        ctx->async_busy = 1U;
        return BMP_OK;
    }

    ctx->state = BMP_ASYNC_READ_TEMP_TX;
    
    /* Envoi commande lecture température (IT) */
    uint8_t cmd = BMP_READTEMPCMD;
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write_IT(
        ctx->bmp->hi2c, (uint16_t)(ctx->bmp->i2cAddr7bit << 1), 
        BMP_CONTROL, 1, &cmd, 1);
    
    if (hal_status != HAL_OK) {
        if (hal_status == HAL_BUSY) {
            ctx->state = BMP_ASYNC_IDLE;
            ctx->last_status = BMP_ERR_BUSY;
            return BMP_ERR_BUSY;
        }
        ctx->state = BMP_ASYNC_ERROR;
        if (hal_status == HAL_TIMEOUT) {
            ctx->last_status = BMP_ERR_TIMEOUT;
            return BMP_ERR_TIMEOUT;
        }
        ctx->last_status = BMP_ERR_I2C;
        return BMP_ERR_I2C;
    }

    ctx->async_busy = 1U;
    return BMP_OK;
}

/**
 * @brief Machine à états asynchrone
 */
void BMP_Async_Process(BMP_Async_t *ctx, uint32_t now_ms) {
    if (ctx == NULL) return;
    
    // Notifications callbacks
    if (ctx->notify_data_pending) {
        ctx->notify_data_pending = false;
        if (ctx->on_data_ready) {
            ctx->on_data_ready(ctx->user_ctx, &ctx->data, ctx->last_status);
        }
        return;
    }
    
    if (ctx->notify_error_pending) {
        ctx->notify_error_pending = false;
        if (ctx->on_error) {
            ctx->on_error(ctx->user_ctx, ctx->last_status);
        }
        return;
    }
    
    // Machine à états
    switch (ctx->state) {
        case BMP_ASYNC_IDLE:
        case BMP_ASYNC_DONE:
        case BMP_ASYNC_ERROR:
            break;
            
        case BMP_ASYNC_READ_TEMP_TX:
            // Attente callback TxCplt
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = BMP_ASYNC_ERROR;
                ctx->last_status = BMP_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
            
        case BMP_ASYNC_WAIT_TEMP:
            // Attente conversion température
            if ((int32_t)(now_ms - ctx->conv_deadline_ms) >= 0) {
                // Lance lecture données température
                ctx->state = BMP_ASYNC_READ_TEMP_RX;
                ctx->i2c_deadline_ms = now_ms + BMP_I2C_TIMEOUT;
                
                HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_IT(
                    ctx->bmp->hi2c, (uint16_t)(ctx->bmp->i2cAddr7bit << 1),
                    BMP_TEMPDATA, 1, ctx->temp_rx_buf, 2);
                
                if (hal_status != HAL_OK) {
                    ctx->state = BMP_ASYNC_ERROR;
                    if (hal_status == HAL_BUSY) {
                        ctx->last_status = BMP_ERR_BUSY;
                    } else if (hal_status == HAL_TIMEOUT) {
                        ctx->last_status = BMP_ERR_TIMEOUT;
                    } else {
                        ctx->last_status = BMP_ERR_I2C;
                    }
                    ctx->error_flag = true;
                }
            }
            break;
            
        case BMP_ASYNC_READ_TEMP_RX:
            // Attente callback RxCplt température
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = BMP_ASYNC_ERROR;
                ctx->last_status = BMP_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
            
        case BMP_ASYNC_READ_PRESS_TX:
            // Attente callback TxCplt pression
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = BMP_ASYNC_ERROR;
                ctx->last_status = BMP_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
            
        case BMP_ASYNC_WAIT_PRESS:
            // Attente conversion pression
            if ((int32_t)(now_ms - ctx->conv_deadline_ms) >= 0) {
                // Lance lecture données pression
                ctx->state = BMP_ASYNC_READ_PRESS_RX;
                ctx->i2c_deadline_ms = now_ms + BMP_I2C_TIMEOUT;
                
                HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_IT(
                    ctx->bmp->hi2c, (uint16_t)(ctx->bmp->i2cAddr7bit << 1),
                    BMP_PRESSUREDATA, 1, ctx->press_rx_buf, 3);
                
                if (hal_status != HAL_OK) {
                    ctx->state = BMP_ASYNC_ERROR;
                    if (hal_status == HAL_BUSY) {
                        ctx->last_status = BMP_ERR_BUSY;
                    } else if (hal_status == HAL_TIMEOUT) {
                        ctx->last_status = BMP_ERR_TIMEOUT;
                    } else {
                        ctx->last_status = BMP_ERR_I2C;
                    }
                    ctx->error_flag = true;
                }
            }
            break;
            
        case BMP_ASYNC_READ_PRESS_RX:
            // Attente callback RxCplt pression
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = BMP_ASYNC_ERROR;
                ctx->last_status = BMP_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
    }
}

bool BMP_Async_IsIdle(const BMP_Async_t *ctx) {
    return (ctx != NULL) && (ctx->state == BMP_ASYNC_IDLE);
}

bool BMP_Async_HasData(const BMP_Async_t *ctx) {
    return (ctx != NULL) && (ctx->state == BMP_ASYNC_DONE);
}

BMP_Status BMP_Async_GetData(BMP_Async_t *ctx, BMP_Data *out) {
    if (ctx == NULL || out == NULL) {
        return BMP_ERR_NULL_PTR;
    }
    
    if (ctx->state != BMP_ASYNC_DONE) {
        return BMP_ERR_BUSY;
    }
    
    *out = ctx->data;
    ctx->state = BMP_ASYNC_IDLE;
    ctx->data_ready_flag = false;
    ctx->notify_data_pending = false;
    return ctx->last_status;
}

bool BMP_Async_DataReadyFlag(const BMP_Async_t *ctx) {
    return (ctx != NULL) && ctx->data_ready_flag;
}

bool BMP_Async_ErrorFlag(const BMP_Async_t *ctx) {
    return (ctx != NULL) && ctx->error_flag;
}

void BMP_Async_ClearFlags(BMP_Async_t *ctx) {
    if (ctx == NULL) return;
    ctx->data_ready_flag = false;
    ctx->error_flag = false;
    ctx->notify_data_pending = false;
    ctx->notify_error_pending = false;
}

BMP_TickResult BMP_Async_Tick(BMP_Async_t *ctx, uint32_t now_ms, BMP_Data *data_out) {
    if (ctx == NULL) {
        return BMP_TICK_IDLE;
    }

    BMP_Async_Process(ctx, now_ms);

    if (BMP_Async_ErrorFlag(ctx) || ctx->state == BMP_ASYNC_ERROR) {
        BMP_Status err = ctx->last_status;
        BMP_Async_ClearFlags(ctx);
        ctx->async_busy = 0U;
        BMP_Async_Reset(ctx);
        ctx->last_status = err;
        return BMP_TICK_ERROR;
    }

    if (BMP_Async_DataReadyFlag(ctx) || BMP_Async_HasData(ctx)) {
        ctx->async_busy = 0U;
        if (data_out != NULL) {
            (void)BMP_Async_GetData(ctx, data_out);
        } else {
            BMP_Data drop;
            (void)BMP_Async_GetData(ctx, &drop);
        }
        return BMP_TICK_DATA_READY;
    }

    if (BMP_Async_IsIdle(ctx)) {
        return BMP_TICK_IDLE;
    }

    return BMP_TICK_BUSY;
}

BMP_Status BMP_Async_TriggerEvery(BMP_Async_t *ctx, uint32_t now_ms,
                                    uint32_t *last_ms) {
    if (ctx == NULL || last_ms == NULL) {
        return BMP_ERR_INVALID_PARAM;
    }

    if (!BMP_Async_IsIdle(ctx)) {
        return BMP_OK;  /* occupé — pas une erreur, retente au prochain appel */
    }

    if ((uint32_t)(now_ms - *last_ms) < ctx->sample_interval_ms) {
        return BMP_OK;
    }

    BMP_Status st = BMP_ReadAll_IT(ctx);
    if (st == BMP_OK) {
        *last_ms = now_ms;
    }
    return st;
}

/* ============================================================================
 * Callbacks HAL
 * ============================================================================ */

void BMP_Async_OnI2CMasterTxCplt(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->bmp == NULL || ctx->bmp->hi2c != hi2c) return;
    
    if (ctx->state == BMP_ASYNC_READ_TEMP_TX) {
        // Commande température envoyée -> attente conversion
        ctx->state = BMP_ASYNC_WAIT_TEMP;
        ctx->conv_deadline_ms = HAL_GetTick() + BMP_TEMP_CONVERSION_DELAY;
    }
    else if (ctx->state == BMP_ASYNC_READ_PRESS_TX) {
        // Commande pression envoyée -> attente conversion
        ctx->state = BMP_ASYNC_WAIT_PRESS;
        uint32_t delay = BMP_GetPressureConvDelay(ctx->bmp->oversampling);
        ctx->conv_deadline_ms = HAL_GetTick() + delay;
    }
}

void BMP_Async_OnI2CMasterRxCplt(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->bmp == NULL || ctx->bmp->hi2c != hi2c) return;
    
    if (ctx->state == BMP_ASYNC_READ_TEMP_RX) {
        if (ctx->bmp->sensorType == BMP_SENSOR_BMP280) {
            int32_t adc_P = ((int32_t)ctx->bmp280_rx_buf[0] << 12)
                          | ((int32_t)ctx->bmp280_rx_buf[1] << 4)
                          | ((int32_t)ctx->bmp280_rx_buf[2] >> 4);
            int32_t adc_T = ((int32_t)ctx->bmp280_rx_buf[3] << 12)
                          | ((int32_t)ctx->bmp280_rx_buf[4] << 4)
                          | ((int32_t)ctx->bmp280_rx_buf[5] >> 4);

            int32_t t01 = bmp280_comp_temp_01C(ctx->bmp, adc_T);
            int32_t pPa = bmp280_comp_press_Pa(ctx->bmp, adc_P);

            if (ctx->bmp->last_error != BMP_OK) {
                ctx->state = BMP_ASYNC_ERROR;
                ctx->last_status = ctx->bmp->last_error;
                ctx->error_flag = true;
                if (ctx->on_irq_error) ctx->on_irq_error(ctx->irq_user_ctx);
                ctx->notify_error_pending = true;
                return;
            }

            ctx->data.temperature = (float)t01 / 100.0f;
            ctx->data.pressure = pPa;
            ctx->state = BMP_ASYNC_DONE;
            ctx->last_status = BMP_OK;
            ctx->data_ready_flag = true;

            if (ctx->on_irq_data_ready) ctx->on_irq_data_ready(ctx->irq_user_ctx);
            ctx->notify_data_pending = true;
            return;
        }

        // Température reçue -> lance lecture pression
        ctx->state = BMP_ASYNC_READ_PRESS_TX;
        ctx->i2c_deadline_ms = HAL_GetTick() + BMP_I2C_TIMEOUT;
        
        uint8_t cmd = BMP_READPRESSURECMD + (ctx->bmp->oversampling << 6);
        HAL_StatusTypeDef hal_status;
        
        hal_status = HAL_I2C_Mem_Write_IT(
            ctx->bmp->hi2c, (uint16_t)(ctx->bmp->i2cAddr7bit << 1),
            BMP_CONTROL, 1, &cmd, 1);
        
        if (hal_status != HAL_OK) {
            ctx->state = BMP_ASYNC_ERROR;
            if (hal_status == HAL_BUSY) {
                ctx->last_status = BMP_ERR_BUSY;
            } else if (hal_status == HAL_TIMEOUT) {
                ctx->last_status = BMP_ERR_TIMEOUT;
            } else {
                ctx->last_status = BMP_ERR_I2C;
            }
            ctx->error_flag = true;
            if (ctx->on_irq_error) ctx->on_irq_error(ctx->irq_user_ctx);
            ctx->notify_error_pending = true;
        }
    }
    else if (ctx->state == BMP_ASYNC_READ_PRESS_RX) {
        // Pression reçue -> calculs finaux
        
        // Calcul température (logique BMP085/180)
        int32_t ut = ((int32_t)ctx->temp_rx_buf[0] << 8) | ctx->temp_rx_buf[1];
        int32_t x1 = ((ut - ctx->bmp->ac6) * ctx->bmp->ac5) >> 15;
        int32_t denom = x1 + ctx->bmp->md;
        if (denom == 0) {
            ctx->state = BMP_ASYNC_ERROR;
            ctx->last_status = BMP_ERR_MATH;
            ctx->error_flag = true;
            if (ctx->on_irq_error) ctx->on_irq_error(ctx->irq_user_ctx);
            ctx->notify_error_pending = true;
            return;
        }
        int32_t x2 = ((int32_t)ctx->bmp->mc << 11) / denom;
        int32_t b5 = x1 + x2;
        ctx->data.temperature = ((b5 + 8) >> 4) / 10.0f;
        
        // Calcul pression (logique BMP085/180)
        int32_t up = (((int32_t)ctx->press_rx_buf[0] << 16) | 
                      ((int32_t)ctx->press_rx_buf[1] << 8) | 
                      ctx->press_rx_buf[2]) >> (8 - ctx->bmp->oversampling);
        
        int32_t b6 = b5 - 4000;
        x1 = (ctx->bmp->b2 * ((b6 * b6) >> 12)) >> 11;
        x2 = (ctx->bmp->ac2 * b6) >> 11;
        int32_t x3 = x1 + x2;
        int32_t b3 = ((((int32_t)ctx->bmp->ac1 * 4 + x3) << ctx->bmp->oversampling) + 2) >> 2;
        
        x1 = (ctx->bmp->ac3 * b6) >> 13;
        x2 = (ctx->bmp->b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        uint32_t b4 = (ctx->bmp->ac4 * (uint32_t)(x3 + 32768)) >> 15;
        if (b4 == 0u) {
            ctx->state = BMP_ASYNC_ERROR;
            ctx->last_status = BMP_ERR_MATH;
            ctx->error_flag = true;
            if (ctx->on_irq_error) ctx->on_irq_error(ctx->irq_user_ctx);
            ctx->notify_error_pending = true;
            return;
        }
        uint32_t b7 = ((uint32_t)up - b3) * (50000 >> ctx->bmp->oversampling);
        
        int32_t p;
        if (b7 < 0x80000000) {
            p = (b7 << 1) / b4;
        } else {
            p = (b7 / b4) << 1;
        }
        
        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        ctx->data.pressure = p + ((x1 + x2 + 3791) >> 4);
        
        ctx->state = BMP_ASYNC_DONE;
        ctx->last_status = BMP_OK;
        ctx->data_ready_flag = true;
        
        if (ctx->on_irq_data_ready) ctx->on_irq_data_ready(ctx->irq_user_ctx);
        ctx->notify_data_pending = true;
    }
}

void BMP_Async_OnI2CError(BMP_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->bmp == NULL || ctx->bmp->hi2c != hi2c) return;

    if (ctx->state != BMP_ASYNC_IDLE && ctx->state != BMP_ASYNC_DONE) {
        ctx->state          = BMP_ASYNC_ERROR;
        ctx->last_status    = BMP_ERR_I2C;
        ctx->async_busy     = 0U;
        ctx->last_hal_error = (uint32_t)HAL_I2C_GetError(hi2c);
        ctx->error_flag     = true;
        /* Propagation vers le handle principal */
        if (ctx->bmp->consecutive_errors < 0xFFU)
            ctx->bmp->consecutive_errors++;
        ctx->bmp->last_hal_error = ctx->last_hal_error;
        ctx->bmp->last_error     = BMP_ERR_I2C;

        if (ctx->on_irq_error) ctx->on_irq_error(ctx->irq_user_ctx);
        ctx->notify_error_pending = true;
    }
}
