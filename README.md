# STM32_BMPX – Driver STM32 HAL pour BMP085 / BMP180 / BMP280 (Bosch)

Driver C pur pour les capteurs **Bosch BMP085 / BMP180 / BMP280** sur STM32 HAL (I2C).
Mesure pression atmosphérique et température avec calcul d'altitude barométrique.

**Fonctionnalités :**

- Auto-détection du type de capteur (BMP085/180 via chip_id `0x55`, BMP280 via `0x58`)
- Adresses I2C configurables : `0x77` (primaire) ou `0x76` (secondaire, SDO=GND)
- Configuration oversampling BMP085/180 (ULow Power…Ultra High Res)
- Configuration avancée BMP280 (osrs_t, osrs_p, filtre IIR, standby, mode Normal/Forced)
- Mesure one-shot BMP280 (mode FORCED + attente conversion)
- Calcul altitude barométrique et pression niveau de la mer
- API synchrone bloquante + API asynchrone IT uniquement (DMA non implémenté : bus partagé + trames courtes ≤ 6 octets)
- Gestion d'erreurs structurée (`BMP_Status`, `consecutive_errors`, `last_hal_error`)
- Aucun `printf` ni `malloc` dans la librairie

---

## 1) Périmètre

| Fonctionnalité                                     |     Statut     | Justification                                              |
| -------------------------------------------------- | :------------: | ---------------------------------------------------------- |
| Lecture température + pression (BMP085/180)        |    SUPPORTÉ    | `BMP_ReadAll`, polling bloquant + async IT                 |
| Lecture température + pression (BMP280)            |    SUPPORTÉ    | `BMP_ReadAll`, `BMP280_ReadAll_OneShot`                    |
| Auto-détection capteur (chip_id `0xD0`)            |    SUPPORTÉ    | BMP085/180 → `0x55`, BMP280 → `0x58`                       |
| Adressage dual (0x77 / 0x76)                       |    SUPPORTÉ    | `BMP_UsePrimaryAddress` / `BMP_UseSecondaryAddress`        |
| Oversampling BMP085/180 (OSS 0..3)                 |    SUPPORTÉ    | Paramètre `mode` dans `BMP_InitWithAddress`                |
| Configuration BMP280 (osrs, filter, standby, mode) |    SUPPORTÉ    | `BMP280_SetConfig` + `BMP280_Config`                       |
| Mesure BMP280 mode NORMAL (mesures automatiques)   |    SUPPORTÉ    | Via `BMP280_SetConfig(... mode=NORMAL ...)`                |
| Mesure BMP280 mode FORCED (one-shot)               |    SUPPORTÉ    | `BMP280_ReadAll_OneShot`                                   |
| Calcul altitude barométrique                       |    SUPPORTÉ    | `BMP_CalculateAltitude`                                    |
| Calcul pression niveau de la mer                   |    SUPPORTÉ    | `BMP_CalculateSeaLevelPressure`                            |
| API asynchrone IT                                  |    SUPPORTÉ    | `BMP_Async_Tick`, `TriggerEvery`, callbacks                |
| Interface SPI (BMP280)                             |  NON SUPPORTÉ  | Hors périmètre — I2C uniquement                            |
| DMA I2C                                            |  NON SUPPORTÉ  | Bus partagé multi-capteurs + trames ≤ 6 octets → IT suffit |
| Interruptions matérielles capteur (broche INT)     | NON APPLICABLE | BMP085/180/BMP280 n'exposent pas d'IRQ data-ready standard |

---

## 2) Fichiers

| Fichier              | Rôle                                              |
| -------------------- | ------------------------------------------------- |
| `STM32_BMPX.h`       | API publique, types, constantes, defines          |
| `STM32_BMPX.c`       | Implémentation sync + async (IT uniquement)       |
| `STM32_BMPX_conf.h`  | Paramètres configurables (timeouts, seuils, etc.) |
| `exemples/`          | 3 exemples couvrant 100 % de l'API                |
| `exemples/README.md` | Index des exemples + matrice couverture API       |

---

## 3) Pré-requis CubeMX / HAL

### Décision IT vs DMA (Q1→Q4)

| Critère                  | Analyse BMPX                                                             |
| ------------------------ | ------------------------------------------------------------------------ |
| **Q1 – Taille trame RX** | 6 octets (BMP280), 2+3 octets (BMP085/180) — trames très courtes         |
| **Q2 – Bus I2C partagé** | Oui (multi-capteurs : SGP40, AHT20, DHT20…) — DMA bloquerait l'arbitrage |
| **Q3 – CPU critique**    | Non — la latence IT sur 6 octets est négligeable                         |
| **Q4 – Conclusion**      | **IT uniquement — DMA non implémenté et non nécessaire**                 |

### Configuration minimale

- **I2C** : mode Master 7 bits, 100 kHz (Standard) ou 400 kHz (Fast Mode)
- **Pull-up I2C** : 4.7 kΩ externes sur SDA/SCL (recommandé)
- **Horloge** : `HAL_GetTick()` fonctionnel (SysTick par défaut)
- **Mode async IT** : activer NVIC I2C Event + Error du bus choisi
- **Optionnel** : UART pour `printf` dans les exemples

### Brochage typique

| BMP085/180/BMP280 | STM32 Nucleo-L476RG                         |
| :---------------- | :------------------------------------------ |
| VCC               | 3.3V                                        |
| GND               | GND                                         |
| SCL               | PB8 (I2C1 SCL)                              |
| SDA               | PB9 (I2C1 SDA)                              |
| SDO               | GND → adresse `0x76` / VCC → adresse `0x77` |

---

## 4) Initialisation

### Méthode rapide (tout-en-un, auto-détection)

```c
#include "STM32_BMPX.h"

BMP_Handle_t hbmp;

BMP_Status st = BMP_Init(&hbmp, &hi2c1);  /* = BMP_InitAuto(..., BMP_STANDARD) */
if (st != BMP_OK) { Error_Handler(); }
```

### Méthode explicite (adresse + mode)

```c
BMP_Handle_t hbmp;

/* Adresse primaire (SDO=VCC=0x77), mode haute résolution */
BMP_Status st = BMP_InitWithAddress(&hbmp, &hi2c1, BMP_I2C_ADDR_PRIMARY_7B, BMP_HIGHRES);
if (st != BMP_OK) {
    printf("Init BMPX: %s\r\n", BMP_StatusToString(st));
    Error_Handler();
}
```

### DeInit (réinitialisation / arrêt)

```c
BMP_DeInit(&hbmp);   /* Dissocie hi2c, efface initialized, consecutive_errors, last_hal_error */
```

---

## 5) Flux recommandé

```
BMP_InitAuto / BMP_InitWithAddress
        │
(optionnel) BMP280_SetConfig(osrs, filter, standby, mode)
        │
    ┌───┴──────────── Boucle principale ─────────────┐
    │  BMP_Async_TriggerEvery(&ctx, now, &last_t)    │  ← déclenche IT si intervalle écoulé
    │  BMP_Async_Tick(&ctx, now, &data)              │  ← avance la FSM
    │  if DATA_READY → traiter data                  │
    │  if ERROR      → BMP_Async_Reset + compteur    │
    └────────────────────────────────────────────────┘
```

Exemple polling minimal :

```c
while (1) {
    float temp;
    int32_t press;

    BMP_Status st = BMP_ReadAll(&hbmp, &temp, &press);
    if (st == BMP_OK) {
        printf("T=%.2f°C  P=%ld Pa\r\n", temp, press);

        float alt;
        BMP_CalculateAltitude(press, 101325.0f, &alt);
        printf("Alt=%.1f m\r\n", alt);
    }
    HAL_Delay(2000U);
}
```

---

## 6) API publique complète

### 6.1 Initialisation et configuration

| Fonction                                                               | Description                                           |
| ---------------------------------------------------------------------- | ----------------------------------------------------- |
| `BMP_Init(bmp, hi2c)`                                                  | Alias : `BMP_InitAuto(bmp, hi2c, BMP_DEFAULT_MODE)`   |
| `BMP_InitAuto(bmp, hi2c, mode)`                                        | Auto-détection adresse + init calibration             |
| `BMP_InitWithAddress(bmp, hi2c, addr7b, mode)`                         | Init avec adresse explicite (0x76 ou 0x77)            |
| `BMP_DeInit(bmp)`                                                      | Remet le handle à zéro (dissocie I2C, efface erreurs) |
| `BMP_DetectAddress7b(hi2c, &addr7b_out)`                               | Scan I2C : détecte l'adresse active                   |
| `BMP_SetAddress` / `BMP_UsePrimaryAddress` / `BMP_UseSecondaryAddress` | Adressage manuel                                      |
| `BMP_GetAddress(bmp)`                                                  | Retourne l'adresse 7-bit courante                     |

### 6.2 Lecture sync (bloquante)

| Fonction                                                 | Description                              |
| -------------------------------------------------------- | ---------------------------------------- |
| `BMP_ReadTemperature(bmp, &temp_degC)`                   | Lit la température en °C                 |
| `BMP_ReadPressure(bmp, &pressure_Pa)`                    | Lit la pression en Pa                    |
| `BMP_ReadAll(bmp, &temp_degC, &pressure_Pa)`             | Lit T+P en une seule opération (optimal) |
| `BMP280_SetConfig(bmp, &cfg)`                            | Configure osrs/filter/standby/mode       |
| `BMP280_ReadAll_OneShot(bmp, &temp, &press, timeout_ms)` | Mesure FORCED + attente conversion       |

### 6.3 Calculs

| Fonction                                               | Description                                   |
| ------------------------------------------------------ | --------------------------------------------- |
| `BMP_CalculateAltitude(press_Pa, sealevel_Pa, &alt_m)` | Altitude barométrique (formule int. standard) |
| `BMP_CalculateSeaLevelPressure(press_Pa, alt_m)`       | Pression niveau de la mer                     |

### 6.4 Debug (conditionnel `#ifdef BMPX_DEBUG_ENABLE`)

| Fonction                              | Description                       |
| ------------------------------------- | --------------------------------- |
| `BMP_StatusToString(status)`          | `BMP_Status` → chaîne lisible     |
| `BMP_SensorTypeToString(sensor_type)` | `BMP_SensorType` → chaîne lisible |

### 6.5 API asynchrone IT

| Fonction                                                  | Description                                  |
| --------------------------------------------------------- | -------------------------------------------- |
| `BMP_Async_Init(ctx, bmp)`                                | Initialise le contexte async                 |
| `BMP_Async_Reset(ctx)`                                    | Reset FSM (préserve callbacks)               |
| `BMP_Async_SetCallbacks(ctx, on_data, on_err, user)`      | Callbacks main-loop                          |
| `BMP_Async_SetIrqCallbacks(ctx, on_data, on_err, user)`   | Callbacks ISR (ultra-courts)                 |
| `BMP_ReadAll_IT(ctx)`                                     | Lance une mesure IT                          |
| `BMP_Async_TriggerEvery(ctx, now_ms, &last_ms)`           | Déclenchement périodique automatique         |
| `BMP_Async_Process(ctx, now_ms)`                          | Avance la FSM (variant sans résultat direct) |
| `BMP_Async_Tick(ctx, now_ms, &data_out)`                  | Avance la FSM → `BMP_TickResult`             |
| `BMP_Async_IsIdle(ctx)`                                   | FSM au repos ?                               |
| `BMP_Async_HasData(ctx)` / `BMP_Async_DataReadyFlag(ctx)` | Donnée disponible ?                          |
| `BMP_Async_GetData(ctx, &out)`                            | Récupère T+P                                 |
| `BMP_Async_ErrorFlag(ctx)` / `BMP_Async_ClearFlags(ctx)`  | Gestion flags                                |
| `BMP_Async_OnI2CMasterTxCplt(ctx, hi2c)`                  | Relais callback HAL TX                       |
| `BMP_Async_OnI2CMasterRxCplt(ctx, hi2c)`                  | Relais callback HAL RX                       |
| `BMP_Async_OnI2CError(ctx, hi2c)`                         | Relais callback HAL Error                    |

---

## 7) Gestion d'erreurs

### Codes `BMP_Status`

```c
typedef enum {
    BMP_OK = 0,              /* Opération réussie */
    BMP_ERR_I2C,             /* Erreur bus I2C (TX ou RX) */
    BMP_ERR_CHIP_ID,         /* chip_id non reconnu (0x55 ou 0x58 attendu) */
    BMP_ERR_CAL_READ,        /* Erreur lecture calibration NVM */
    BMP_ERR_NULL_PTR,        /* Pointeur NULL passé en argument */
    BMP_ERR_INVALID_PARAM,   /* Paramètre invalide (mode, plage…) */
    BMP_ERR_BUSY,            /* Contexte occupé (mesure async en cours) */
    BMP_ERR_TIMEOUT,         /* Timeout HAL I2C */
    BMP_ERR_MATH,            /* Erreur calcul interne (division par zéro) */
    BMP_ERR_NOT_INITIALIZED, /* Handle non initialisé */
    BMP_ERR_NOT_CONFIGURED   /* Pré-requis HW/CubeMX absents */
} BMP_Status;
```

| Code                      | Diagnostic typique                               |
| ------------------------- | ------------------------------------------------ |
| `BMP_ERR_I2C`             | Câblage SDA/SCL, pull-up, adresse, bus occupé    |
| `BMP_ERR_CHIP_ID`         | Capteur absent, mauvaise adresse (SDO mal câblé) |
| `BMP_ERR_CAL_READ`        | Capteur instable, bruit alimentation             |
| `BMP_ERR_BUSY`            | `BMP_ReadAll_IT` relancé avant fin de la FSM     |
| `BMP_ERR_NOT_INITIALIZED` | `BMP_Init` non appelé ou a échoué                |
| `BMP_ERR_MATH`            | Coefficients calibration corrompus (rare)        |
| `BMP_ERR_TIMEOUT`         | Capteur ne répond plus (alimentation ?)          |

### `consecutive_errors` + `last_hal_error`

```c
/* Lecture dans le handle après une erreur */
printf("Erreurs consécutives : %u\r\n", hbmp.consecutive_errors);
printf("Code HAL : 0x%08lX\r\n", hbmp.last_hal_error);  /* HAL_I2C_GetError() */

/* Dans le contexte async */
printf("HAL async : 0x%08lX\r\n", bmp_async.last_hal_error);
```

- `consecutive_errors` est incrémenté à chaque `BMP_ERR_I2C` dans `BMP_Async_OnI2CError`.
- `consecutive_errors` **n'est pas** incrémenté pour `BMP_ERR_BUSY` (contention normale).
- Seuil configurable via `BMPX_MAX_CONSECUTIVE_ERRORS` (par défaut : 3).

> **`BMP_StatusToString()`** disponible uniquement si `#define BMPX_DEBUG_ENABLE` est défini
> avant l'inclusion de `STM32_BMPX.h`. En production, laisser commenté pour économiser la Flash.

---

## 8) Contraintes et comportements

- **BMP085/180 et BMP280** partagent la même API (`BMP_ReadAll`), le bon code path est sélectionné automatiquement sur `bmp->sensorType`.
- **Oversampling BMP085/180** : mode `BMP_ULTRAHIGHRES` nécessite 25.5 ms de conversion (voir §13).
- **BMP280 mode NORMAL** : `BMP280_SetConfig(..., BMP280_MODE_NORMAL)` → `BMP_ReadAll` lit les derniers résultats disponibles en continu.
- **HAL_BUSY TX → FSM reste IDLE** : `BMP_ReadAll_IT` retourne `BMP_ERR_BUSY`, l'état reste `IDLE`, `TriggerEvery` retente au prochain appel.
- **Bus partagé** : l'adresse primaire `0x77` peut entrer en conflit avec un DS1307 RTC — vérifier l'absence de conflit sur le bus.
- **Portabilité** : `#include "main.h"` uniquement (compatible toutes familles STM32 via CubeMX).
- **0 malloc** : aucune allocation dynamique dans la librairie.

---

## 9) Intégration asynchrone

### États FSM (`BMP_AsyncState`)

```
IDLE → READ_TEMP_TX → WAIT_TEMP → READ_TEMP_RX
     → READ_PRESS_TX → WAIT_PRESS → READ_PRESS_RX → DONE
   ↘ ERROR (à tout moment sur erreur I2C ou timeout)
```

### `BMP_TickResult` (valeurs FIXES)

| Valeur numérique | Constante             | Signification                              |
| :--------------: | --------------------- | ------------------------------------------ |
|       `0`        | `BMP_TICK_IDLE`       | FSM inactive, rien à faire                 |
|       `1`        | `BMP_TICK_BUSY`       | Mesure en cours                            |
|       `2`        | `BMP_TICK_DATA_READY` | `data_out` rempli avec T+P valides         |
|       `3`        | `BMP_TICK_ERROR`      | Erreur — FSM réinitialisée automatiquement |

### Callbacks HAL (dans `USER CODE BEGIN 4`)

```c
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BMP_Async_OnI2CMasterTxCplt(&bmp_async, hi2c);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BMP_Async_OnI2CMasterRxCplt(&bmp_async, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BMP_Async_OnI2CError(&bmp_async, hi2c);
}
```

> Chaque callback contient un guard `ctx->bmp->hi2c != hi2c` en interne — multi-instance safe.

### Boucle principale (pattern minimal)

```c
while (1) {
    uint32_t now = HAL_GetTick();
    BMP_Data data;

    BMP_Async_TriggerEvery(&bmp_async, now, &last_trigger);  /* Déclenche si intervalle écoulé */

    BMP_TickResult tick = BMP_Async_Tick(&bmp_async, now, &data);
    if (tick == BMP_TICK_DATA_READY) {
        printf("T=%.2f°C  P=%ld Pa\r\n", data.temperature, data.pressure);
    } else if (tick == BMP_TICK_ERROR) {
        error_count++;
        if (error_count >= BMPX_MAX_CONSECUTIVE_ERRORS) {
            BMP_Async_Reset(&bmp_async);
            error_count = 0;
        }
    }
}
```

### Compatibilité FreeRTOS

> **FreeRTOS** : les champs `async_busy`, `state`, `last_status`, `data_ready_flag`, `error_flag`
> sont déclarés `volatile`. Pour un accès depuis une tâche principale et une IRQ, aucun mutex
> n'est requis si la tâche principale est la seule à appeler `Tick`/`TriggerEvery`.
> Ne jamais appeler `BMP_ReadAll()` (bloquant) pendant une mesure async.

### Override de l'intervalle

```c
BMP_Async_Init(&bmp_async, &hbmp);
bmp_async.sample_interval_ms = 5000U;  /* Surcharge BMPX_DEFAULT_SAMPLE_INTERVAL_MS (2000ms) */
```

---

## 10) Exemples

Dossier : `exemples/` — index complet dans `exemples/README.md`.

| Fichier                           | Description                                                     |
| --------------------------------- | --------------------------------------------------------------- |
| `exemple_bmpx_polling.c`          | Polling auto-détection, BMP085/180 + BMP280, monitoring erreurs |
| `exemple_bmpx_polling_features.c` | Polling avancé : BMP280 SetConfig, one-shot, async init         |
| `exemple_bmpx_async_it.c`         | Mesures non bloquantes IT + callbacks + FreeRTOS note           |

### Couverture API

| Fonction publique                    | polling | polling_features | async_it |
| ------------------------------------ | :-----: | :--------------: | :------: |
| `BMP_Init` / `BMP_InitWithAddress`   |    ✓    |        ✓         |    ✓     |
| `BMP_DeInit`                         |    ✓    |        ✓         |          |
| `BMP_SetAddress` / `UsePrimary…`     |    ✓    |                  |          |
| `BMP_ReadAll`                        |    ✓    |        ✓         |          |
| `BMP_ReadTemperature`                |    ✓    |                  |          |
| `BMP_ReadPressure`                   |    ✓    |                  |          |
| `BMP_CalculateAltitude`              |    ✓    |        ✓         |    ✓     |
| `BMP_CalculateSeaLevelPressure`      |    ✓    |                  |          |
| `BMP280_SetConfig`                   |         |        ✓         |          |
| `BMP280_ReadAll_OneShot`             |         |        ✓         |          |
| `BMP_Async_Init`                     |         |        ✓         |    ✓     |
| `BMP_Async_Reset`                    |         |        ✓         |          |
| `BMP_Async_SetCallbacks`             |         |        ✓         |    ✓     |
| `BMP_Async_SetIrqCallbacks`          |         |        ✓         |    ✓     |
| `BMP_ReadAll_IT`                     |         |        ✓         |    ✓     |
| `BMP_Async_TriggerEvery`             |         |        ✓         |    ✓     |
| `BMP_Async_Tick`                     |         |        ✓         |    ✓     |
| `BMP_Async_Process`                  |         |        ✓         |          |
| `BMP_Async_IsIdle`                   |         |        ✓         |          |
| `BMP_Async_HasData` / `GetData`      |         |        ✓         |          |
| `BMP_Async_DataReadyFlag`            |         |        ✓         |          |
| `BMP_Async_ErrorFlag` / `ClearFlags` |         |        ✓         |          |
| `BMP_Async_OnI2CMasterTxCplt`        |         |        ✓         |    ✓     |
| `BMP_Async_OnI2CMasterRxCplt`        |         |        ✓         |    ✓     |
| `BMP_Async_OnI2CError`               |         |        ✓         |    ✓     |
| `BMP_StatusToString`                 |    ✓    |        ✓         |    ✓     |
| `BMP_SensorTypeToString`             |    ✓    |        ✓         |          |

---

## 11) Conformité

- Timings conversion datasheet BMP180 respectés (max 25.5 ms OSS=3) via `BMPX_WAIT_TIMEOUT_MS=50U`
- Timings BMP280 respectés (Forced mode : attente polling `status.measuring`)
- Convention CubeMX `USER CODE BEGIN/END` respectée dans tous les exemples
- Aucun `HAL_Delay()` après `__disable_irq()`
- Aucun `printf` dans la librairie (debug uniquement via `#ifdef BMPX_DEBUG_ENABLE`)
- Adresses I2C HAL = `addr7b << 1` (shift appliqué en interne, l'utilisateur passe toujours l'adresse 7-bit)

---

## 12) Dépannage

| Erreur / Symptôme                       | Cause probable                                 | Correction                                        |
| --------------------------------------- | ---------------------------------------------- | ------------------------------------------------- |
| `BMP_ERR_CHIP_ID`                       | Capteur absent ou mauvaise adresse             | Vérifier SDO, pull-up, câblage SDA/SCL            |
| `BMP_ERR_I2C` systématique              | Pull-up manquant, tension trop basse           | Ajouter 4.7 kΩ SDA/SCL, vérifier VCC=3.3V         |
| `BMP_ERR_BUSY` en async                 | `Tick` / `TriggerEvery` non appelé dans boucle | Vérifier la boucle principale                     |
| `BMP_ERR_NOT_INITIALIZED`               | `BMP_Init` non appelé ou a retourné une erreur | Vérifier le retour d'`Init` avant toute lecture   |
| `BMP_ERR_CAL_READ`                      | Alimentation instable lors de l'init           | Ajouter condensateur 100 nF sur VCC capteur       |
| Erreur compilation `BMP_StatusToString` | `BMPX_DEBUG_ENABLE` non défini                 | Ajouter `#define BMPX_DEBUG_ENABLE` AVANT include |
| Callbacks IT non appelés                | NVIC I2C EV/ER non activé dans CubeMX          | Activer « NVIC I2C Event interrupt »              |
| Adresse `0x77` en conflit               | DS1307 RTC sur le même bus                     | Passer BMPX en adresse secondaire `0x76`          |

---

## 13) Paramètres configurables (`STM32_BMPX_conf.h`)

Tous les paramètres sont surchargeable via `#define` avant l'inclusion de `STM32_BMPX.h`
ou via les options de compilation (`-DBMPX_MAX_CONSECUTIVE_ERRORS=5`).

| Macro                             | Défaut | Unité | Description                                                                        |
| --------------------------------- | :----: | :---: | ---------------------------------------------------------------------------------- |
| `BMPX_DEFAULT_TIMEOUT_MS`         | `100`  |  ms   | Timeout I2C pour les transactions bloquantes                                       |
| `BMPX_WAIT_TIMEOUT_MS`            |  `50`  |  ms   | Timeout attente conversion (datasheet BMP180 OSS=3 : max 25.5 ms)                  |
| `BMPX_MAX_CONSECUTIVE_ERRORS`     |  `3`   |   —   | Seuil erreurs I2C consécutives avant signalement critique                          |
| `BMPX_DEFAULT_SAMPLE_INTERVAL_MS` | `2000` |  ms   | Intervalle `TriggerEvery` par défaut (surchargeable via `ctx->sample_interval_ms`) |
| `BMPX_DEBUG_ENABLE`               |  `—`   |   —   | Active `BMP_StatusToString()` et `BMP_SensorTypeToString()`                        |

> **Production** : `BMPX_DEBUG_ENABLE` doit rester commenté pour économiser la Flash.
> Une erreur `HAL_BUSY` ne compte pas comme erreur consécutive.

---

## 14) Version

- **Driver** : 0.9.0
- **Standard lib** : v0.9 (SGP40-aligné)
- **Datasheet applicable** :
  - [BMP085 Datasheet (BST-BMP085-DS000)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp085-ds000.pdf)
  - [BMP180 Datasheet (BST-BMP180-DS000)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp180-ds000.pdf)
  - [BMP280 Datasheet (BST-BMP280-DS001)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)

### Changelog

| Version | Date       | Changements                                                                                                                                                                                                    |
| ------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0.9.0   | 2026-02-22 | Standard v0.9 : `initialized`, `consecutive_errors`, `last_hal_error`, `async_busy`, `sample_interval_ms`, `BMP_DeInit`, `StatusToString #ifdef`, `TriggerEvery` 3 params, `conf.h DEFAULT_SAMPLE_INTERVAL_MS` |
