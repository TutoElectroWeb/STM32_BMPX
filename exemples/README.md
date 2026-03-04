# README Exemples — STM32_BMPX

## 1) Vue d'ensemble

Exemples v0.9 pour la librairie BMPX (BMP180/BMP280), en polling et async IT.

## 2) Fichiers d'exemple

- `exemple_bmp_polling.c`
- `exemple_bmp_polling_features.c`
- `exemple_bmp_async_it.c`

## 3) Pré-requis CubeMX

- I2C configuré en 7-bit.
- NVIC I2C event + error activé pour async IT.
- UART debug optionnel.
- Décision IT/DMA : IT retenu (DMA non prioritaire dans ces exemples).

## 4) Pattern d'initialisation

- Auto-détection puis init capteur (`BMP_InitAuto*`).
- En async : init contexte + callbacks + boucle périodique `TriggerEvery + Tick`.

## 5) Matrice API → exemples

- Polling: `BMP_ReadAll*` dans `polling`/`polling_features`
- Async: `BMP_Async_IsIdle`, `BMP_Async_TriggerEvery`, `BMP_Async_Tick` dans `async_it`
- `BMP_DeInit` : couverture via scénarios de lifecycle/recovery

## 6) Conventions v0.9

- Nommage `exemple_<lib>_<mode>.c`.
- Pas de `test_*`.
- Gestion d'erreur cohérente avec `consecutive_errors`.

## 7) Adaptation protocole (I2C)

- Callbacks I2C adaptés.
- Async IT utilisé comme implémentation de référence.
- §2.10b : décision IT/DMA explicitée.
