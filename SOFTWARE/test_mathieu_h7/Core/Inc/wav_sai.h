/*
 * wav_sai.h
 *
 *  Created on: Oct 16, 2025
 *      Author: mathi
 */

#ifndef INC_WAV_SAI_H_
#define INC_WAV_SAI_H_
/* wav_sai_min.h — lecteur WAV minimal (SAI + DMA + FatFs)
 * Formats: PCM 16-bit, mono (dupliqué) ou stéréo
 * Périph: SAI master TX, DataSize=16, 2 slots, MCK off
 */

#include <stdbool.h>
#include <stdint.h>

/* Taille du demi-buffer (en frames stéréo) — ajustable avant l’include */
#ifndef WAV_DMA_FRAMES
#define WAV_DMA_FRAMES   1024
#endif

void WAV_Init(void);
bool WAV_Play(const char *path);   /* retourne false si erreur */
void WAV_Stop(void);
void WAV_Task(void);               /* à appeler souvent (ex: toutes 1–2 ms) */
bool WAV_IsPlaying(void);

/* Callbacks utilisés par HAL :
   void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai);
   void HAL_SAI_TxCpltCallback    (SAI_HandleTypeDef *hsai);
*/

#endif /* INC_WAV_SAI_H_ */
