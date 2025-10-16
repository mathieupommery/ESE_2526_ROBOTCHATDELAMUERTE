#include "wav_sai.h"
#include "fatfs.h"
#include "sai.h"     /* doit exposer extern SAI_HandleTypeDef hsai_BlockA3; */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ===== Configuration ===== */
static int16_t s_dma[WAV_DMA_FRAMES*2*2];   /* 2 demi-bufs × (L,R) */
static volatile uint8_t s_need0=0, s_need1=0;

static FIL      s_f;
static uint32_t s_left=0;        /* bytes restants dans "data" */
static uint16_t s_ch=0;          /* 1 ou 2 */
static bool     s_play=false;

/* ===== Helpers compacts ===== */
static bool rd(void *p, UINT n){ UINT br; return (f_read(&s_f,p,n,&br)==FR_OK && br==n); }

static bool find_data(uint32_t *sz){
  struct __attribute__((packed)){char id[4]; uint32_t sz;} ch;
  for(;;){
    if(!rd(&ch,sizeof ch)) return false;
    if(!memcmp(ch.id,"data",4)){ *sz=ch.sz; return true; }
    UINT skip=ch.sz + (ch.sz&1);
    if(f_lseek(&s_f, f_tell(&s_f)+skip)!=FR_OK) return false;
  }
}

static bool fill_half(int half){
  int16_t *dst = &s_dma[(half?1:0)*WAV_DMA_FRAMES*2];
  uint32_t frames = WAV_DMA_FRAMES;
  while(frames){
    if(!s_left){ memset(dst,0,frames*2*sizeof(int16_t)); break; }
    int16_t tmp[2048]; /* jusqu'à 1024 frames stéréo */
    uint32_t need_samples = (s_ch==1)? frames : frames*2;
    if(need_samples>2048) need_samples=2048;
    UINT to_bytes = need_samples*2; UINT br=0;
    if(f_read(&s_f,tmp,to_bytes,&br)!=FR_OK) return false;
    s_left -= br; uint32_t got = br/2;
    if(s_ch==1){ uint32_t gf=got; for(uint32_t i=0;i<gf;i++){ int16_t m=tmp[i]; *dst++=m; *dst++=m; } frames-=gf; }
    else       { uint32_t gf=got/2; memcpy(dst,tmp,gf*2*sizeof(int16_t)); dst+=gf*2; frames-=gf; }
    if(!br){ memset(dst,0,frames*2*sizeof(int16_t)); break; }
  }
  return true;
}

/* ===== API ===== */
void WAV_Init(void){ s_play=false; s_need0=s_need1=0; }

bool WAV_Play(const char *path){
  if(s_play) return false;
  if(f_open(&s_f,path,FA_READ)!=FR_OK) return false;

  /* entête WAV minimal (PCM 16-bit) */
  struct __attribute__((packed)){
    char RIFF[4]; uint32_t cs; char WAVE[4];
    char fmt_[4]; uint32_t fsz; uint16_t fmt; uint16_t ch;
    uint32_t fs;  uint32_t br;  uint16_t ba;  uint16_t bps;
  } h;
  if(!rd(&h,sizeof h) || memcmp(h.RIFF,"RIFF",4) || memcmp(h.WAVE,"WAVE",4) ||
     memcmp(h.fmt_,"fmt ",4) || h.fmt!=1 || h.bps!=16 || (h.ch!=1 && h.ch!=2)){
    f_close(&s_f); return false;
  }
  s_ch=h.ch; uint32_t dataSz=0; if(!find_data(&dataSz)){ f_close(&s_f); return false; }
  s_left=dataSz;

  /* pré-remplir 2 demi-bufs, puis DMA circulaire */
  if(!fill_half(0) || !fill_half(1)){ f_close(&s_f); return false; }
  s_need0=s_need1=0;
  HAL_SAI_Transmit_DMA(&hsai_BlockA3, (uint8_t*)s_dma,
                       WAV_DMA_FRAMES*2 /*L+R*/ *2 /*demis*/);
  s_play=true; return true;
}

void WAV_Stop(void){
  if(!s_play) return;
  HAL_SAI_DMAStop(&hsai_BlockA3);
  f_close(&s_f);
  s_play=false; s_need0=s_need1=0; s_left=0;
}

bool WAV_IsPlaying(void){ return s_play; }

/* à appeler souvent (1–2 ms) dans la loop ou un thread */
void WAV_Task(void){
  if(!s_play) return;
  if(s_need0){ s_need0=0; if(!fill_half(0)){ WAV_Stop(); return; } }
  if(s_need1){ s_need1=0; if(!fill_half(1)){ WAV_Stop(); return; } }

  /* option: arrêt auto quand fini */
  if(!s_left && !s_need0 && !s_need1){
    WAV_Stop();
  }
}

/* ===== Callbacks HAL ===== */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){ if(hsai==&hsai_BlockA3) s_need0=1; }
void HAL_SAI_TxCpltCallback    (SAI_HandleTypeDef *hsai){ if(hsai==&hsai_BlockA3) s_need1=1; }
