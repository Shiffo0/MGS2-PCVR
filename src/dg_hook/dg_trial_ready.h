#pragma once
#include <stdint.h>
// Observed continuity, not prediction: any unsafe epoch or >500ms gap resets.
struct TrialReady {
 bool active=false;uint32_t first=0,last=0,epoch=0;unsigned samples=0;
 bool ready(uint32_t now,uint32_t generation)const{return active && epoch==generation && samples>=8 && uint32_t(last-first)>=2000 && uint32_t(now-last)<=500;}
 void reset(){active=false;samples=0;}
 bool step(uint32_t now,uint32_t stamp,uint32_t generation,bool allowed){
  if(!allowed || uint32_t(now-stamp)>500){reset();return false;}
  if(!active || epoch!=generation || uint32_t(stamp-last)>500){
   active=true;first=last=stamp;epoch=generation;samples=1;return false;
  }
  if(stamp!=last){last=stamp;++samples;}
  return samples>=8 && uint32_t(stamp-first)>=2000;
 }
};
