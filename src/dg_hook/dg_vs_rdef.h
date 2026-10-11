#pragma once
/* Where does a vertex shader keep its camera? The game's world shaders name it
 * gVS_Pers; its offset in the constant buffer bound at b0 differs per shader
 * (c16, c20, c24 in the 18 Sep table). Read it from the shader's own reflection
 * data (DXBC chunk RDEF) instead of a hand-kept table, so shaders the table has
 * never seen are handled too. Pure parsing, bounds-checked, no allocation.
 *   >= 0  constant index of gVS_Pers in the b0 buffer
 *   -1    no gVS_Pers (screen-space shader: never retargeted)
 *   -2    gVS_Pers lives in a buffer not bound at b0
 *   -3    not a readable DXBC/RDEF */
#include <stdint.h>
#include <string.h>

static int dgVsPersIndex(const unsigned char*code,size_t size){
 auto u32=[&](size_t o,uint32_t&v)->bool{if(o+4>size||o+4<o)return false;memcpy(&v,code+o,4);return true;};
 if(!code||size<32||memcmp(code,"DXBC",4))return -3;
 uint32_t chunks=0;if(!u32(28,chunks)||chunks>64)return -3;
 size_t rdef=0,rsize=0;
 for(uint32_t i=0;i<chunks;i++){uint32_t off=0,fourcc=0,len=0;
  if(!u32(32+4*i,off)||!u32(off,fourcc)||!u32(off+4,len))return -3;
  if(!memcmp(code+off,"RDEF",4)){rdef=(size_t)off+8;rsize=len;break;}}
 if(!rdef||rdef+rsize>size||rsize<28)return -3;
 const unsigned char*r=code+rdef;
 auto ru32=[&](size_t o,uint32_t&v)->bool{if(o+4>rsize)return false;memcpy(&v,r+o,4);return true;};
 auto name=[&](uint32_t o,const char*want)->bool{size_t n=strlen(want);return o<rsize&&rsize-o>n&&!memcmp(r+o,want,n+1);};
 uint32_t cbCount=0,cbOff=0,bindCount=0,bindOff=0,version=0;
 if(!ru32(0,cbCount)||!ru32(4,cbOff)||!ru32(8,bindCount)||!ru32(12,bindOff)||!ru32(16,version))return -3;
 unsigned major=(version>>8)&0xff;const size_t varSize=major>=5?40:24;
 if(cbCount>64||bindCount>256)return -3;
 for(uint32_t b=0;b<cbCount;b++){
  size_t cb=cbOff+24ull*b;uint32_t cbName=0,varCount=0,varOff=0;
  if(!ru32(cb,cbName)||!ru32(cb+4,varCount)||!ru32(cb+8,varOff)||varCount>4096)return -3;
  for(uint32_t v=0;v<varCount;v++){
   size_t var=varOff+varSize*v;uint32_t vName=0,start=0;
   if(!ru32(var,vName)||!ru32(var+4,start))return -3;
   if(!name(vName,"gVS_Pers"))continue;
   if(start%16)return -3;
   /* The buffer must be the one bound at slot b0. */
   if(cbName>=rsize)return -3;
   const char*cbn=(const char*)r+cbName;size_t cbl=strnlen(cbn,rsize-cbName);if(cbl==rsize-cbName)return -3;
   for(uint32_t k=0;k<bindCount;k++){size_t bd=bindOff+32ull*k;uint32_t bName=0,type=0,point=0;
    if(!ru32(bd,bName)||!ru32(bd+4,type)||!ru32(bd+20,point))return -3;
    if(type==0&&bName<rsize&&rsize-bName>cbl&&!memcmp(r+bName,cbn,cbl+1))return point==0?(int)(start/16):-2;}
   return -2;
  }
 }
 return -1;
}
