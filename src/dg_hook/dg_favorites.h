/* Versioned, all-or-nothing preferences. Never serialize native inventory. */
#ifndef DG_FAVORITES_H
#define DG_FAVORITES_H
#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static int dg_favorites_load(const char *path, uint64_t out[2]) {
    FILE *f = NULL;
    unsigned char data[24];
    size_t n;
    int extra;
    if (fopen_s(&f,path,"rb") || !f) return 0;
    n=fread(data,1,sizeof data,f); extra=fgetc(f); fclose(f);
    if (n!=sizeof data || extra!=EOF || memcmp(data,"DGFAV001",8)) return 0;
    memcpy(out,data+8,16);
    return 1;
}
static int dg_favorites_save(const char *path, const uint64_t values[2]) {
    char temp[MAX_PATH];
    unsigned char data[24];
    HANDLE f;
    DWORD written=0;
    int ok;
    if (sprintf_s(temp,sizeof temp,"%s.tmp",path)<0) return 0;
    memcpy(data,"DGFAV001",8); memcpy(data+8,values,16);
    f=CreateFileA(temp,GENERIC_WRITE,0,NULL,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,NULL);
    if (f==INVALID_HANDLE_VALUE) return 0;
    ok=WriteFile(f,data,sizeof data,&written,NULL) && written==sizeof data;
    if (ok) ok=FlushFileBuffers(f)!=0;
    if (!CloseHandle(f)) ok=0;
    if (ok) ok=MoveFileExA(temp,path,MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)!=0;
    if (!ok) DeleteFileA(temp);
    return ok;
}
#endif
