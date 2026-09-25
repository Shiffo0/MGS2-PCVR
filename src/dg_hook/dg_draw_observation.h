#pragma once
// Passive original-call observation. No hooks, protection changes or request
// route are installed here. An explicit scoped token opts into verification.
struct DrawObservation;
static thread_local DrawObservation*drawObservation=nullptr;
struct DrawObservation {
 void*context;void*api;unsigned kind,count,first;int base;DWORD thread;
 unsigned calls=0;bool invalid=false,inCall=false,attached=false;
 DrawObservation(void*c,void*a,unsigned k,unsigned n,unsigned f,int b):context(c),api(a),kind(k),count(n),first(f),base(b),thread(GetCurrentThreadId()){
  if(drawObservation){invalid=true;drawObservation->invalid=true;return;}drawObservation=this;attached=true;
 }
 ~DrawObservation(){if(attached&&drawObservation==this)drawObservation=nullptr;}
 DrawObservation(const DrawObservation&)=delete;DrawObservation&operator=(const DrawObservation&)=delete;
 bool good(unsigned expected)const{return attached&&!invalid&&!inCall&&calls==expected&&thread==GetCurrentThreadId();}
 bool enter(void*c,void*a,unsigned k,unsigned n,unsigned f,int b,bool authorized){
  if(inCall){invalid=true;return false;}
  if(!authorized||thread!=GetCurrentThreadId()||c!=context||a!=api||k!=kind||n!=count||f!=first||b!=base||calls>=2)invalid=true;
  ++calls;inCall=true;return true;
 }
};
template<class Call>void observedOriginal(void*c,void*a,unsigned kind,unsigned n,unsigned first,int base,bool authorized,Call call){
 auto token=drawObservation;
 if(!token){call();return;}
 bool entered=token->enter(c,a,kind,n,first,base,authorized);
 struct Finish{DrawObservation*t;bool entered,done=false;~Finish(){if(!done)t->invalid=true;if(entered)t->inCall=false;}}finish{token,entered};
 call();finish.done=true; // Refusal never suppresses the normal downstream call.
}
