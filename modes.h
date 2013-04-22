#include<stdio.h>
#include<iostream>
#include<string.h>
class PSM		//Power save mode
{
public: 
int level;	//level to decide mode
int PMF;		//power management field
int MPSF;		//Mesh power save field
char mode[15];		//mode (active, ls,ds)
char active_mode[7];
char deep_sleep_mode[16];
char light_sleep_mode[17];
PSM()
{
level=100;
PMF=0;
MPSF=0;
strcpy(active_mode,"active");
strcpy(deep_sleep_mode,"deep sleep mode");
strcpy(light_sleep_mode,"light sleep mode");
strcpy(mode,active_mode);			//initially all are active
}
void set_active();					//All the fuctions regarding the same.
void set_deep_sleep();
void set_light_sleep();
void set_mode_by_field();
int check_mode_by_level();
};

void PSM::set_active()
{
strcpy(mode,active_mode);
} 

void PSM::set_deep_sleep()
{
strcpy(mode,deep_sleep_mode);
} 

void PSM::set_light_sleep()
{
strcpy(mode,light_sleep_mode);
} 

void PSM::set_mode_by_field()
{
if(PMF==0&&MPSF==0)
set_active();
 if(PMF==1&&MPSF==0)
set_light_sleep();
 if(PMF==1&&MPSF==1)
set_deep_sleep();
}

int PSM::check_mode_by_level()
{
if(level<=100&&level>75)
return 0;
 if(level<=75&&level>35)
return 1;
 if(level<=35&&level>=0)
return 2;
}

