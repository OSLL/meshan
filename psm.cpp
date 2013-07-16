#include<iostream>
#include "modes.h"
#include<stdlib.h>
#include<stdio.h>
using namespace std;
int main()
{
    PSM p[10];
    int r,i,s;
    //randomize();
    for(i=0;i<4;i++)
    {
        r=rand()%10;
        p[r].PMF=1;
        p[r].MPSF=1;
        p[r].set_mode_by_field();	//setting variable indexed by r to deep sleep
        s=rand()%10;
        p[s].PMF=1;
        p[s].MPSF=0;
        p[s].set_mode_by_field(); 	//setting variable indexed by s to light sleep
    }

    for(i=0;i<10;i++)
    {
        cout<<"\nfor node \t"<<i+1;
        cout<<"	 Mode:\t"<<p[i].mode<<endl;	//print active,deep sleep or light sleep mode of a node
    }
}
