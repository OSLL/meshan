#include<iostream>
#include "modes.h"
#include<stdlib.h>
#include<stdio.h>
#define INFINITY 1000
#define MAXNODES 5
using namespace std;
int dist[MAXNODES][MAXNODES]={{0,1,15,11,14},{20,0,INFINITY,22,10},{22,INFINITY,0,40,12},{5,16,11,0,4},{INFINITY,23,13,3,0}};
//int dist[MAXNODES][MAXNODES]={{0,10,5,11,14},{20,0,INFINITY,22,10},{22,INFINITY,0,4,12},{6,16,11,0,4},{INFINITY,23,13,4,0}};

int main()
{
    PSM p[MAXNODES];
    int r,i,j,s,min;
    //randomize();
    /*for(i=0;i<MAXNODES;i++)
{
p[i].set_active();       //setting all modes by default to active mode
}*/
    /*
for(i=0;i<MAXNODES;i++)
{
r=rand()%MAXNODES;
p[r].PMF=1;
p[r].MPSF=1;
p[r].set_mode_by_field();	//setting variable indexed by r to deep sleep
s=rand()%MAXNODES;
p[s].PMF=1;
p[s].MPSF=0;
p[s].set_mode_by_field(); 	//setting variable indexed by s to light sleep
}*/
    for(i=0;i<MAXNODES;i++)
    {
        for(j=0;j<MAXNODES;j++)
        {
            if(i!=j)

            {
                if((dist[i][j])>15)
                {	p[j].set_deep_sleep();  }
                else if(((dist[i][j])>7)&&((dist[i][j])<=15))
                {	p[j].set_light_sleep(); }
                else
                {	p[j].set_active(); }
                cout<<"\n"<<i<<" -> "<<j<<"\tdistance = "<<dist[i][j]<<"\tmode is\t "<<p[j].mode;
            }
        }
    }
    /*for(i=0;i<MAXNODES;i++)
{
cout<<"\n"<<i<<" -> "<<j<<" distance = "<<dist[i][j]<<" mode is "<<p[i].mode;
cout<<"\nfor node \t"<<i+1;
cout<<"	 Mode:\t"<<p[i].mode<<endl;	//print active,deep sleep or light sleep mode of a node 
}*/
}



