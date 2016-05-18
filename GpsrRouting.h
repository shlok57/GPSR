/*********************************************************/
/*  Copyright (c) 2011. University of Pau, France        */
/*  LIUPPA Laboratory, T2I Team                          */
/*                                                       */
/*  Permission to use, copy, modify and distribute this  */
/*  code, without fee, and without written agreement is  */
/*  hereby granted, provided that the above copyright    */
/*  notice and the authors appear in all copies          */
/*                                                       */
/*  GPSR Routing Protocol                                */
/*  Version:  1.0                                        */
/*  Authors: Diop Mamour <serignemamour.diop@gmail.com>  */
/*           Congduc Pham <congduc.pham@univ-pau.fr>     */
/*********************************************************/


#ifndef _GPSRROUTING_H_
#define _GPSRROUTING_H_

#include <map>
#include "VirtualRouting.h"
#include "GpsrRoutingControl_m.h"
#include "GpsrRoutingPacket_m.h"

//******************************
//#define PI 22/7
//******************************
#define DEFAULT_GPSR_TIMEOUT   200.0   
        /* the default time out period is 200.0 sec
           If the hello messge was not received during this period
           the entry in the neighbor list may be deleted 
        */

// if something is wrong, will retry sending HELLO message GPSR_RETRY_HELLO_DELAY second later
#define GPSR_RETRY_HELLO_DELAY 1

using namespace std;

struct GPSR_neighborRecord {
	int id;      // the node's ID
	int x;       // the node's coordinates : the geographic information
	int y;
	double ts;   //the last time stamp of the hello msg from it
 	int timesRx;   

	GPSR_neighborRecord() {
	   id = 0;
       x = 0.0;
       y = 0.0;
       ts = 0.0;
	   timesRx = 0;
    }
    
};

struct sink {
       int id;          // the Sink's ID
	   int x;       // the Sink's coordonates : the geographic information
       int y;
};
    

enum GpsrRoutingTimers {
    GPSR_HELLO_MSG_REFRESH_TIMER = 0,
    GPSR_HELLO_MSG_EXPIRE_TIMER  = 1,
};

class GpsrRouting: public VirtualRouting {
 private:
    // Parameters
    int GpsrSetupFrameOverhead;	// in bytes
    double netSetupTimeout;
    bool collectTraceInfo;
    int currentSequenceNumber;
    double helloInterval;
    double activeRouteTimeout; //in s

    // GpsrRouting-related member variables
    int self;         // the node's ID
    double self_xCoo; // store the node's position in meters
    double self_yCoo;
	bool isCoordinateSet; // to know whether the node's position has been set or not
    int totalSNnodes;
    int packetsPerNode;
    bool isSink;		//is a .ned file parameter of the Application module
    sink mySink; 
    int seqHello;
	vector<GPSR_neighborRecord> neighborTable;

 protected:

    void startup();
    void finishSpecific();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
	void handleNetworkControlCommand(cMessage *);
    void sendTopologySetupPacket();
    void timerFiredCallback(int);
    void processBufferedPacket();

    void sendHelloMessage();
	void processDataPacketFromMacLayer(GpsrPacket*);

    void updateNeighborTable(int, int, int, int); // add a possible new neighbor
    int getNextHopGreedy(int, int);              // Greedy forwarding mode
    int getNextHopPerimeter(int, int);          // Perimeter mode
    double distance(int, int, int, int); //calculate distance between 2 nodes
    double norm(double );					//+++++Calculate Norm Fun
};

#endif				//GPSRROUTINGMODULE
