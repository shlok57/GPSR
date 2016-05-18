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

#include "GpsrRouting.h"
#include "math.h"

using namespace std;

Define_Module(GpsrRouting);

//================================================================
//    startup
//================================================================
void GpsrRouting::startup(){

    self = getParentModule()->getParentModule()->getIndex();
	isCoordinateSet = false;

    totalSNnodes = getParentModule()->getParentModule()->getParentModule()->par("numNodes");
    
    trace() << "Total node = " << totalSNnodes << endl;
//****    trace() << "SQRT TRY " << atan2(7.00,-7.00)<<endl;
//****    trace() << "SQRT TRY " << norm(atan2(7.00,-7.00))<<endl;
    
    
    helloInterval = (double)par("helloInterval") / 1000.0;
    activeRouteTimeout = (double)par("activeRouteTimeout") / 1000.0;
    packetsPerNode = par("packetsPerNode");
    neighborTable.clear();
    neighborTable.reserve(totalSNnodes);
	seqHello = par("seqHello");
    
    // check that the Application module used has the boolean parameter "isSink"
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");
    
    if (appModule->hasPar("isSink"))
       isSink = appModule->par("isSink");
    else
	   isSink = false;
     
	// no info on sink at initialization
	mySink.id = -1;
	mySink.x = 0;
	mySink.y = 0;

	declareOutput("GPSR Packets received");
	declareOutput("GPSR Packets sent");
	declareOutput("GPSR Packets forwarded");

	// we will only send HELLO message if the node's coordinates are set
}

//================================================================
//    timerFiredCallback
//================================================================
void GpsrRouting::timerFiredCallback(int index){
     
    switch(index){
       
      case GPSR_HELLO_MSG_REFRESH_TIMER :{
		  sendHelloMessage();
          break;  
      }

      case GPSR_HELLO_MSG_EXPIRE_TIMER :{
		  //sendHelloMessage();
          break;  
      }
      
      default: break;
    }
}

//================================================================
//    processBufferedPacket
//================================================================

void GpsrRouting::processBufferedPacket(){
	while (!TXBuffer.empty()) {
		toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
		TXBuffer.pop();
	}
}

//================================================================
//    fromApplicationLayer
//================================================================
void GpsrRouting::fromApplicationLayer(cPacket * pkt, const char *destination){  

	GpsrPacket *dataPacket = new GpsrPacket("GPSR routing data packet", NETWORK_LAYER_PACKET);

	encapsulatePacket(dataPacket, pkt);
	dataPacket->setGpsrPacketKind(GPSR_DATA_PACKET);
	dataPacket->setSource(SELF_NETWORK_ADDRESS);
	dataPacket->setDestination(destination);

	if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
		// if broadcast, just give it to MAC layer
		trace() << "Received data from application layer, final destination: BROADCAST";
		toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
		collectOutput("GPSR Packets received", "DATA from Application (broadcast)");
		collectOutput("GPSR Packets sent", "DATA (broadcast)");
		return;
    }
    
	trace() << "Received data from application layer, final destination: " << string(destination);
    
	if (mySink.id==-1) {
		trace() << "Sink coordinates are not set by application yet for sink node " << string(destination);
		trace() << "Sorry, data will be not sent";
		return;
	}
	else
		trace() << "Sink coordinates are set by application for sink node " << mySink.id;

	// normally, the application layer has to give the coordinates of the destination
	// therefore mySink.id should be equal to atoi(destination)
	// we just need the coordinates stored in MySink.x and mySink.y
    int nextHop = getNextHopGreedy(mySink.x, mySink.y);

	// set the coordinate of the sink in the data packet so that intermediate node can get it
	dataPacket->setX_dst(mySink.x);
	dataPacket->setY_dst(mySink.y);

	if (nextHop != -1) {
		// It exists a closest neighbor to SINK => Greedy Mode
		trace() << "Send data in greedy mode, next hop node " << nextHop << ", final destination: " << string(destination);
		toMacLayer(dataPacket, nextHop);
		collectOutput("GPSR Packets received", "DATA from Application (unicast,greedy)");
		collectOutput("GPSR Packets sent", "DATA (unicast,greedy)");
		return;
    }
	else
	{
		// It don't exist any closest neighbor to SINK => Perimeter Mode
		nextHop = getNextHopPerimeter(mySink.x, mySink.y);

		if (nextHop != -1) {
			// the rule of the right hand found. It exists a neighbor
			trace() << "Send data in perimeter mode, next hop node: " << nextHop << ", final destination: " << string(destination);
			toMacLayer(dataPacket, nextHop);
			collectOutput("GPSR Packets received", "DATA from Application (unicast,perimeter)");
			collectOutput("GPSR Packets sent", "DATA (unicast,perimeter)");
			return;
		}
		else {
			trace() << "PRESENCE of hole in node " << self <<
				" when trying to forward data (from application layer) to node " << string(destination);
			delete dataPacket;
		}	
    }	
}

//================================================================
//    fromMacLayer
//================================================================
void GpsrRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
	
    GpsrPacket *netPacket = dynamic_cast <GpsrPacket*>(pkt);

	if (!netPacket)
		return;

	switch (netPacket->getGpsrPacketKind()) {
 
        // process hello msg
		case GPSR_HELLO_MSG_PACKET: {

			trace() << "Received HELLO from node " << string(netPacket->getSource()) << "(" <<
					netPacket->getX_src() << "," << netPacket->getY_src()<< ")";

			collectOutput("GPSR Packets received", "HELLO");

			updateNeighborTable(atoi(netPacket->getSource()), seqHello, netPacket->getX_src(),netPacket->getY_src());
			break;
		}
        
        // process sink address msg
		case GPSR_SINK_ADDRESS_PACKET:{ 
			//processSinkAddress(netPacket);
			break;
		}
        // process data packet
		case GPSR_DATA_PACKET:{ 
 
			collectOutput("GPSR Packets received", "DATA from MAC");

			string dst(netPacket->getDestination());
			string src(netPacket->getSource());

			if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0))
				trace() << "Received data from node " << src << " by broadcast";
			else
				trace() << "Received data from node " << src << ", final destination: " << dst;

			processDataPacketFromMacLayer(netPacket);
			break;
		}

        default: return;
	}
}

//================================================================
//    finishSpecific
//================================================================
void GpsrRouting::finishSpecific() {

}

//================================================================
//    sendHelloMsg
//================================================================
void GpsrRouting::sendHelloMessage(){

    GpsrPacket *helloMsg = new GpsrPacket("GPSR hello message packet", NETWORK_LAYER_PACKET);
    helloMsg->setGpsrPacketKind(GPSR_HELLO_MSG_PACKET);
    helloMsg->setX_src(self_xCoo);		
    helloMsg->setY_src(self_yCoo);
    helloMsg->setSource(SELF_NETWORK_ADDRESS);
    helloMsg->setDestination(BROADCAST_NETWORK_ADDRESS);
    toMacLayer(helloMsg, BROADCAST_MAC_ADDRESS);

	trace() << "Broadcast HELLO(" << self_xCoo << "," << self_yCoo << ") seq=" << seqHello;

	collectOutput("GPSR Packets sent", "HELLO");

	seqHello++;
	setTimer(GPSR_HELLO_MSG_REFRESH_TIMER, helloInterval);
}

//================================================================
//    processDataPacket
//================================================================
void GpsrRouting::processDataPacketFromMacLayer(GpsrPacket* pkt){

    string dst(pkt->getDestination());
    string src(pkt->getSource());

	// if the node is the destination
	if ((dst.compare(SELF_NETWORK_ADDRESS) == 0) || (self == mySink.id)) {
		trace() << "Received data for myself (routing unicast) from MAC, send data to application layer. Source node: " << src;
		collectOutput("GPSR Packets received", "final from MAC");
#ifdef DEBUG_OUTPUT_LEVEL2		
		collectOutput("GPSR Packets received", atoi(src.c_str()), "final from MAC");
#endif		
		toApplicationLayer(pkt->decapsulate());
		return;
    } 

	// if the node is the destination by broadcast, we do not forward it
	if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
		trace() << "Received data (routing broadcast) from MAC, send data to application layer. Source node: " << src;
		collectOutput("GPSR Packets received", "broadcast from MAC");
#ifdef DEBUG_OUTPUT_LEVEL2
		collectOutput("GPSR Packets received", atoi(src.c_str()), "broadcast from MAC");
#endif
		toApplicationLayer(pkt->decapsulate());
		return;
	}

	// otherwise, the node has received a message in unicast (through MAC layer) but is not the final destination
	// need to find how to reach the final destination which coordinates' are stored in the incoming packet
	int nextHop = getNextHopGreedy(pkt->getX_dst(), pkt->getY_dst());

	// duplicate the packet because we are going to forward it
	GpsrPacket *netPacket = pkt->dup();

	if (nextHop != -1) {
		// It exists a closest neighbor to SINK => Greedy Mode
		trace() << "Received data (routing unicast) from MAC, forward data in greedy mode. Source node: " << src
				<< ", next hop node: " << nextHop << ", final destination: " << dst;
        toMacLayer(netPacket, nextHop);
		collectOutput("GPSR Packets forwarded","greedy");
        return;
     }
	 else {
		// It don't exist any closest neighbor to SINK => Perimeter Mode
		nextHop = getNextHopPerimeter(pkt->getX_dst(), pkt->getY_dst());

		if (nextHop != -1) {
			// the rule of the right hand  found. It exists a neighbor
			trace() << "Received data from MAC, forward data in perimeter mode. Source node: " << src
				<< ", next hop node: " << nextHop << ", final destination: " << dst;
			toMacLayer(netPacket, nextHop);
			collectOutput("GPSR Packets forwarded","perimeter");
			return;
        }
        else
			trace() << "PRESENCE of hole in node " << self <<
				" when trying to forward data (from MAC) from node " << src << " to node " << dst;
     }		          
}

//================================================================
//    updateNeighborTable
//================================================================
void GpsrRouting::updateNeighborTable(int nodeID, int theSN, int x_node, int y_node) {

	int pos = -1;
	int tblSize = (int)neighborTable.size();
    
	for (int i = 0; i < tblSize; i++)
		if (neighborTable[i].id == nodeID) {
			pos = i;
			break;
        }

	// it's a new neighbor
	if (pos == -1) {
		GPSR_neighborRecord newRec;

		newRec.id = nodeID;
		newRec.x = x_node;
		newRec.y = y_node;
		newRec.ts = simTime().dbl();
		newRec.timesRx = 1;

	    neighborTable.push_back(newRec);

		// print the last item
		trace() << "New neighbor for node " << self << " : node "<< neighborTable[(int)neighborTable.size()-1].id;

		/*trace() << "id:" << neighborTable[pos].id << " x:" << neighborTable[pos].x << " y:" << neighborTable[pos].y << " timestamp:" << neighborTable[pos].ts << " times Rx:" << neighborTable[pos].timesRx << " received packets:" << neighborTable[pos].receivedPackets << endl;
                */
	} else {

		//it's an already known neighbor
		neighborTable[pos].x = x_node; // updating of location
		neighborTable[pos].y = y_node;
		neighborTable[pos].ts = simTime().dbl();
		neighborTable[pos].timesRx++;
	}
 
	trace() << "Neighbors list of node " << self << ":";

	tblSize = (int)neighborTable.size();

	for (int j = 0; j < tblSize; j++)
		trace() << "Node " << neighborTable[j].id << "(" << neighborTable[j].x << "," << neighborTable[j].y <<
				"). Received " << neighborTable[j].timesRx << " HELLO from it.";

	trace() << "--------------";
}

//================================================================
//   getNextHopGreedy
//================================================================
int GpsrRouting::getNextHopGreedy(int x_Sink, int y_Sink){
  
   int nextHop = -1; double dist = 0;
   int tblSize = (int)neighborTable.size();
	//**************************
	trace() << "here in greedy";
	
	//**************************
	
   //initializing the minimal distance as my distance to sink
   trace() << "Node "<< self << "(" << self_xCoo << "," << self_yCoo << ")";

   double minDist = distance(self_xCoo, self_yCoo, x_Sink, y_Sink);
   //trace()<< "Distance ("<< self <<", Sink)" << " = " << minDist << endl; 

   for (int i = 0; i < tblSize; i++) {
		dist = distance(neighborTable[i].x, neighborTable[i].y, x_Sink, y_Sink);
		trace() << "Distance ("<< neighborTable[i].id <<", Sink)" << " = " << dist;

		if (dist < minDist) {
			minDist = dist;
			nextHop = neighborTable[i].id;
		}
   }

   return nextHop;
}

//================================================================
//    getNextHopPerimeter
//================================================================
int GpsrRouting::getNextHopPerimeter(int x_Sink, int y_Sink) {
    // NOT IMPLEMENTED YET
//**********************
	trace() <<"perimeter";
	int nextHop = -1; double dist = 0;
	int tblSize = (int)neighborTable.size();
	int bin,dmin,db,ba;	
		
	bin = norm(atan2(self_yCoo - y_Sink ,self_xCoo - x_Sink));
	dmin = 3*PI;
	for (int i = 0; i < tblSize; i++) {
		if(neighborTable[i].id == mySink.id )
		{
			continue;
		}
		ba = norm(atan2(self_yCoo - neighborTable[i].y ,self_xCoo - neighborTable[i].x));
		db = norm(ba - bin);
		if( db < dmin )
		{
			dmin = db;
			nextHop = neighborTable[i].id;
		}
		return nextHop;
	} 
	
//**********************
    return -1;
}

//================================================================
//    distance
//================================================================
double GpsrRouting::distance(int x1, int y1, int x2, int y2) {

    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));	
}

//**********************************
//    NORF Fun
//**********************************
double GpsrRouting::norm(double rad) {

	while(rad < 0)
	{
		rad = rad + (2*PI);
	}
	while(rad > (2*PI))
	{
		rad = rad - (2*PI);
	}
	return rad;
}

// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void GpsrRouting::handleNetworkControlCommand(cMessage *msg) {

	GpsrRoutingControlCommand *cmd = check_and_cast <GpsrRoutingControlCommand*>(msg);

	switch (cmd->getGpsrRoutingCommandKind()) {

		case SET_GPSR_NODE_POS: {

			self_xCoo = cmd->getDouble1();
			self_yCoo = cmd->getDouble2();
			isCoordinateSet = true;

			trace() << "Application layer has set node's position for to (" << self_xCoo << "," << self_yCoo << ")";

			// normally, this is the first HELLO message
			if (isCoordinateSet) {
				  sendHelloMessage();
			 }

			break;
		}

		case SET_GPSR_SINK_POS: {

			mySink.x = (int)cmd->getDouble1();
			mySink.y = (int)cmd->getDouble2();
			mySink.id = cmd->getInt1();

			trace() << "Application layer has set sink's position for next transferts SINK_" << mySink.id << "(" << mySink.x << ","
					<< mySink.y << ")";

			break;
		}
	}
	// don't delete the message since it will get deleted by the VirtualRouting class
}
