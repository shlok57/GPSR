//*********************************************************/
//*  Copyright (c) 2011. University of Pau, France        */
//*  LIUPPA Laboratory, T2I Team                          */
//*                                                       */
//*  Permission to use, copy, modify and distribute this  */
//*  code, without fee, and without written agreement is  */
//*  hereby granted, provided that the above copyright    */
//*  notice and the authors appear in all copies          */
//*                                                       */
//*  GPSR Routing Protocol                                */
//*  Version:  1.0                                        */
//*  Authors: Diop Mamour <serignemamour.diop@gmail.com>  */
//*           Congduc Pham <congduc.pham@univ-pau.fr>     */
//*********************************************************/


package node.communication.routing.gpsrRouting;

simple GpsrRouting like node.communication.routing.iRouting {
 parameters:
	bool collectTraceInfo = default (false);
	int maxNetFrameSize = default (0);					// bytes, 0 means no limit
    int netDataFrameOverhead = default (14);			// bytes
	int netBufferSize = default (32);					// number of messages

	int GpsrSetupFrameOverhead = default (13);	        // bytes
	//int netSetupTimeout = default (50);	
    int seqHello = default(0);				// msec
   
    int helloInterval = default(2500);                  // in ms
    int activeRouteTimeout = default(60000); //ACTIVE_ROUTE_TIMEOUT in ms
    int packetsPerNode = default (100);
 gates:
	output toCommunicationModule;
	output toMacModule;
	input fromCommunicationModule;
	input fromMacModule;
	input fromCommModuleResourceMgr;
}

