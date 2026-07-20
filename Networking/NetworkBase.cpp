#include "NetworkBase.h"

#ifdef _WIN32
#include "./enet/enet.h"
#endif

NetworkBase::NetworkBase()	{
//	netHandle = nullptr;
}

NetworkBase::~NetworkBase()	{
#ifdef _WIN32
	//if (netHandle) {
	//	enet_host_destroy(netHandle);
	//}
#endif
}

void NetworkBase::Initialise() {
#ifdef _WIN32
//	enet_initialize();
#endif
}

void NetworkBase::Destroy() {
#ifdef _WIN32
//	enet_deinitialize();
#endif
}

bool NetworkBase::ProcessPacket(GamePacket* packet, int peerID) {
	return false;
}