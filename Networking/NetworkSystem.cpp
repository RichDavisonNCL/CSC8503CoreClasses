#include "NetworkSystem.h"

#ifdef _WIN32
#include "./enet/Win32.h"
#include "./enet/enet.h"
#endif

NetworkSystem::NetworkSystem()	{
#ifdef _WIN32
	enet_initialize();
#endif
}

NetworkSystem::~NetworkSystem()	{
#ifdef _WIN32
	ShutdownClient();
	ShutdownServer();
#endif

#ifdef _WIN32
	enet_deinitialize();
#endif
}

void NetworkSystem::UpdateNetwork() {
	ENetEvent event;
	if (IsServer()) {
		while (enet_host_service(serverHandle, &event, 0) > 0) {
			if (event.type == ENET_EVENT_TYPE_CONNECT) {
				std::cout << " Server : New client connected " << std::endl;
			}
			if (event.type == ENET_EVENT_TYPE_DISCONNECT) {
				std::cout << " A client has disconnected " << std::endl;
			}
			else if (event.type == ENET_EVENT_TYPE_RECEIVE) {
				GamePacket* packet = (GamePacket*)event.packet -> data;
				ReceivePacketFromClient(packet, event.peer->incomingPeerID);
			}
			enet_packet_destroy(event.packet);
		}
	}
	if (IsClient()) {
		while (enet_host_service(clientHandle, &event, 0) > 0) {
			if (event.type == ENET_EVENT_TYPE_CONNECT) {
				std::cout << " Client : Connected to server! " << std::endl;
			}
			else if (event.type == ENET_EVENT_TYPE_RECEIVE) {
				GamePacket* packet = (GamePacket*)event.packet->data;
				ReceivePacketFromServer(packet);
			}
			enet_packet_destroy(event.packet);
		}
	}
}

void NetworkSystem::InitialiseServer(int portNum, int maxClientCount) {
	ENetAddress address;
	address.host = ENET_HOST_ANY;
	address.port = portNum;

	serverHandle = enet_host_create(&address, maxClientCount, 1, 0, 0);

	std::cout << "Initialised server!\n";
}

void NetworkSystem::InitialiseClient(uint8_t a, uint8_t b, uint8_t c, uint8_t d, int portNum)	{
	clientHandle = enet_host_create(nullptr, 1, 1, 0, 0);

	ENetAddress address;
	address.host = (d << 24) | (c << 16) | (b << 8) | (a);
	address.port = portNum;

	clientPeer = enet_host_connect(clientHandle, &address, 2, 0);

	std::cout << "Initialised client!\n";
}

void NetworkSystem::ShutdownServer()	{
	//SendServerPacket(BasicNetworkMessages::Shutdown);
	//SendServerPacket()
	enet_host_destroy(serverHandle);
	serverHandle = nullptr;

}

void NetworkSystem::ShutdownClient()	{
	enet_peer_disconnect(clientPeer, 0);
	enet_host_destroy(clientHandle);
	clientHandle	= nullptr;
	clientPeer		= nullptr;
}

void NetworkSystem::SendClientPacket(GamePacket& packet) {
 	ENetPacket* dataPacket = enet_packet_create(&packet, packet.GetTotalSize(), 0);
	enet_peer_send(clientPeer, 0, dataPacket);
}

void NetworkSystem::BroadcastServerPacket(GamePacket& packet) {
	ENetPacket* dataPacket = enet_packet_create(&packet,packet.GetTotalSize(), 0);
	enet_host_broadcast(serverHandle, 0, dataPacket);
}

void NetworkSystem::SendServerPacket(GamePacket& packet, int toPeer) {
	ENetPacket* dataPacket = enet_packet_create(&packet, packet.GetTotalSize(), 0);
	enet_host_broadcast(serverHandle, 0, dataPacket);
}