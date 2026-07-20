#pragma once
//#include "./enet/enet.h"
struct _ENetHost;
struct _ENetPeer;
struct _ENetEvent;


namespace NCL {
	namespace CSC8503 {
		enum BasicNetworkMessages {
			None,
			Hello,
			Message,
			String_Message,
			Delta_State,	//1 byte per channel since the last state
			Full_State,		//Full transform etc
			Received_State, //received from a client, informs that its received packet n
			Player_Connected,
			Player_Disconnected,
			Shutdown
		};

		struct GamePacket {
			short size;
			short type;

			GamePacket() {
				type = BasicNetworkMessages::None;
				size = 0;
			}

			GamePacket(short type) : GamePacket() {
				this->type = type;
			}

			int GetTotalSize() {
				return sizeof(GamePacket) + size;
			}
		};

		class NetworkSystem {
		public:
			static int GetDefaultPort() {
				return 1234;
			}

		protected:
			NetworkSystem();
			~NetworkSystem();

			void InitialiseServer(int portNum, int maxClientCount);
			void InitialiseClient(uint8_t a, uint8_t b, uint8_t c, uint8_t d, int portNum);

			void ShutdownServer();
			void ShutdownClient();

			void SendClientPacket(GamePacket& p);

			void BroadcastServerPacket(GamePacket& p);
			void SendServerPacket(GamePacket& p, int toPeer);

			virtual void ReceivePacketFromClient(GamePacket* payload, int source = -1) = 0;
			virtual void ReceivePacketFromServer(GamePacket* payload) = 0;

			void UpdateNetwork();

			bool IsServer() const { return serverHandle; }
			bool IsClient() const { return clientHandle; }

		private:
			int			port = 0;

			//Client Specifics
			_ENetHost* clientHandle = nullptr;
			_ENetPeer* clientPeer = nullptr;

			//Server specifics
			_ENetHost* serverHandle = nullptr;
			int			clientMax = 0;
			int			clientCount = 0;

		};
	}
}