#include <QUdpSocket>
#include "RefereeBox.h"
#include <iostream>
#include "referee.pb.h"
#include <thread>
namespace {
    int REFEREE_PORT = 10003;
    struct sCMD_TYPE {
        char cmd;
        unsigned int step;
    };

    struct stGamePacket {
        char cmd;
        unsigned char cmd_counter;
        unsigned char goals_blue;
        unsigned char goals_yellow;
        unsigned short time_remaining;
    };
    int VECTOR = 1;
}

RefereeBox::RefereeBox(){}
RefereeBox::~RefereeBox(){}

void RefereeBox::BindJoint(){
receiveSocket.bind(QHostAddress::AnyIPv4,REFEREE_PORT, QUdpSocket::ShareAddress);
receiveSocket.joinMulticastGroup(QHostAddress("224.5.23.1"));
}

void RefereeBox::ifForceStart(){
	SSL_Referee ssl_referee;
    QByteArray datagram;
    while( true ) {
		bool jump_out = false;
		std::cout << receiveSocket.hasPendingDatagrams() << std::endl;
        while (receiveSocket.state() == QUdpSocket::BoundState && receiveSocket.hasPendingDatagrams()) {
            datagram.resize(receiveSocket.pendingDatagramSize());
            receiveSocket.readDatagram(datagram.data(), datagram.size());
            ssl_referee.ParseFromArray((void*)datagram.data(), datagram.size());
            unsigned long long packet_timestamp = ssl_referee.packet_timestamp();
            const SSL_Referee_Stage& stage = ssl_referee.stage();
            const SSL_Referee_Command& command = ssl_referee.command();
            unsigned long command_counter = ssl_referee.command_counter();
            unsigned long long command_timestamp = ssl_referee.command_timestamp();
            const SSL_Referee_TeamInfo& yellow = ssl_referee.yellow();
            const SSL_Referee_TeamInfo& blue = ssl_referee.blue();
            long long stage_time_left = 0;
            if (ssl_referee.has_stage_time_left())
            {
                stage_time_left = ssl_referee.stage_time_left();
            }
            char cmd;
            struct stGamePacket pCmd_temp;
            unsigned char cmd_index = 0;
            switch(command) {
            case 0: cmd = 'H'; break; // Halt
            case 1: cmd = 'S'; break; // Stop
            case 2: cmd = ' '; break; // Normal start (Ready)
            case 3: cmd = 's'; break; // Force start (Start)
            case 4: cmd = 'k'; break; // Kickoff Yellow
            case 5: cmd = 'K'; break; // Kickoff Blue
            case 6: cmd = 'p'; break; // Penalty Yellow
            case 7: cmd = 'P'; break; // Penalty Blue
            case 8: cmd = 'f'; break; // Direct Yellow
            case 9: cmd = 'F'; break; // Direct Blue
            case 10: cmd = 'i'; break; // Indirect Yellow
            case 11: cmd = 'I'; break; // Indirect Blue
            case 12: cmd = 't'; break; // Timeout Yellow
            case 13: cmd = 'T'; break; // Timeout Blue
            case 14: cmd = 'g'; break; // Goal Yellow
            case 15: cmd = 'G'; break; // Goal Blue
            case 16: cmd = 'b'; break; // Ball Placement Yellow
            case 17: cmd = 'B'; break; // Ball Placement Blue
            default:
                std::cout << "refereebox is fucked !!!!! command : " << command << std::endl;
                cmd = 'H';break;
            }
			if (cmd == 's') {
				std::cout <<"ForceStart" << std::endl;
				jump_out = true;
				break;
			}
			else {
				std::cout << "FUCK No" << std::endl;
			}
        }
		if (jump_out) {
			break;
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
     }
}