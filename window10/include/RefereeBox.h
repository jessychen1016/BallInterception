#include <QUdpSocket>
#include <iostream>
#include "referee.pb.h"


class RefereeBox{
public:
	RefereeBox();
	~RefereeBox();
	void BindJoint();
	void ifForceStart();
protected:
	QUdpSocket receiveSocket;


};
