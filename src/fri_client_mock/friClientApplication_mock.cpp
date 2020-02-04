/*
 * friClientApplication_mock.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: rosdeveloper
 */

#include "fri_client/friClientApplication.h"

namespace KUKA
{
namespace FRI
{

class IClient
{
};
class TransformationClient
{
};
class IConnection
{
};
struct ClientData
{
};

ClientApplication::ClientApplication(IConnection &connection, IClient &client) :
    _connection(), _robotClient(), _trafoClient(), _data()
{

}

ClientApplication::ClientApplication(IConnection &connection, IClient &client, TransformationClient &trafoClient) :
    _connection(), _robotClient(), _trafoClient(), _data()
{

}

ClientApplication::~ClientApplication()
{

}

bool ClientApplication::connect(int port, const char *remoteHost = NULL)
{
  return false;
}

void ClientApplication::disconnect()
{
  return;
}

bool ClientApplication::step()
{
  return false;
}
}
}
