#include "leuze_rsl_driver/data_type.hpp"
#include "leuze_rsl_driver/communication.hpp"

//Need a class where the LaserScan msg is populated by ScanData
//Probably same class should parse buffer to header and scan data

template <typename ConnectionType>
class HardwareInterface
{
public:
    HardwareInterface(std::string address, std::string port, DataParser *parser)
    {
        this->connection = new ConnectionType(address, port);
        this->parser = parser; //probably pass mutex, and conditional var?

        connection->set_handle_read(&DataParser::parseBuffer, parser);
    }

    void connect()
    {
        connection->connect();
        connection->start_read(1500);
    }

    void disconnect()
    {
    }

    bool isConnected()
    {
        return true;
    }

    //device specific interface should take care of feeding watchdog, etc
    void get_scan()
    {
        std::cout << "returning scan" << std::endl;
    }

protected:
    Connection *connection;
    DataParser *parser;
};
