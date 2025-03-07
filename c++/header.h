#pragma onc
#include <iostream>
#include <string>
#include <boost/asio.hpp>     //lib for serial port comms, got it via vspkg and cmake, had issues wih winsock2
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <map>
#include <cstdint>
#include <array>
#include <vector>
#include <ctime>        //for convert_the_daytime()

using namespace std;
using namespace boost;

                                            // values from the original apcserial.py file, didn't check those but they should be correct
const double APC_RCV_TIMEOUT = 0.25;
const int APC_RCV_SIZE = 19;
//APC_RCV_TIMEOUT = 0.5;
const int BAUD_RATE= 9600;
const int CHAR_SIZE= 8; 
const std::array<uint8_t, 2> APC_CMD_INIT= {0xF7, 0xFD};
const std::array<uint8_t, 1> APC_CMD_BACK= {0xF7};
const std::array<uint8_t, 1> APC_CMD_RESET= {0xFD};
const std::array<uint8_t, 1> APC_CMD_NEXT= {0xFE};



enum class CommState
{
        INIT,
        INIT_RESET,
        MODE0,
        MODE1
};
class ApcComm
{
    public:
        ApcComm(boost::asio::serial_port&);
        ~ApcComm();
        bool get_state() const;
        bool get_daemon() const;
        double convert_from_bp(const std::vector<uint8_t>&, int, bool);
        bool send_apc_msg(std::array<uint8_t, 1>);
        void run();                                     //TODO
        std::vector<uint8_t> receive_msg();
        bool verify_msg_checksum(const std::vector<uint8_t>&);
        uint16_t calc_checksum(const std::vector<uint8_t>&);
        std::vector<uint8_t> create_msg_data(uint8_t, uint8_t, const std::vector<uint8_t>&);
        bool handle_apc_msg(const std::vector<uint8_t>&);
        std::vector<uint8_t> calculate_challenge();
        
        std::vector<uint8_t> convert_to_bp(float, int);
        std::string convert_to_datetime(int);
        void stop();
        
        
        std::map<string, string> ups_state;
        atomic<bool> running = false;

    private:
        std::array<uint8_t, 1> next_apc_msg;
        bool daemon = false;
        asio::io_context io;
        //asio::serial_port serial;
        thread worker_thread;
        CommState state;
        CommState previous_state;
        boost::asio::serial_port& serial_;
        std::vector<uint8_t> rcv_data;
    
};

class ApcCli
{
    public:
        ApcCli();
        ~ApcCli();
        void print_keys(string);
        void do_commstate();
        void do_voltage();
        void do_current();
        void do_frequency();
        void do_runtime();
        void do_battery();
        void do_status();
        void do_all();
        void do_set();
        void do_write();
        void do_exit();
        void send_msg();
        
    private:
        string intro= "ApcComm CLI. Type help or ? to list commands.\n";
        string prompt= "(apc) ";
        string file; 
};

class Fletcher {
    public:
        // Fletcher();              //throws undefined reference error, don't plan to implement those but it seemd good practice to define them
        // ~Fletcher();
        uint16_t fletcher8(const std::vector<uint8_t>& data);
        void update(const std::vector<uint8_t>& data);
    private:
        uint8_t cb0 = 0;
        uint8_t cb1 = 0; 
    };