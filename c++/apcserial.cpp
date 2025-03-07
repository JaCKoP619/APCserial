/*
This code is an c++ implementation of the python code attempting to connect to APC smart-UPS
via serial port. It is based on klaasdc code available at https://github.com/klaasdc/apcups-serial-test/tree/master
The code is intended to be used with the APC Smart-UPS 750, but it should work with other models from this product line.
Also, the guy did try to document the reverse engineering process here: https://sites.google.com/site/klaasdc/apc-smartups-decode

@author: W0lsZcZ@n

TODO:
[✔]    Implement the Boost/asio lib, mitigate the winsock2 errors
[✔]    Learn how to use asio for serial comms  
[✔]    implemnt serial comm setup
[]      implement comm read&write
[✔]    implement comm close 
[✔]    setup gettng the comm parameter from the CLI parameter, setup some basic error detection at this stage
[✔]    APCComm threading: start and exit 
[✔]    finish ApcComm::Run()
[]      finish ApcComm::handle_apc_msg(), mostly conditions rewriting
[]      ApcCli class


*/
#include "header.h"



void check_if_serial_port_given(int argc, char* argv[])  
{
   if(argc!=2) {
        cout << "Usage: " << argv[0] << " <serial port>" << endl;
        exit(1);}
} 


int main(int argc, char* argv[])                
{
    check_if_serial_port_given(argc, argv);



    //serial connection options, line 966 in .py file, with try-catch block added
    boost::asio::io_context io_service;
    boost::asio::serial_port serial(io_service);
    
    try {
        serial.open(argv[1]);
        serial.set_option(asio::serial_port_base::baud_rate(BAUD_RATE));
        serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial.set_option(asio::serial_port_base::character_size(CHAR_SIZE));
        serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
        
        }
    catch (std::exception& e) {
        cerr << "Error opening serial port: " << e.what() << endl;
        exit(1);
    }


    try {  
        if (serial.is_open()) {
            std::cout << "Serial port opened successfully.\n";
        } else {
            std::cerr << "Failed to open serial port.\n";
        }
        } catch (const boost::system::system_error& e) {
            std::cerr << "Error: " << e.what() << '\n';
            }
    
    ApcComm apcComm(serial);


    while(apcComm.running){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

}
    
    // apcComm.running=false;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    return 0;

}