#include "header.h"

using namespace std;
using namespace boost;

//python's code object "s" ex. s. is "copy" of serial port, we deal with it by reference, serial_ .
ApcComm::ApcComm(boost::asio::serial_port& serial)
:serial_(serial) {        //assigning const variables in the member initialisation list at the construction stage, it's not possible otherwise
    state= CommState::INIT;
    previous_state= CommState::INIT;
    next_apc_msg= APC_CMD_NEXT;
    daemon= true;
    ups_state["comm_state"]= "offline";
    

    running= true;
    worker_thread = std::thread(&ApcComm::run, this);



}
ApcComm::~ApcComm() {
    stop();
    if(worker_thread.joinable())
        worker_thread.join();
}

bool ApcComm::send_apc_msg(std::array<uint8_t, 1> raw_msg){
    if(state!= CommState::MODE1){
        return false;
    }
    next_apc_msg= raw_msg;
    while(next_apc_msg != APC_CMD_NEXT){
        std::this_thread::sleep_for(chrono::duration<double>(APC_RCV_TIMEOUT));
    }
    return true;
}
void ApcComm::run(){                                                                
    while(running){
        switch(state){
            case CommState::INIT:{                  //initialize the communication
                asio::write(serial_, asio::buffer(APC_CMD_INIT));
                rcv_data= receive_msg();

                if(!handle_apc_msg(rcv_data))
                    state= CommState::INIT_RESET;
                else
                    state= CommState::MODE0;
                break; 
            }
            case CommState::INIT_RESET:{            //Reset the UPS communication
                
                this_thread::sleep_for(chrono::seconds(1));
                boost::asio::write(serial_, boost::asio::buffer(APC_CMD_RESET));
                rcv_data= receive_msg();
                
                if(!handle_apc_msg(rcv_data))
                    state= CommState::INIT;
                else
                    state= CommState::MODE0; 
                break;
                }
            case CommState::MODE0:{                 //Normal communication flow with UPS according to MODE0
                boost::asio::write(serial_, boost::asio::buffer(next_apc_msg));
                rcv_data= receive_msg();
                if(!handle_apc_msg(rcv_data))
                    state= CommState::INIT;         
                break;
            }
            case CommState::MODE1:{                 //Normal communication flow with UPS according to MODE1
                boost:asio::write(serial_, boost::asio::buffer(next_apc_msg));
                rcv_data= receive_msg();

                if(!next_apc_msg.empty())
                    next_apc_msg= APC_CMD_NEXT;

                if(!handle_apc_msg(rcv_data))
                    state= CommState::INIT;
                break;
            }
            default:{
                if (state!= previous_state)
                    if(state== CommState::MODE0 || state== CommState::MODE1)
                        ups_state["comm_state"]= "online";
                    else
                        ups_state["comm_state"]= "offline";
                break;
            }
        }
        previous_state= state;
    }

}
std::vector<uint8_t> ApcComm::receive_msg(){
    std::vector<uint8_t> data;
    auto curTime = chrono::steady_clock::now();
    while(chrono::duration<double>(chrono::steady_clock::now()- curTime).count()< APC_RCV_TIMEOUT ){
        std::vector<uint8_t> buffer(APC_RCV_SIZE);              // temp buffer
        boost::asio::read(serial_, boost::asio::buffer(buffer, APC_RCV_SIZE));
        data.insert(data.end(), buffer.begin(), buffer.end());
    }
    return data;
}
void ApcComm::stop() {
    running = false;
    //io.stop();
}
bool ApcComm::verify_msg_checksum(const std::vector<uint8_t>& raw_msg) {
    Fletcher f8;
    // Extract the last 2 bytes for the message checksum
    uint16_t msg_chksum = (raw_msg[raw_msg.size() - 2] << 8) | raw_msg[raw_msg.size() - 1];
    uint16_t checksum = f8.fletcher8(raw_msg);
    return msg_chksum == checksum;
}
uint16_t ApcComm::calc_checksum(const std::vector<uint8_t>& raw_msg) {
    Fletcher f8;
    return f8.fletcher8(raw_msg);
}
std::vector<uint8_t> ApcComm::create_msg_data(uint8_t msg_id, uint8_t offset, const std::vector<uint8_t>& msg_data){
    std::vector<uint8_t> raw_msg;
    Fletcher f8;

    raw_msg.push_back(msg_id);
    raw_msg.push_back(offset);
    raw_msg.push_back(static_cast<uint8_t>(msg_data.size()));               // Length as a single byte

                // Append msg_data to raw_msg
    raw_msg.insert(raw_msg.end(), msg_data.begin(), msg_data.end());
                //checksumming
    f8.update(raw_msg);
    uint16_t checksum= f8.fletcher8(raw_msg);
                //appennd msg, Big-endian format
    raw_msg.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF));            // High byte
    raw_msg.push_back(static_cast<uint8_t>(checksum & 0xFF));                   // Low byte
    
    return raw_msg;
}
bool ApcComm::handle_apc_msg(const std::vector<uint8_t>& raw_msg){
    if(!raw_msg.empty()){
        uint8_t msg_id = raw_msg[0];
        vector<uint8_t> msg_data(raw_msg.begin() + 1, raw_msg.end() - 2);

        if(!verify_msg_checksum(raw_msg)){
            next_apc_msg= APC_CMD_BACK;
            return true;
        }
                                               //ID data via msg_id variable
        switch(msg_id){
            case 0x00:{
                ups_state["protocol_version"]= to_string(msg_data[0]);
                ups_state["msg_size"]= to_string(msg_data[1]);
                ups_state["num_ids"]= to_string(msg_data[2]);

                vector<uint8_t> series_id_bytes(msg_data.begin() + 3, msg_data.begin() + 5);
                uint16_t series_id= (static_cast<uint16_t>(series_id_bytes[0]) << 8) | series_id_bytes[1];
                ups_state["series_id"]= to_string(series_id);
            
                std::ostringstream oss;
                for (auto byte : series_id_bytes) {
                oss << std::hex << static_cast<int>(byte); }
                ups_state["series_id_raw"] = oss.str();

                ups_state["series_data_version"]= to_string(msg_data[5]);
                ups_state["unknown_3"]= to_string(msg_data[6]);
                ups_state["unknown_4"]= to_string(msg_data[7]);
                vector<uint8_t> header_raw_bytes(msg_data.begin(), msg_data.begin() + 8);
                string header_raw_bytes_str(header_raw_bytes.begin(), header_raw_bytes.end());
                ups_state["header_raw"]= header_raw_bytes_str;
                break;  
            }
            case 0x40:{
                //serial_nb_raw has no sense in c++ I think, due to the ups_state datatype i need to convert it to string anyway, kept it for the sake of consistency with the python code
                string serial_nb(msg_data.begin(), msg_data.begin()+ 14);
                ups_state["serial_nb"]= serial_nb;
                std::vector<uint8_t> serial_nb_raw(msg_data.begin(), msg_data.begin() + 14);
                string serial_nb_raw_str(serial_nb_raw.begin(), serial_nb_raw.end());
                ups_state["serial_nb_raw"] = serial_nb_raw_str;
                uint16_t production_date_raw = (static_cast<uint16_t>(msg_data[14]) << 8) | static_cast<uint16_t>(msg_data[15]);
                ups_state["production_date"] = convert_to_datetime(production_date_raw);
                break; 
            }
            case 0x41:{
                string ups_type(msg_data.begin(), msg_data.end());
                ups_state["ups_type"]= ups_type;
                break;
            }
            case 0x42:{
                string ups_type(msg_data.begin(), msg_data.end());
                ups_state["ups_type"]+= ups_type;
                break;
            }
            case 0x43:{
                string ups_sku(msg_data.begin(), msg_data.end());
                ups_state["ups_sku"]= ups_sku;
                break;
            }
            case 0x44:{
                string ups_sku(msg_data.begin(), msg_data.begin() + 4);
                ups_state["ups_sku"]+= ups_sku;
                break;
            }
            case 0x45:{
                string fw_version_1(msg_data.begin(), msg_data.begin()+ 8);
                ups_state["fw_version_1"]= fw_version_1;
                string fw_version_2(msg_data.begin()+ 8, msg_data.end());
                ups_state["fw_version_2"]= fw_version_2;
                break;
            }
            case 0x46:{
                string fw_version_3(msg_data.begin(), msg_data.begin()+ 8);
                ups_state["fw_version_3"]= fw_version_3;
                string fw_version_4(msg_data.begin()+ 8, msg_data.end());
                ups_state["fw_version_4"]= fw_version_4;
                break;
            }
            case 0x47:{
                uint16_t battery_install_date = (msg_data[0] << 8) | msg_data[1];
                string battery_install_date_str = convert_to_datetime(battery_install_date);
                ups_state["battery_install_date"] = battery_install_date_str;
                uint16_t  battery_lifetime = (msg_data[2] << 8) | msg_data[3];
                ups_state["battery_lifetime"] = to_string(battery_lifetime);
                uint16_t battery_near_eol_alarm_notification = (msg_data[4] << 8) | msg_data[5];
                ups_state["battery_near_eol_alarm_notification"] = to_string(battery_near_eol_alarm_notification);
                uint16_t battery_near_eol_alarm_reminder = (msg_data[6] << 8) | msg_data[7];
                ups_state["battery_near_eol_alarm_reminder"] = to_string(battery_near_eol_alarm_reminder);
                break;
            }
            case 0x48:{
                string battery_sku(msg_data.begin(), msg_data.end());
                ups_state["battery_sku"]= battery_sku;
                break;
            }
            case 0x49:{
                string ups_name(msg_data.begin(), msg_data.end());
                ups_state["ups_name"]= ups_name;
                break;
            }
            case 0x4a:{
                uint16_t allowed_op_mode= (msg_data[0] << 8) | msg_data[1];
                ups_state["allowed_operating_mode"]= to_string(allowed_op_mode);
                uint16_t pwr_quality_config= (msg_data[2] << 8) | msg_data[3];
                ups_state["power_quality_configuration"]= to_string(pwr_quality_config);
                
                uint16_t battery_replacetest_interval_raw= (msg_data[4] << 8) | msg_data[5];
                ups_state["battery_replacetest_interval_raw"]= to_string(battery_replacetest_interval_raw);
                std::vector<std::string> battery_replacetest_interval;
                
                    if (battery_replacetest_interval_raw & 1) {
                        battery_replacetest_interval.push_back("DISABLED");  // Testing is disabled
                    }
                    if (battery_replacetest_interval_raw & 2) {
                        battery_replacetest_interval.push_back("STARTUP");  // Testing is done only at every startup of UPS
                    }
                    if (battery_replacetest_interval_raw & 4) {
                        battery_replacetest_interval.push_back("EACH 7 DAYS SINCE STARTUP");  // Test every 7 days since startup
                    }
                    if (battery_replacetest_interval_raw & 8) {
                        battery_replacetest_interval.push_back("EACH 14 DAYS SINCE STARTUP");  // Test every 14 days since startup
                    }
                    if (battery_replacetest_interval_raw & 16) {
                        battery_replacetest_interval.push_back("EACH 7 DAYS SINCE LAST");  // Test every 7 days since last test
                    }
                    if (battery_replacetest_interval_raw & 32) {
                        battery_replacetest_interval.push_back("EACH 14 DAYS SINCE LAST");  // Test every 14 days since last test
                    }
//build string out of battery_replacetest_interval vector
                std::ostringstream battery_replacetest_interval_str;
                for (const auto& s : battery_replacetest_interval) {
                    if (!battery_replacetest_interval_str.str().empty()) { 
                        battery_replacetest_interval_str << "\n";  
                    }
                    battery_replacetest_interval_str << s;
                    }

                ups_state["battery_replacetest_interval"] = battery_replacetest_interval_str.str();

                uint16_t battery_replacement_due= (msg_data[6] << 8) | msg_data[7];
                ups_state["battery_replacement_due"]= convert_to_datetime(battery_replacement_due);
                uint16_t low_runtime_alarm_config= (msg_data[8] << 8) | msg_data[9];
                ups_state["low_runtime_alarm_config"]= to_string(low_runtime_alarm_config);
                uint16_t voltage_accept_max= (msg_data[10] << 8) | msg_data[11];
                ups_state["voltage_accept_max"]= to_string(voltage_accept_max);
                uint16_t voltage_accept_min= (msg_data[12] << 8) | msg_data[13];
                ups_state["voltage_accept_min"]= to_string(voltage_accept_min);

                uint16_t voltage_sensitivity_raw= msg_data[15];
                ups_state["voltage_sensitivity_raw"]= to_string(voltage_sensitivity_raw);
                
                switch(voltage_sensitivity_raw) {
                    case 1:
                        ups_state["voltage_sensitivity"]= "HIGH";
                        break;
                    case 2:
                        ups_state["voltage_sensitivity"]= "MEDIUM";
                        break;
                    case 4:
                        ups_state["voltage_sensitivity"]= "LOW";
                        break;
                    default:
                        ups_state["voltage_sensitivity"]= "UNKNOWN";
                        break; 
                    }

                break;
            }
            case 0x4b:{
                uint16_t apparent_power_rating= (msg_data[0] << 8) | msg_data[1];
                ups_state["apparent_power_rating"]= to_string(apparent_power_rating);
                uint16_t real_power_rating= (msg_data[2] << 8) | msg_data[3];
                ups_state["real_power_rating"]= to_string(real_power_rating);

                uint16_t voltage_config= (msg_data[4] << 8) | msg_data[5];
                ups_state["voltage_config_raw"]= to_string(voltage_config); 
                switch (voltage_config){
                    case 1:{
                        ups_state["voltage_config"]= "100";
                        break;
                    }
                    case 2:{
                        ups_state["voltage_config"]= "120";
                        break;
                    }
                    case 4:{
                        ups_state["voltage_config"]= "200";
                        break;
                    }
                    case 8:{
                        ups_state["voltage_config"]= "208";
                        break;
                    }
                    case 16:{
                        ups_state["voltage_config"]= "220";
                        break;
                    }
                    case 32:{
                        ups_state["voltage_config"]= "230";
                        break;
                    }
                    case 64:{
                        ups_state["voltage_config"]= "240";
                        break;
                    }
                    case 2048:{
                        ups_state["voltage_config"]= "115";
                        break;
                    }
                    default:{
                        ups_state["voltage_config"]= "UNKNOWN";
                        break;
                    }
                }
                break;
            }
            case 0x4c:{
                uint16_t power_on_delay;
                std::memcpy(&power_on_delay, &msg_data[0], sizeof(power_on_delay));
                ups_state["power_on_delay"]=to_string(power_on_delay);
                
                uint16_t power_off_delay;
                std::memcpy(&power_off_delay, &msg_data[2], sizeof(power_off_delay));
                ups_state["power_off_delay"]=to_string(power_off_delay);

                uint32_t reboot_delay;
                std::memcpy(&reboot_delay, &msg_data[4], sizeof(reboot_delay));
                ups_state["reboot_delay"]=to_string(reboot_delay);

                ups_state["runtime_minimum_return"]= convert_from_bp({msg_data.begin() + 8, msg_data.begin() + 10}, 0, false);

                uint16_t load_shed_config_raw;
                std::memcpy(&load_shed_config_raw, &msg_data[10], sizeof(load_shed_config_raw));
                ups_state["load_shed_config_raw"]= to_string(load_shed_config_raw);
                std::vector<std::string> load_shed_config;

                if(load_shed_config_raw & 1 == 1)
                    load_shed_config.push_back("USE_OFF_DELAY");     //UseOffDelay- Modifier: When set, the load shed conditions that have this as a valid modifier will use the TurnOffCountdownSetting to shut the outlet off.
                if(load_shed_config_raw & 2 == 2)
                    load_shed_config.push_back("MANUAL_RESTART_REQUIRED");
/*  ManualRestartRequired - Modifier - When set, the load shed conditions that have this as a valid modifier will use a turn off command instead of shutdown. 
This results in a manual intervention to restart the outlet. */
                if(load_shed_config_raw & 4 == 4)
                    load_shed_config.push_back("RESERVED_BIT");
/*  TimeOnBattery: The outlet group will shed based on the LoadShedTimeOnBatterySetting usage. When operating on battery greater than this time, the outlet will turn off.
The modifier bits UseOffDelay and ManualRestartRequired are valid with this bit */
                if(load_shed_config_raw & 8 == 8)
                    load_shed_config.push_back("TIME_ON_BATTERY");
/* TimeOnBattery: The outlet group will shed based on the LoadShedTimeOnBatterySetting usage. When operating on battery greater than this time, the outlet will turn off.
The modifier bits UseOffDelay and ManualRestartRequired are valid with this bit */
                if(load_shed_config_raw & 16 == 16)
                    load_shed_config.push_back("RUNTIME_REMAINING");
/* RunTimeRemaining: The outlet group will shed based on the LoadShedRuntimeRemainingSetting usage. When operating on battery and the runtime remaining is
less than or equal to this value, the outlet will turn off. The modifier bits UseOffDelay and ManualRestartRequired are valid with this bit. */
                if (load_shed_config_raw & 32 == 32)        //original stated 16bit space, but i think that's an error
                    load_shed_config.push_back("ON_OVERLOAD");
/* UPSOverload - When set, the outlet will turn off immediately (no off delay possible) when the UPS is in overload. The outlet will require a manual command
to restart. Not applicable for the Main Outlet Group (MOG) */

//build string out of battery_replacetest_interval vector
                std::ostringstream load_shed_config_str;
                for (const auto& s : load_shed_config) {
                    if (!load_shed_config_str.str().empty()) { 
                        load_shed_config_str << "\n";  
                    }
                    load_shed_config_str << s;
                    }

                ups_state["load_shed_config"]= load_shed_config_str.str();
//Outlet switches off (load shedding) when runtime drops to this value, in second
                ups_state["loadshed_runtime_remainging"]= convert_from_bp({msg_data.begin() + 12, msg_data.begin() + 14}, 0, false);
//Outlet switches off (load shedding) after maximum time on battery, in seconds
                ups_state["loadshed_runtime_limit"]= convert_from_bp({msg_data.begin() + 14, msg_data.begin()+16}, 0, false);

                break;
                }
            case 0x4d:{
                string outlet_name(msg_data.begin(), msg_data.end());
                ups_state["outlet_name"]=outlet_name;
                break;
            }
            case 0x4e:{
                uint16_t InterfaceDisable_BF;
                std::memcpy(&InterfaceDisable_BF, &msg_data[4], sizeof(InterfaceDisable_BF));
                break;
            }   
            case 0x5c:{
                uint16_t communicationMethod_EN;
                std::memcpy(&communicationMethod_EN, &msg_data[8], sizeof(communicationMethod_EN));
                break;
            }
            case 0x6c:{ 
                uint16_t battery_lifetime_status_raw;
                std::memcpy(&battery_lifetime_status_raw, &msg_data[6], sizeof(battery_lifetime_status_raw));
                ups_state["battery_lifetime_status_raw"]= to_string(battery_lifetime_status_raw);
                std::vector<std::string> battery_lifetime_status;
                if(battery_lifetime_status_raw & 1 == 1)
                    battery_lifetime_status.push_back("OK");
                if(battery_lifetime_status_raw & 2 == 2)
                    battery_lifetime_status.push_back("NEAR EOL");
                if(battery_lifetime_status_raw & 4 == 4)
                    battery_lifetime_status.push_back("OVER EOL");
                if(battery_lifetime_status_raw & 8 == 8)
                    battery_lifetime_status.push_back("NEAR EOL ACK");
                if(battery_lifetime_status_raw & 16 == 16)
                    battery_lifetime_status.push_back("OVER EOL ACK");
                break;
            }
            case 0x6d:{
                    
                ups_state["battery_soc"]= convert_from_bp({msg_data.begin() + 2, msg_data.begin() + 4}, 9, false);
//simple self-test
                uint16_t battery_replacetest_cmd;
                std::memcpy(&battery_replacetest_cmd, &msg_data[4], sizeof(battery_replacetest_cmd));
                ups_state["battery_replacetest_cmd"]= to_string(battery_replacetest_cmd);

//simple self-test
                uint16_t battery_replacetest_status_raw;
                std::memcpy(&battery_replacetest_status_raw, &msg_data[6], sizeof(battery_replacetest_status_raw));
                ups_state["battery_replacetest_status_raw"]= to_string(battery_replacetest_status_raw);
                std::vector<std::string> battery_replacetest_status;
                if(battery_replacetest_status_raw== 0)
                    battery_replacetest_status.push_back("UNKNOWN");    //empty data
                if(battery_replacetest_status_raw & 1 == 1)
                    battery_replacetest_status.push_back("PENDING");    //test will start soon
                if(battery_replacetest_status_raw & 2 == 2)
                    battery_replacetest_status.push_back("IN PROGRESS");    //test is running
                if(battery_replacetest_status_raw & 4 == 4)
                    battery_replacetest_status.push_back("PASSED");    //Battery passed replacement test
                if(battery_replacetest_status_raw & 8 == 8)
                    battery_replacetest_status.push_back("FAILED");    //Battery failed replacement test
                if(battery_replacetest_status_raw & 16 == 16)
                    battery_replacetest_status.push_back("REFUSED");    //UPS cannot test now, refused
                if(battery_replacetest_status_raw & 32 == 32)
                    battery_replacetest_status.push_back("ABORTED");    //Test was aborted
                if(battery_replacetest_status_raw & 64 == 64)
                    battery_replacetest_status.push_back("SOURCE PROTOCOL");    //Start or stopping of test was triggered from protocol
                if(battery_replacetest_status_raw & 128 == 128)
                    battery_replacetest_status.push_back("SOURCE UI");          //Start or stopping of test was triggered from user interface (UPS front panel)
                if(battery_replacetest_status_raw & 256 == 256)
                    battery_replacetest_status.push_back("SOURCE INTERNAL");    //Start or stopping of test was triggered internally
                if (battery_replacetest_status_raw & 512 == 512)
                    battery_replacetest_status.push_back("INVALID STATE");    //Invalid UPS Operating state to perform the test
                if (battery_replacetest_status_raw & 1024 == 1024)
                    battery_replacetest_status.push_back("INTERNAL FAULT");    //Internal fault such as battery missing, inverter failure, overload, etc.
                if (battery_replacetest_status_raw & 2048 == 2048)
                    battery_replacetest_status.push_back("SOC UNACCEPTABLE");    //SOC is too low to do the test

                std::ostringstream battery_replacetest_status_str;
                for (const auto& s : battery_replacetest_status) {
                    if (!battery_replacetest_status_str.str().empty()) { 
                        battery_replacetest_status_str << "\n";  
                    }
                    battery_replacetest_status_str << s;
                    }
                ups_state["battery_replacetest_status"]= battery_replacetest_status_str.str();

                uint16_t runtime_calibration_status_raw;
                std::memcpy(&runtime_calibration_status_raw, &msg_data[10], sizeof(runtime_calibration_status_raw));
                ups_state["runtime_calibration_status_raw"]= to_string(runtime_calibration_status_raw);
                std::vector<std::string> runtime_calibration_status;

                if(runtime_calibration_status_raw & 1 == 1)
                    runtime_calibration_status.push_back("PENDING");    //test will start soon
                if(runtime_calibration_status_raw & 2 == 2)
                    runtime_calibration_status.push_back("IN PROGRESS");    //test is running
                if(runtime_calibration_status_raw & 4 == 4)
                    runtime_calibration_status.push_back("PASSED");    //Calibration completed
                if(runtime_calibration_status_raw & 8 == 8)
                    runtime_calibration_status.push_back("FAILED");    //Calibration failed
                if(runtime_calibration_status_raw & 16 == 16)
                    runtime_calibration_status.push_back("REFUSED");    //Test refused (too small load connected?)
                if(runtime_calibration_status_raw & 32 == 32)
                    runtime_calibration_status.push_back("ABORTED");    //test aborted
                if(runtime_calibration_status_raw & 64 == 64)
                    runtime_calibration_status.push_back("SOURCE PROTOCOL");    //Start or stopping of test was triggered from protocol
                if(runtime_calibration_status_raw & 128 == 128)
                    runtime_calibration_status.push_back("SOURCE UI");          //Start or stopping of test was triggered from user interface (UPS front panel)
                if(runtime_calibration_status_raw & 256 == 256)
                    runtime_calibration_status.push_back("SOURCE INTERNAL");    //Start or stopping of test was triggered internally
                if(runtime_calibration_status_raw & 512 == 512)
                    runtime_calibration_status.push_back("INVALID STATE");    //Invalid UPS Operating state to perform the test
                if(runtime_calibration_status_raw & 1024 == 1024)
                    runtime_calibration_status.push_back("INTERNAL FAULT");    //Internal fault such as battery missing, inverter failure, overload, etc.
                if(runtime_calibration_status_raw & 2048 == 2048)
                    runtime_calibration_status.push_back("SOC UNACCEPTABLE");    //SOC is too low to do the test
                if(runtime_calibration_status_raw & 4096 == 4096)
                    runtime_calibration_status.push_back("LOAD CHANGED");    //The connected load varied too much to be able to calibrate
                if(runtime_calibration_status_raw & 8192 == 8192)
                    runtime_calibration_status.push_back("AC INPUT NOT ACCEPTABLE");    //AC Input not acceptable so test was aborted
                if(runtime_calibration_status_raw & 16384 == 16384)
                    runtime_calibration_status.push_back("LOAD TOO LOW");    //Connected load is too small to perform the calibration
                if(runtime_calibration_status_raw & 32768 == 32768)
                    runtime_calibration_status.push_back("OVERCHARGE IN PROGRESS");    //A battery overcharge is in progress so calibration would be inaccurate
                
                std::ostringstream runtime_calibration_status_str;
                for (const auto& s : runtime_calibration_status) {
                    if (!runtime_calibration_status_str.str().empty()) { 
                        runtime_calibration_status_str << "\n";  
                    }
                    runtime_calibration_status_str << s;
                    }
                ups_state["battery_replacetest_status"]= runtime_calibration_status_str.str();
                
                ups_state["runtime_remaining"]= to_string(static_cast<int>(convert_from_bp({msg_data.begin() + 14, msg_data.begin() + 16}, 0, false)));
                break;
            }
            case 0x6e:{
                ups_state["runtime_remaining_2"]= to_string(static_cast<int>(convert_from_bp({msg_data.begin(), msg_data.begin() + 4}, 0, false)));
                break;
            }
            case 0x6f:{
                ups_state["temperature"]= convert_from_bp({msg_data.begin(), msg_data.begin() + 2}, 7, true);
                
                uint16_t user_interface_cmd= (static_cast<uint16_t>(msg_data[2]) << 8) | msg_data[3];
                ups_state["user_interface_cmd"]= to_string(user_interface_cmd);

                uint16_t user_interface_status_raw= (static_cast<uint16_t>(msg_data[4]) << 8) | msg_data[6];
                ups_state["user_interface_status_raw"]= to_string(user_interface_status_raw);
                std::vector<std::string> user_interface_status;
                if (user_interface_status_raw & 1 == 1)
                    user_interface_status.push_back("CONT. TEST IN PROGRESS");
                if (user_interface_status_raw & 2 == 2)
                    user_interface_status.push_back("AUDIBLE ALARM IN PROGRESS");
                if (user_interface_status_raw & 4 == 4)
                    user_interface_status.push_back("AUDIBLE ALARM MUTED");

                std::ostringstream user_interface_status_str;
                for (const auto& s : user_interface_status) {
                    if (!user_interface_status_str.str().empty()) { 
                        user_interface_status_str << "\n";  
                    }
                    user_interface_status_str << s;
                    }
                ups_state["user_interface_status"]= user_interface_status_str.str();

                ups_state["voltage_out"]= convert_from_bp({msg_data.begin()+ 6, msg_data.begin()+ 8,}, 6, false);
                ups_state["current_out"]= convert_from_bp({msg_data.begin()+ 8, msg_data.begin()+ 10}, 5, false);
                ups_state["frequency_out"]= convert_from_bp({msg_data.begin()+ 10, msg_data.begin()+ 12}, 7, false);
                ups_state["apparent_power_pctused"]= convert_from_bp({msg_data.begin()+ 12, msg_data.begin()+ 14}, 8, false);
                ups_state["real_power_pctused"]= convert_from_bp({msg_data.begin()+ 14, msg_data.begin()+ 16}, 8, false);

                break;
            }
            case 0x70:{
                uint16_t input_status_raw;
                std::memcpy(&input_status_raw, &msg_data[4], sizeof(input_status_raw));
                ups_state["input_status_raw"]= to_string(input_status_raw);

                std::vector<std::string> input_status;
                if(input_status_raw & 1 == 1)
                    input_status.push_back("ACCEPTABLE");
                if(input_status_raw & 2 == 2)
                    input_status.push_back("PENDING ACCEPTABLE");
                if(input_status_raw & 4 == 4)
                    input_status.push_back("LOW VOLTAGE");
                if(input_status_raw & 8 == 8)
                    input_status.push_back("HIGH VOLTAGE");
                if(input_status_raw & 16 == 16)
                    input_status.push_back("DISTORTED");
                if(input_status_raw & 32 == 32)
                    input_status.push_back("BOOST");
                if(input_status_raw & 64 == 64)
                    input_status.push_back("TRIM");
                if(input_status_raw & 128 == 128)
                    input_status.push_back("LOW FREQUENCY");
                if(input_status_raw & 256 == 256)
                    input_status.push_back("HIGH FREQUENCY");
                if(input_status_raw & 512 == 512)
                    input_status.push_back("PHASE NOT LOCKED");
                if(input_status_raw & 1024 == 1024)
                    input_status.push_back("DELTA PHASE OUT OF RANGE");
                if(input_status_raw & 2048 == 2048)
                    input_status.push_back("NEUTRAL NOT CONNECTED");
                if(input_status_raw & 4096 == 4096)
                    input_status.push_back("NOT ACCEPTABLE");
                if(input_status_raw & 8192 == 8192)
                    input_status.push_back("PLUG RATING EXCEEDED");
                
                std::ostringstream input_status_str;
                for (const auto& s : input_status) {
                    if (!input_status_str.str().empty()) { 
                        input_status_str << "\n";  
                    }
                    input_status_str << s;
                    }
                ups_state["input_status"]= input_status_str.str();
                
                ups_state["voltage_in"]= convert_from_bp({msg_data.begin()+ 4, msg_data.begin()+ 6}, 6, false);
                ups_state["voltage_in"]= convert_from_bp({msg_data.begin()+ 6, msg_data.begin()+ 8}, 7, false);
                
                uint16_t green_mode;
                std::memcpy(&green_mode, &msg_data[4], sizeof(green_mode));
                ups_state["input_status_raw"]= to_string(green_mode);

                uint16_t powsys_error_raw;
                std::memcpy(&powsys_error_raw, &msg_data[10], sizeof(powsys_error_raw));
                ups_state["input_status_raw"]= to_string(powsys_error_raw);

                std::vector<std::string> powsys_error;
                if(powsys_error_raw & 1 == 1)
                    powsys_error.push_back("OUTPUT OVERLOAD");
                if(powsys_error_raw & 2 == 2)
                    powsys_error.push_back("OUTPUT SHORT CIRCUIT");
                if(powsys_error_raw & 4 == 4)
                    powsys_error.push_back("OUTPUT OVERVOLTAGE");
                if(powsys_error_raw & 8 == 8)
                    powsys_error.push_back("TRANSFORMER DC IMBALANCE");
                if(powsys_error_raw & 16 == 16)
                    powsys_error.push_back("OVERTEMPERATURE");
                if(powsys_error_raw & 32 == 32)
                    powsys_error.push_back("BACKFEEDING");
                if(powsys_error_raw & 64 == 64)
                    powsys_error.push_back("AVR RELAY FAULT");
                if(powsys_error_raw & 128 == 128)
                    powsys_error.push_back("PFC INPUT RELAY FAULT");
                if(powsys_error_raw & 256 == 256)
                    powsys_error.push_back("OUTPUT RELAY FAULT");
                if(powsys_error_raw & 512 == 512)
                    powsys_error.push_back("BYPASS RELAY FAULT");
                if(powsys_error_raw & 1024 == 1024)
                    powsys_error.push_back("FAN FAULT");
                if(powsys_error_raw & 2048 == 2048)
                    powsys_error.push_back("PFC FAULT");
                if(powsys_error_raw & 4096 == 4096)
                    powsys_error.push_back("DC BUS OVERVOLTAGE");
                if(powsys_error_raw & 8192 == 8192)
                    powsys_error.push_back("INVERTER FAULT");

                std::ostringstream powsys_error_str;
                for (const auto& s : powsys_error) {
                    if (!powsys_error_str.str().empty()) { 
                        powsys_error_str << "\n";  
                    }
                    powsys_error_str << s;
                    }
                ups_state["powsys_error"]= powsys_error_str.str();


                uint16_t general_error_raw;
                std::memcpy(&general_error_raw, &msg_data[10], sizeof(general_error_raw));
                ups_state["input_status_raw"]= to_string(general_error_raw);

                std::vector<std::string> general_error;
                if(general_error_raw & 1 == 1)
                    general_error.push_back("SITE WIRING FAULT");
                if(general_error_raw & 2 == 2)
                    general_error.push_back("EEPROM ERROR");
                if(general_error_raw & 4 == 4)
                    general_error.push_back("AD CONVERTER ERROR");
                if(general_error_raw & 8 == 8)
                    general_error.push_back("LOGIC PSU FAULT");
                if(general_error_raw & 16 == 16)
                    general_error.push_back("INTERNAL COMM FAULT");
                if(general_error_raw & 32 == 32)
                    general_error.push_back("UI BUTTON FAULT");
                if(general_error_raw & 64 == 64)
                    general_error.push_back("EPO ACTIVE"); //python code jumped from 32 to 128, looks like mistrake so i left 64
                if(general_error_raw & 128 == 128)
                    general_error.push_back("EPO ACTIVE");

                std::ostringstream general_error_str;
                for (const auto& s : general_error) {
                    if (!general_error_str.str().empty()) { 
                        general_error_str << "\n";  
                    }
                    general_error_str << s;
                    }
                ups_state["general_error"]= general_error_str.str();


                uint16_t batt_error_raw;
                std::memcpy(&batt_error_raw, &msg_data[10], sizeof(batt_error_raw));
                ups_state["input_status_raw"]= to_string(batt_error_raw);

                std::vector<std::string> batt_error;
                if(batt_error_raw & 1 == 1)
                batt_error.push_back("DISCONNECTED");
                if(batt_error_raw & 2 == 2)
                batt_error.push_back("OVERVOLTAGE");
                if(batt_error_raw & 4 == 4)
                batt_error.push_back("NEEDS REPLACEMENT");
                if(batt_error_raw & 8 == 8)
                batt_error.push_back("OVERTEMPERATURE");
                if(batt_error_raw & 16 == 16)
                batt_error.push_back("CHARGER FAULT");
                if(batt_error_raw & 32 == 32)
                batt_error.push_back("TEMP SENSOR FAULT");
                if(batt_error_raw & 64 == 64)
                batt_error.push_back("BATTERY BUS SOFT START FAULT"); //python code jumped from 32 to 128, looks like mistrake so i left 64
                if(batt_error_raw & 128 == 128)
                batt_error.push_back("HIGH TEMPERATURE");
                if(batt_error_raw & 256 == 256)
                batt_error.push_back("GENERAL ERROR");
                if(batt_error_raw & 512 == 512)
                batt_error.push_back("COMM ERROR");

                std::ostringstream batt_error_str;
                for (const auto& s : batt_error) {
                    if (!batt_error_str.str().empty()) { 
                        batt_error_str << "\n";  
                    }
                    batt_error_str << s;
                    }
                ups_state["batt_error"]= batt_error_str.str();

                break;
            }
            case 0x71:{
                uint16_t ups_cmd;
                std::memcpy(&ups_cmd, &msg_data[0], sizeof(ups_cmd));
                ups_state["ups_cmd"]= to_string(ups_cmd);

                //This ID is actually used to send commands to the outlet. No idea what the read values say, probably not relevant
                uint16_t outlet_cmd;
                std::memcpy(&outlet_cmd, &msg_data[0], sizeof(outlet_cmd));
                ups_state["outlet_cmd"]= to_string(outlet_cmd);
                break;
            }
            case 0x72:{
                uint16_t outlet_status_raw;
                std::memcpy(&outlet_status_raw, &msg_data[0], sizeof(outlet_status_raw));
                ups_state["outlet_status_raw"]= to_string(outlet_status_raw);
                
                vector<string> outlet_status;
                if(outlet_status_raw & 1 == 1)
                outlet_status.push_back("OUTLET ON");
                if(outlet_status_raw & 2 == 2)
                outlet_status.push_back("OUTLET OFF");
                if(outlet_status_raw & 4 == 4)
                outlet_status.push_back("REBOOTING");
                if(outlet_status_raw & 8 == 8)
                outlet_status.push_back("HIGH VOLTAGE");
                if(outlet_status_raw & 16 == 16)
                outlet_status.push_back("DISTORTED");
                if(outlet_status_raw & 32 == 32)
                outlet_status.push_back("BOOST");
                if(outlet_status_raw & 64 == 64)
                outlet_status.push_back("TRIM");
                if(outlet_status_raw & 128 == 128)
                outlet_status.push_back("LOW FREQUENCY");
                if(outlet_status_raw & 256 == 256)
                outlet_status.push_back("HIGH FREQUENCY");
                if(outlet_status_raw & 512 == 512)
                outlet_status.push_back("PHASE NOT LOCKED");
                if(outlet_status_raw & 1024 == 1024)
                outlet_status.push_back("DELTA PHASE OUT OF RANGE");
                if(outlet_status_raw & 2048 == 2048)
                outlet_status.push_back("NEUTRAL NOT CONNECTED");
                if(outlet_status_raw & 4096 == 4096)
                outlet_status.push_back("NOT ACCEPTABLE");
                if(outlet_status_raw & 8192 == 8192)
                outlet_status.push_back("PLUG RATING EXCEEDED");




                break;
            }
            default:{                           //Default behavior is to request next data
                next_apc_msg = APC_CMD_NEXT;
                return true;               
                break;
            }
        }

    }
    else {
        next_apc_msg = APC_CMD_RESET;
        return false;}
    
}
std::vector<uint8_t> ApcComm::calculate_challenge(){              //Calculate challenge from actual known ups state
    uint8_t b0= ups_state["series_id_raw"][1];
    uint8_t b1= ups_state["series_id_raw"][0];

    for (uint8_t header_byte : ups_state["header_raw"]) {
        b0 = (b0 + header_byte) % 255;
        b1 = (b1 + b0) % 255;
    }
    for (uint8_t serial_nb_byte : ups_state["serial_nb_raw"]) {
        b0 = (b0 + serial_nb_byte) % 255;
        b1 = (b1 + b0) % 255;
    }
    for (uint8_t serial_nb_byte : ups_state["serial_nb_raw"]) {
        b0 = (b0 + serial_nb_byte) % 255;
        b1 = (b1 + b0) % 255;
    }
    std::vector<uint8_t> challenge= {1, 1, b0, b1};
    return challenge;
}
double ApcComm::convert_from_bp(const std::vector<uint8_t>& data, int frac_pos, bool is_signed = false){     //Convert the binary point number in data to a float, given the fractional bit position
    int32_t int_value = 0;
    for (uint8_t byte : data) {
        int_value = (int_value << 8) | byte; 
    }
    if (is_signed && (data[0] & 0x80)) {  // Check if the highest bit is set (negative number)
        int_value |= -1 ^ ((1 << (8 * data.size())) - 1);  // Sign extension
    }
    return static_cast<double>(int_value) / (1 << frac_pos);
}
std::vector<uint8_t> ApcComm::convert_to_bp(float value, int frac_pos) {
    // Multiply the value by 2^frac_pos to scale it
    int scaled_value = static_cast<int>(value * (1 << frac_pos));

    // Create a 2-byte vector for the result
    std::vector<uint8_t> data(2);

    // Convert the scaled value to bytes (big-endian format)
    data[0] = (scaled_value >> 8) & 0xFF;  // High byte
    data[1] = scaled_value & 0xFF;         // Low byte

    return data;
}
std::string ApcComm::convert_to_datetime(int value) {
    // Define the base date (1 Jan 2000)
    std::tm base_date = {};
    base_date.tm_year = 2000 - 1900;  // tm_year is years since 1900
    base_date.tm_mon = 0;             // January
    base_date.tm_mday = 1;            // 1st day of the month

    // Convert the base date to a time_t value
    std::time_t base_time = std::mktime(&base_date);
    if (base_time == -1) {
        return "Invalid date"; // Error handling if mktime fails
    }

    // Add the days to the base date
    base_time += value * 24 * 60 * 60; // Add the number of seconds for the given number of days

    // Convert time_t back to tm structure
    std::tm* new_date = std::localtime(&base_time);

    // Format the new date as a string (e.g., YYYY-MM-DD)
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d", new_date);

    return std::string(buffer);
}

ApcCli::ApcCli(){}                       // member initialisation list, for const variables
ApcCli::~ApcCli() {}

void ApcCli::print_keys(string keylist){}
void ApcCli::do_commstate(){}
void ApcCli::do_voltage(){}
void ApcCli::do_current(){}
void ApcCli::do_frequency(){}
void ApcCli::do_runtime(){}
void ApcCli::do_battery(){}
void ApcCli::do_status(){}
void ApcCli::do_all(){}
void ApcCli::do_set(){}
void ApcCli::do_write(){}
void ApcCli::do_exit(){}
void ApcCli::send_msg(){}
//Stupid fletcher checksum implementation bcs this shit brick has to implement some fossil alghoritm
uint16_t Fletcher::fletcher8(const std::vector<uint8_t>& raw_msg) {
    
    std::vector<uint8_t> sub_msg(raw_msg.begin(), raw_msg.begin() -2); 
    
    update(sub_msg);  
    uint16_t checksum = (static_cast<uint16_t>(cb0) << 8) + cb1; 
    return checksum;  
}
void Fletcher::update(const std::vector<uint8_t>& data) {

    uint8_t sum0 = 0;
    uint8_t sum1 = 0;
    
    for (uint8_t byte : data) {
        sum0 = (sum0 + byte) % 255;
        sum1 = (sum1 + sum0) % 255;
    }
    
    cb0 = sum0;
    cb1 = sum1;
}
