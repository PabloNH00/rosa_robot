#include "rosa_wifi.h"
#include "rosa_esp32_utils.h"
#include <stdarg.h>
ROSAmens::CircularBuffer<> RosaWiFi::buffer{};
ROSAmens::MsgReader RosaWiFi::msg_reader{};
AsyncUDP RosaWiFi::udp;
IPAddress RosaWiFi::ip_remote(0,0,0,0);
long RosaWiFi::_time_watch_dog=0;
RosaWiFi::WIFI_STATE RosaWiFi::state;
long RosaWiFi::_timeout=0;
long RosaWiFi::_time=0;
long RosaWiFi::_stop_time=0;

void RosaWiFi::init(int trials, const char *ssid, const char *psswd)
{
  const IPAddress local_IP(RosaDefs::IP_ADDRESS[0], RosaDefs::IP_ADDRESS[1], RosaDefs::IP_ADDRESS[2], RosaDefs::IP_ADDRESS[3]);
  const IPAddress gateway(RosaDefs::GATEWAY_ADDRESS[0], RosaDefs::GATEWAY_ADDRESS[1], RosaDefs::GATEWAY_ADDRESS[2], RosaDefs::GATEWAY_ADDRESS[3]);
  const IPAddress subnet(RosaDefs::SUBNET_MASK[0], RosaDefs::SUBNET_MASK[1], RosaDefs::SUBNET_MASK[2], RosaDefs::SUBNET_MASK[3]); 
  

    if(WiFi.status() == WL_CONNECTED)WiFi.disconnect();

    
   WiFi.config(local_IP, gateway, subnet);

    state=WIFI_STATE::INITIALIZING;

    WiFi.begin(RosaDefs::WIFI_SSID, RosaDefs::WIFI_KEY);
    _time=millis();
    _timeout = trials;
}
void RosaWiFi::setup()
{
  init(1000);
}
void RosaWiFi::loop()
{
    switch(state){
        case WIFI_STATE::NONE:
          if (millis() - _stop_time > _timeout)
            init();
          return;
        case WIFI_STATE::INITIALIZING:
             if(WiFi.status() == WL_CONNECTED){
                 state=WIFI_STATE::CONNECTED;
                 char aux[20];
                 WiFi.localIP().toString().toCharArray(aux,20);
                 _time_watch_dog=millis();
                 listenUDP();
                 
             }else if(millis()-_time>_timeout){
                 _stop_time = millis();
                 state=WIFI_STATE::NONE;
             }
        break;
        case WIFI_STATE::CONNECTED:
           if(millis()-_time_watch_dog>TIME_OUT_WIFI_MASTER)ip_remote[0]=0;
           while(buffer.there_is_msg()){
                ROSAmens &&m=buffer.getMessage();
                ROSAmens &&resp=executeWifiMessage(m);
                RosaWiFi::sendMessage(resp);
           }
        break;
    }
}

ROSAmens RosaWiFi::executeWifiMessage(const ROSAmens &m)
{
  switch(m.id){
    case ROSA_SET_MASTER_IP:
      _time_watch_dog=millis();
       WIFI_DEBUG("MASTER CONECTED");
      return name_message(RosaDefs::ROBOT_NAME);    
  }
  return ROSAmens::none();
//return proccess_message(m); //remaining messages are executed as always
}

void RosaWiFi::sendMessage(const ROSAmens &m, const IPAddress &ip)
{
  if(!ip[0])return;
  if(m.size){
    AsyncUDPMessage mens;
    mens.write(m.data,m.datagram_size());
    udp.sendTo(mens,ip,ROSA_OUTPUT_UDP_PORT);  
  }
}

void RosaWiFi::listenUDP()
{

  if (udp.listen(ROSA_INPUT_UDP_PORT)) {
    udp.onPacket([](AsyncUDPPacket packet) {
        for(int i=0;i<packet.length();i++){
            if(msg_reader.add_uchar(packet.data()[i])){
                auto&& mens=msg_reader.getMessage();
                if(!RosaWiFi::ip_remote[0])RosaWiFi::ip_remote=packet.remoteIP();
                if(RosaWiFi::ip_remote==packet.remoteIP())buffer.push_single(mens);
            }
        }
    });
  }
}
const char *RosaWiFi::translateEncryptionType(wifi_auth_mode_t encryptionType) {
 
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
    case (WIFI_AUTH_MAX):
       return "WIFI_AUTH_MAX";
  }
  return "UNKNOWN";
}

void RosaWiFi::sendText(const char *text)
{
      ROSAmens &&rep_msg = debug_text_message(text);
      RosaWiFi::sendMessage(rep_msg);
}
void RosaWiFi::sendPrint(const char *fmt, ... )
{
    char text[300];
    va_list myargs;
    va_start(myargs,fmt);
    vsprintf(text,fmt,myargs);
    va_end(myargs);
    ROSAmens &&rep_msg = debug_text_message(text);
    RosaWiFi::sendMessage(rep_msg);
}