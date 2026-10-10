#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "Protocol.h"

// All networking and application work runs in loop(). The Wi-Fi callback only queues.
class ReceiverLink {
 public:
  typedef void (*ControlHandler)(const ctl::Packet&);
  typedef void (*StopHandler)();
  typedef uint8_t (*StatusProvider)();
  void setStatusProvider(StatusProvider provider) { status_=provider; }
  bool begin(const char *ssid, uint8_t role, ControlHandler control, StopHandler stop) {
    self=this; role_=role; control_=control; stop_=stop; strlcpy(ssid_,ssid,sizeof(ssid_));
    q_=xQueueCreate(12, sizeof(Rx));
    if (!q_) return false;
    // AP ONLY: no STA profile can reconnect to a router and silently move our AP channel.
    if(!startAP()) return false;
    if (esp_now_init()!=ESP_OK) return false;
    esp_now_register_recv_cb(onReceive);
    if (!udp_.begin(ctl::UDP_PORT)) return false;
    radioReady_=true;
    // Silent until real control data (or an initialization error) in Receiver_Test.
    return true;
  }
  uint32_t dropped() const{return queueDrops_;}
  uint32_t restarts() const{return restarts_;}
  uint32_t timeouts() const{return timeouts_;}
  void update() {
    // Recover a stopped/misconfigured AP, always neutralizing outputs first.
    if(millis()-healthAt_>=1000) {
      healthAt_=millis(); uint8_t ch=0; wifi_second_chan_t second;
      if(!radioReady_ || WiFi.getMode()!=WIFI_AP || esp_wifi_get_channel(&ch,&second)!=ESP_OK || ch!=ctl::WIFI_CHANNEL) {
        // A single transient driver query must not reboot the AP and disconnect everyone.
        if(++healthFaults_>=3){
        if(everControl_)Serial.printf("[RX-LINK] radio recovery qDrop=%lu\n",(unsigned long)queueDrops_);
        restarts_++;stop_(); active_=false; paired_=false; session_=0; haveSeq_=false;
        udp_.stop(); esp_now_unregister_recv_cb(); esp_now_deinit(); memset(mac_,0,6);
        WiFi.softAPdisconnect(true); radioReady_=false;
        if(startAP() && esp_now_init()==ESP_OK) {
          esp_now_register_recv_cb(onReceive); radioReady_=udp_.begin(ctl::UDP_PORT);
          xQueueReset(q_);
        }
        healthFaults_=0;
        }
      } else healthFaults_=0;
    }
    if(!radioReady_) return;
    int n=udp_.parsePacket();
    if (n) {
      ctl::Hello h;
      IPAddress ip=udp_.remoteIP(); uint16_t port=udp_.remotePort();
      if (n==(int)sizeof(h) && udp_.read((uint8_t*)&h,sizeof(h))==(int)sizeof(h)
          && ctl::valid(h) && h.type==ctl::HELLO && h.session && !(h.mac[0]&1)) {
        bool same=paired_ && h.session==session_ && !memcmp(h.mac,mac_,6);
        // A second controller cannot steal a live session. Demo pairing is NOT authentication.
        if (same || !paired_ || !memcmp(h.mac,mac_,6) || millis()-lastContact_>2000) {
          if (!same) {
            stop_(); active_=false;
            if (paired_) esp_now_del_peer(mac_);
            memcpy(mac_,h.mac,6); session_=h.session; haveSeq_=false; paired_=false;
            esp_now_peer_info_t peer={};
            memcpy(peer.peer_addr,mac_,6); peer.channel=ctl::WIFI_CHANNEL;
            peer.ifidx=WIFI_IF_AP; peer.encrypt=false;
            if (esp_now_add_peer(&peer)==ESP_OK) paired_=true;
          }
          if (paired_) {
            lastContact_=millis();
            ctl::Hello reply; reply.type=ctl::WELCOME; reply.session=session_;
            reply.role=role_; WiFi.softAPmacAddress(reply.mac);
            udp_.beginPacket(ip,port); udp_.write((uint8_t*)&reply,sizeof(reply)); udp_.endPacket();
          }
        }
      } else { while(udp_.available()) udp_.read(); }
    }
    Rx rx,newest; bool haveNewest=false;
    while (xQueueReceive(q_,&rx,0)==pdTRUE) {
      const ctl::Packet &p=rx.packet;
      if (!paired_ || memcmp(rx.mac,mac_,6) || !ctl::valid(p) || p.session!=session_) continue;
      if (millis()-rx.at>ctl::FAILSAFE_MS) continue; // never replay stale queue entries
      lastContact_=millis();
      if (p.type==ctl::PING) {
        ctl::Packet pong=p; pong.type=ctl::PONG; pong.flags=status_?status_():0;
        if(esp_now_send(mac_,(uint8_t*)&pong,sizeof(pong))!=ESP_OK)sendErrors_++;
      } else if (p.type==ctl::CONTROL || p.type==ctl::NEUTRAL) {
        if (haveSeq_ && !ctl::newer(p.seq,lastSeq_)) continue;
        haveSeq_=true; lastSeq_=p.seq; lastControl_=rx.at;
        newest=rx;haveNewest=true;
      }
    }
    // Coalesce backlog; never replay a burst of old relative mouse/control commands.
    if(haveNewest){
      if(newest.packet.type==ctl::NEUTRAL){stop_();active_=false;}
      else {control_(newest.packet);active_=true;everControl_=true;}
    }
    if (active_ && millis()-lastControl_>ctl::FAILSAFE_MS) {
      timeouts_++;
      if(everControl_)Serial.printf("[RX-LINK] control timeout age=%lu qDrop=%lu txErr=%lu\n",(unsigned long)(millis()-lastControl_),(unsigned long)queueDrops_,(unsigned long)sendErrors_);
      active_=false; stop_(); // app prints ONE state transition, not a periodic SAFE stream
    }
  }
  bool paired() const { return paired_; }
  bool active() const { return active_; }
 private:
  bool startAP() {
    WiFi.persistent(false); WiFi.setAutoReconnect(false);
    if(!WiFi.mode(WIFI_AP)) return false;
    WiFi.setSleep(false);esp_wifi_set_ps(WIFI_PS_NONE);esp_wifi_set_bandwidth(WIFI_IF_AP,WIFI_BW_HT20);
    if(!WiFi.softAP(ssid_,ctl::WIFI_PASSWORD,ctl::WIFI_CHANNEL,false,2)) return false;
    wifi_config_t config={};
    if(esp_wifi_get_config(WIFI_IF_AP,&config)!=ESP_OK) return false;
    config.ap.ssid_hidden=0; config.ap.channel=ctl::WIFI_CHANNEL;
    config.ap.beacon_interval=100; config.ap.authmode=WIFI_AUTH_WPA2_PSK;
    if(esp_wifi_set_config(WIFI_IF_AP,&config)!=ESP_OK) return false;
    return esp_wifi_set_protocol(WIFI_IF_AP,WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N)==ESP_OK;
  }
  uint8_t healthFaults_=0;bool everControl_=false;
  volatile uint32_t queueDrops_=0;uint32_t restarts_=0,timeouts_=0,sendErrors_=0;
  char ssid_[33]={}; uint32_t healthAt_=0; bool radioReady_=false;
  struct Rx { uint8_t mac[6]; ctl::Packet packet; uint32_t at; };
  static ReceiverLink *self;
  static void onReceive(const esp_now_recv_info_t *info,const uint8_t *data,int len) {
    if (!self || !self->q_ || !info || len!=(int)sizeof(ctl::Packet)) return;
    Rx rx; memcpy(rx.mac,info->src_addr,6); memcpy(&rx.packet,data,len); rx.at=millis();
    if(xQueueSend(self->q_,&rx,0)!=pdTRUE)self->queueDrops_++; // drop on overflow, NEVER block the Wi-Fi task
  }
  QueueHandle_t q_=nullptr;
  WiFiUDP udp_;
  uint8_t mac_[6]={},role_=0;
  uint32_t session_=0,lastContact_=0,lastControl_=0,lastSeq_=0;
  bool paired_=false,active_=false,haveSeq_=false;
  ControlHandler control_=nullptr; StopHandler stop_=nullptr;
  StatusProvider status_=nullptr;
};
// Exactly one translation unit includes this header in these receiver sketches.
ReceiverLink *ReceiverLink::self=nullptr;
