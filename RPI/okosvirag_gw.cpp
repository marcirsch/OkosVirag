#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <RF24/RF24.h>
#include "MQTTClient.h"

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/error/error.h"

using namespace std;

RF24 radio(22,0);


#define MSG_TYPE_DOWNLINK 0
#define MSG_TYPE_TEMP_HUM 1

#pragma pack(1)
typedef struct {
	uint32_t myaddr;
	uint8_t msgType;
	uint32_t msg[27];
}__attribute__ ((packed)) msg_header_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	int8_t temperature;
	int8_t humidity;
}__attribute__ ((packed)) sensor_msg_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	int8_t water_threshold;
	int8_t sleep_seconds;
} __attribute__ ((packed)) downlink_msg_t;
#pragma pack()


#pragma pack(1)
typedef struct {
	int8_t water_threshold;
	int8_t sleep_seconds;
} __attribute__ ((packed)) configuration_t;
#pragma pack()


downlink_msg_t downlinkMsg;
sensor_msg_t sensorMsg;
msg_header_t headerMsg;


/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"1Node","2Node"};

void replyToSensor(uint32_t address);
void setupRadio(RF24 &rf);



//*************MQTT*****************//

#define MQTT_ADDRESS     "tcp://localhost:1883"
#define MQTT_CLIENTID    "NRF2MQTT"
#define MQTT_QOS         1
#define MQTT_TIMEOUT     10000L
#define MQTT_SUB_TOPIC   "Okosvirag/config"

volatile MQTTClient_deliveryToken deliveredtoken;
MQTTClient_deliveryToken token;
MQTTClient_message pubmsg = MQTTClient_message_initializer;
//MQTT callback functions
void setupMQTT(MQTTClient &cli);

void onDelivery(void *context, MQTTClient_deliveryToken dt);
int onMessage(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void onConnLost(void *context, char *cause);

vector<msg_header_t>  MQTTMessageQueue;

void addMQTTQueue(msg_header_t msg);

void addRandom2vectors(){
	msg_header_t header;
	downlink_msg_t dl;


	for(int i=0; i< 20; i++){
		header.myaddr = i*100;
		header.msgType = 0;
		dl.water_threshold  = 10+5*i;
		dl.sleep_seconds = 5+5*i;
		memcpy(header.msg,&dl,sizeof(dl));
		MQTTMessageQueue.push_back(header);
	}

	header.myaddr = 0x18473431;
	header.msgType = 0;
	dl.water_threshold  = 19;
	dl.sleep_seconds = 2;
	memcpy(header.msg,&dl,sizeof(dl));
	MQTTMessageQueue.push_back(header);

	for(int i=0; i< 20; i++){
		header.myaddr = i*100;
		header.msgType = 0;
		dl.water_threshold  = 10+5*i;
		dl.sleep_seconds = 5+5*i;
		memcpy(header.msg,&dl,sizeof(dl));
		MQTTMessageQueue.push_back(header);
	}

}

int main(int argc, char** argv){

	setupRadio(radio);

	MQTTClient client;
	setupMQTT(client);

	radio.startListening();

	addRandom2vectors();
	
	// forever loop
	while (1)
	{
		// if there is data ready
		if ( radio.available() )
		{
			cout<<"received something"<<endl;
			// Fetch the payload, and see if this was the last one.
			while(radio.available()){
				radio.read( &headerMsg, sizeof(msg_header_t) );
			}
			radio.stopListening();          // First, stop listening so we can talk.

			replyToSensor(headerMsg.myaddr);

			//receive something from the cloud in 100ms window
			radio.startListening();

			printf("msg from %x, type: %d\n",headerMsg.myaddr, headerMsg.msgType);
			memcpy(&sensorMsg,headerMsg.msg,sizeof(sensor_msg_t));
			printf("Temp: %d, hum: %d\n",sensorMsg.temperature, sensorMsg.humidity);

			char pubstr[100];

			sprintf(pubstr, "{\"id\": %d, \"humidity\": %d}\0",headerMsg.myaddr, sensorMsg.humidity);
			pubmsg.payload = pubstr;
			pubmsg.payloadlen = strlen(pubstr);
			MQTTClient_publishMessage(client,"Okosvirag/humidity",&pubmsg, &token);

			sprintf(pubstr, "{\"id\": %d, \"temperature\": %d}\0",headerMsg.myaddr ,sensorMsg.temperature);
			pubmsg.payload = pubstr;
			pubmsg.payloadlen = strlen(pubstr);
			MQTTClient_publishMessage(client,"Okosvirag/temperature",&pubmsg, &token);
		}

	} // forever loop

	return 0;
}

void addMQTTQueue(msg_header_t msg){

	for(int i=0; i<MQTTMessageQueue.size();++i){
		if(MQTTMessageQueue[i].myaddr == msg.myaddr){
			MQTTMessageQueue[i] = msg;
			return;
		}
	}

	MQTTMessageQueue.push_back(msg);
	return;
}

msg_header_t getMQTTQueue(uint32_t address){
	msg_header_t retval;

	for(int i=0; i<MQTTMessageQueue.size();++i){
		// cout<<"     "<<MQTTMessageQueue[i].myaddr<<endl;
		if(MQTTMessageQueue[i].myaddr == address){
			memcpy(&retval,&MQTTMessageQueue[i], sizeof(msg_header_t));
			MQTTMessageQueue.erase(MQTTMessageQueue.begin() + i);
			cout<<"found msg in queue for "<<address<<endl;
		}
	}

	return retval;
}



void replyToSensor(uint32_t address){
	msg_header_t header;

	if(MQTTMessageQueue.empty()){return;}

	header = getMQTTQueue(address);

	if(header.myaddr == 0 ){
		return;
	}
	

	// dl_msg.water_threshold = 10;
	// dl_msg.sleep_seconds = 20;
	// header.msgType = MSG_TYPE_DOWNLINK;
	// memcpy(header.msg,&dl_msg,sizeof(downlink_msg_t));

	if (!radio.write(&header, sizeof(msg_header_t))) {
		cout<<"Error at replyToSensor"<<endl;
	}
}



void onDelivery(void *context, MQTTClient_deliveryToken dt)
{
   // printf("Message with token value %d delivery confirmed\n", dt);
	deliveredtoken = dt;
}

int onMessage(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
	int i;
	char* payloadptr;

	printf("Message arrived\n");
	printf("     topic: %s\n", topicName);
	printf("length: %d\n", message->payloadlen);
	printf("   message: ");

	payloadptr = (char*)message->payload;
	for(i=0; i<message->payloadlen; i++)
	{
		putchar(*payloadptr++);
	}
	putchar('\n');

	msg_header_t h;
	downlink_msg_t dl;

	try{
		rapidjson::Document d;
		rapidjson::ParseResult ok = d.Parse((char*)message->payload, message->payloadlen);
		if (!ok)
			printf( "JSON parse error: %s (%u)\n", GetParseError_En(ok.Code()), ok.Offset());

		cout<<"got mqtt msg id: "<<d["id"].GetInt()<<" water_threshold: "<<d["water_th"].GetInt()<<"  sleep sec: :"<<d["sleep_sec"].GetInt()<<endl;

		
		h.myaddr = d["id"].GetInt();
		h.msgType = MSG_TYPE_DOWNLINK;
		dl.water_threshold = d["water_th"].GetInt();
		dl.sleep_seconds = d["sleep_sec"].GetInt();
		memcpy(h.msg,&dl,sizeof(dl));
	}catch(int e){
		cout<< "wrong json format"<<endl;
	}

	addMQTTQueue(h);

	MQTTClient_freeMessage(&message);
	MQTTClient_free(topicName);
	return 1;
}



void onConnLost(void *context, char *cause)
{
	printf("\nConnection lost\n");
	printf("     cause: %s\n", cause);
}

void setupMQTT(MQTTClient &cli){
	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	
	
	int rc;

	MQTTClient_create(&cli, MQTT_ADDRESS, MQTT_CLIENTID,MQTTCLIENT_PERSISTENCE_NONE, NULL);
	conn_opts.keepAliveInterval = 20;
	conn_opts.cleansession = 1;

	MQTTClient_setCallbacks(cli, NULL, onConnLost, onMessage, onDelivery);

	if ((rc = MQTTClient_connect(cli, &conn_opts)) != MQTTCLIENT_SUCCESS){
		printf("Failed to connect to MQTT broker, return code %d\n", rc);
		exit(-1);
	}

	MQTTClient_subscribe(cli, MQTT_SUB_TOPIC, MQTT_QOS);

	pubmsg.qos = MQTT_QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;
}

void setupRadio(RF24 &rf){
  // Setup and configure rf radio
	rf.begin();

  // optionally, increase the delay between retries & # of retries
	rf.setRetries(15,15);
  // Dump the configuration of the rf unit for debugging
	rf.printDetails();
	rf.setPayloadSize(32);

	rf.openWritingPipe(pipes[1]);
	rf.openReadingPipe(1,pipes[0]);
}