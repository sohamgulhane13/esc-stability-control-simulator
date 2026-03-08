/*
  mqtt_publisher.c
  Pure-C MQTT publisher (Paho C library)

  Build:
      gcc -o mqtt_pub mqtt_publisher.c -lpaho-mqtt3c

  Run:
      ./mqtt_pub demo_sensors.csv
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "MQTTClient.h"

#define ADDRESS   "tcp://localhost:1883"
#define CLIENTID  "esc_publisher"
#define TOPIC     "esc/sensors"
#define QOS       1

int main(int argc, char *argv[]) {

    if (argc < 2) {
        printf("Usage: ./mqtt_pub <csv_file>\n");
        return 1;
    }

    const char *csv_file = argv[1];
    FILE *f = fopen(csv_file, "r");
    if (!f) {
        perror("Cannot open CSV");
        return 1;
    }

    MQTTClient client;
    MQTTClient_create(&client, ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    if (MQTTClient_connect(client, &conn_opts) != MQTTCLIENT_SUCCESS) {
        printf("Publisher: MQTT connect failed!\n");
        return 2;
    }

    printf("Publisher connected to broker. Streaming data...\n");

    char line[1024];
    while (fgets(line, sizeof(line), f)) {

        // Skip comments / blank lines
        if (line[0] == '#' || strlen(line) < 3)
            continue;

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        pubmsg.payload = line;
        pubmsg.payloadlen = (int)strlen(line);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

        MQTTClient_deliveryToken token;
        MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
        MQTTClient_waitForCompletion(client, token, 1000L);

        usleep(50000);   // 50ms = 20Hz real-time updates
    }

    fclose(f);
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    printf("Publisher finished.\n");
    return 0;
}
