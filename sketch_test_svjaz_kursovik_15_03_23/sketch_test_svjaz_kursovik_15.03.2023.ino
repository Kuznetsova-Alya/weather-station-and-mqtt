
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>

/* change it with your ssid-password */
const char *ssid = "AndroidAP069D";
const char *password = "ikmu9538";
/* this is the IP of PC/raspberry where you installed MQTT Server
on Wins use "ipconfig"
on Linux use "ifconfig" to get its IP address */
const char *mqtt_server = "51.250.11.159";

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient client(espClient);

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
uint16_t input_from_stm[4];
uint8_t output_from_esp[1];
long lastPublishTime;


/* topics */
#define DHT11_TOPIC_T "smarthome/temp"
#define DHT11_TOPIC_HUM "smarthome/hum"
#define LED_TOPIC_BRIGHT "smarthome/brigh"
#define LED_TOPIC "smarthome/status_led"
#define LED_TOPIC_1 "smarthome/led_on"

//сервер получил данные, запускается функция Callback по этому событию. Серевер отправляет данные всем подписчикам, 
//Callback приостанавливаем смежные процессы и принимает данные
void receivedCallback(char *topic, byte *payload, unsigned int length)// topic - имя топика, payload - содержимое типа байт, length - длина содержимого
{
  Serial.print("topic = ");
  Serial.println(topic);
  Serial.print("payload[0] = ");
  //чтобы прочитать набор byte как символы - делаем преобразование типов
  Serial.println(((char *)payload)[0]); //чтобы вытащить значение в нужном виде
                                        //сначала преобразовываем адрес byte в адес char, поэтому ставим скобки, а потом только читаем первый элемент массива
  if (strcmp(topic, LED_TOPIC_1) == 0)
  {
    Serial.println("topic valid");
    output_from_esp[0] = payload[0];
    
    Serial.print("resieved - ");
    Serial.println(((char *)output_from_esp)[0]);
  }
  
 While(strcmp(topic, LED_TOPIC_1) == 0)
 {
 Serial2.write(output_from_esp, 1);
 delay(5000);
 }
}

void mqttconnect()
{
  /* Loop until reconnected */
  while (!client.connected())
  {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
      client.subscribe(LED_TOPIC_1);
    }
    else
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // We start by connecting to a WiFi network - не возвращаем
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked
  when client received subscribed topic */
  client.setCallback(receivedCallback);

  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);

  Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // SERIAL_8N2 означает, что по последовательному порту ардуино передаст пакет
                                                       // длиной 8 бит без бита контроля четности (указано N) и с одним стоповым битом,
                                                       // который будет добавлен после байта данных.
}
void loop()
{
  
  /* if client was disconnected then try to reconnect again */
  if (!client.connected())
  {
    mqttconnect();
  }
  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  client.loop();

  if (Serial2.available() > 0) // если есть доступные данные
                                //  считываем 1 байта. 
  {
   uint8_t start=0;
    Serial2.readBytes(&start,1); //читаем 1 байта в переменную start
    Serial.print("read start = ");
    Serial.println(start,HEX);
        
  if (start == 0xFF) // если это начало передачи, читаем остальное. 
  {
    Serial2.readBytes((uint8_t *)input_from_stm, 8);
  }
  }

if (millis() - lastPublishTime > 5000) {
  lastPublishTime = millis();
  
  char c[20];
  sprintf(c, "%d", input_from_stm[0]);
  Serial.print("t=");
  Serial.println(input_from_stm[0]);
  client.publish(DHT11_TOPIC_T, c, strlen(c));

  char a[20];
  sprintf(a, "%d", input_from_stm[1]);
  Serial.print("h=");
  Serial.println(input_from_stm[1]);
  client.publish(DHT11_TOPIC_HUM, a, strlen(a));

  char b[20];
  sprintf(b, "%d", input_from_stm[2]);
  Serial.print("br=");
  Serial.println(input_from_stm[2]);
  client.publish(LED_TOPIC_BRIGHT, b, strlen(b));

  char d[20];
  sprintf(d, "%d", input_from_stm[3]);
  Serial.print("rez=");
  Serial.println(input_from_stm[3]);
  client.publish(LED_TOPIC, d, strlen(d));
}
}
