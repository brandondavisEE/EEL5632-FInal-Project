const int TRIG_PIN = 2;
const int ECHO_PIN = 4;
const int YELLOW_LED = 32;
const int RED_LED = 30;
const int BLUE_LED = 27;
const int GREEN_LED = 25;

bool BLUE_STATE;
bool RED_STATE;
bool YELLOW_STATE;
bool GREEN_STATE;

const unsigned int MAX_DIST = 23200;

const byte numChars = 32;
char recv_buffer[numChars];
bool newMsg = false;
static byte idx = 0;

int speed = 0;
bool initialized;
bool collision_state;
float prev_dist = 0;
float current_dist = 0;
float measurements[] = {0,0,0,0,0};
int idm[] = {1,2,3,4,5};

// threshold in cm
int threshold_1 = 50;
int threshold_2 = 65;
int threshold_3 = 80;

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG_PIN,OUTPUT);
  digitalWrite(TRIG_PIN,LOW);

  pinMode(ECHO_PIN,INPUT);
  
  pinMode(YELLOW_LED,OUTPUT);
  digitalWrite(YELLOW_LED,LOW);
  YELLOW_STATE = false;
  pinMode(RED_LED,OUTPUT);
  digitalWrite(RED_LED,LOW);
  RED_STATE = false;
  pinMode(BLUE_LED,OUTPUT);
  digitalWrite(BLUE_LED,LOW);
  BLUE_STATE = false;
  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(GREEN_LED,LOW);
  GREEN_STATE = false;

  Serial.begin(115200);

  while(Serial.available() > 0)
  {
    Serial.read();
  }

  collision_state = false;
  initialized = false;

}

void loop() {
  // put your main code here, to run repeatedly:
  // if(Serial.available() > 0)
  // {
  //   //Read serial data
  //   receive_serial();

  //   speed = 0;

  //   speed = atoi(recv_buffer);

  //   if(speed > 0)
  //   {
  //     initialized = true;
  //   }

  // }

  if(initialized == true)
  {

    current_dist = measure_hcsr04();

  /*
    if(distance > MAX_DIST)
    {
      Serial.println("Out of range");
    }
    else
    {
      Serial.print(millis());
      Serial.print(" : ");
      Serial.print(distance);
      Serial.println(" cm");
    }
  */
    memcpy(measurements, &measurements[1], sizeof(measurements) - sizeof(float));
    measurements[5] = current_dist;
    collision_detect(current_dist,prev_dist);
    
    // TEST
    //Serial.print("Fit :");
    //Serial.println(calc_fit());
    
    //collision_watchdog();
    send_distance("$HDIST",current_dist);
    prev_dist = current_dist;

    // Wait 60ms until next measurment
    delay(60);
  }
  
}

void serialEvent()
{
  while(Serial.available())
  {
    receive_serial();
  }
  if(newMsg)
  {
    parse_incoming();
    newMsg == false;
  }
}

void parse_incoming()
{
  char* token = strtok(recv_buffer,",");
  //Serial.print("Token: ");
  //Serial.println(token);
  int x = 0;

  if(token = "$SPEED")
  {
    while(token != NULL)
    {
      if(x == 0)
      {
        token = strtok(NULL,",");
        speed = atoi(token);
        //Serial.print("Speed received: ");
        //Serial.println(speed);
      }
      else if(x == 1)
      {
        token = strtok(NULL,",");
        threshold_1 = atof(token);
        threshold_2 = threshold_1*2;
        threshold_3 = threshold_1*2.5;
        //Serial.print("threshold: ");
        //Serial.println(threshold_1);
      }
      else
      {
        token = strtok(NULL,",");
      }
      x++;
    }

    if(initialized == false && speed > 0)
    {
      initialized = true;
      //Serial.println("initialized");
    }
  }
}

void receive_serial()
{
  char marker = '\n';
  char rc;

  rc = Serial.read();

  if(rc != marker)
  {
    recv_buffer[idx] = rc;
    idx++;
    if(idx >= numChars)
    {
      idx = numChars - 1;
    }
  }
  else
  {
    recv_buffer[idx] = '\0';
    idx = 0;
    newMsg = true;
  }
}

void send_distance(char header[] ,float d)
{
  String message = header;
  message += ",";
  message += String(d);
  message += "*";

  byte checksum = 0;

  for(int i=1; i < message.length()-1; i++)
  {
    checksum ^= message[i];
  }

  if(checksum < 0x10)
  {
    message += String(0);
  }
  message += String(checksum);
  message += "\n";

  int length = message.length();

  //char buffer[length];

  //sprintf(buffer, message.c_str());

  if(Serial.availableForWrite() >= length)
  {
    Serial.write(message.c_str());
  }
  
}

void send_collision(char header[] ,int s)
{
  String message = header;
  message += ",";
  message += String(s);
  message += "*";

  //calc checksum
  byte checksum = 0;

  for(int i=1; i < message.length()-1; i++)
  {
    checksum ^= message[i];
  }

  if(checksum < 0x10)
  {
    message += String(0);
  }
  message += String(checksum);
  message += "\n";

  int length = message.length();

  //char buffer[length];

  //sprintf(buffer, message.c_str());

  if(Serial.availableForWrite() >= length)
  {
    Serial.write(message.c_str());
  }
  
}

float measure_hcsr04()
{
  unsigned long t_start;
  unsigned long t_stop;
  unsigned long pw;
  
  float result;

  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);

  while(digitalRead(ECHO_PIN)==0);

  t_start = micros();
  while(digitalRead(ECHO_PIN)==1);
  t_stop = micros();

  pw = t_stop - t_start;

  result = pw / 58.0;

  return result;
}

void collision_detect(float d,float pd)
{
  if(d <= threshold_1)
  {
    if(pd-d > 2 && collision_state == false)
    {
      collision_state = true;
      send_collision("$COLIS",1);
    }
    else if(pd-d <= 2 && collision_state == true)
    {
      collision_state = false;
      send_collision("$COLIS",0);
    }
    else
    {
      //collision_state = false;
      region1_warn();
    }
  }
  else if(d <= MAX_DIST && d > threshold_3)
  {
    region4_warn();
    collision_state = false;
  }
  else if(d <= threshold_3 && d > threshold_2)
  {
    region3_warn();
    collision_state = false;
  }
  else if(d <= threshold_2 && d > threshold_1)
  {
    region2_warn();
    collision_state = false;
  }
  else if(d > MAX_DIST)
  {
    // handle outside scope
    // all LEDs OFF
    digitalWrite(BLUE_LED,LOW);
    digitalWrite(YELLOW_LED,LOW);
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,LOW);
  }
}

void region1_warn()
{
  // RED LED
  if(YELLOW_STATE)
  {
    digitalWrite(YELLOW_LED,LOW);
    YELLOW_STATE = false;
  }
  else if(BLUE_STATE)
  {
    digitalWrite(BLUE_LED,LOW);
    BLUE_STATE = false;
  }
  else if(GREEN_STATE)
  {
    digitalWrite(GREEN_LED,LOW);
    GREEN_STATE = false;
  }
  
  digitalWrite(RED_LED,HIGH);
  RED_STATE = true;
}

void region2_warn()
{
  // YELLOW LED
  if(RED_STATE)
  {
    digitalWrite(RED_LED,LOW);
    RED_STATE = false;
  }
  else if(BLUE_STATE)
  {
    digitalWrite(BLUE_LED,LOW);
    BLUE_STATE = false;
  }
  else if(GREEN_STATE)
  {
    digitalWrite(GREEN_LED,LOW);
    GREEN_STATE = false;
  }
  
  digitalWrite(YELLOW_LED,HIGH);
  YELLOW_STATE = true;
}

void region3_warn()
{
  // GREEN LED
  if(YELLOW_STATE)
  {
    digitalWrite(YELLOW_LED,LOW);
    YELLOW_STATE = false;
  }
  else if(BLUE_STATE)
  {
    digitalWrite(BLUE_LED,LOW);
    BLUE_STATE = false;
  }
  else if(RED_STATE)
  {
    digitalWrite(RED_LED,LOW);
    RED_STATE = false;
  }
  
  digitalWrite(GREEN_LED,HIGH);
  GREEN_STATE = true;
}

void region4_warn()
{
  // BLUE LED
  if(YELLOW_STATE)
  {
    digitalWrite(YELLOW_LED,LOW);
    YELLOW_STATE = false;
  }
  else if(RED_STATE)
  {
    digitalWrite(RED_LED,LOW);
    RED_STATE = false;
  }
  else if(GREEN_STATE)
  {
    digitalWrite(GREEN_LED,LOW);
    GREEN_STATE = false;
  }
  
  digitalWrite(BLUE_LED,HIGH);
  BLUE_STATE = true;
}

void collision_watchdog()
{
  if(collision_state)
  {
    if(RED_STATE)
    {
      digitalWrite(RED_LED,LOW);
      RED_STATE = false;
    }
    else
    {
      digitalWrite(RED_LED,HIGH);
      RED_STATE = true;
    }
  }
}

float calc_fit()
{
  int sum_x,x2;
  float sum_y,y2,sum_xy,m;
  int n = 5;

  for(int i=0; i < n; i++)
  {
    sum_x += idm[i];
    sum_y += measurements[i];
    x2 += idm[i] * idm[i];
    y2 += measurements[i] * measurements[i];
    sum_xy = sum_xy + (idm[i]*measurements[i]);
  }

  m = ((n * sum_xy) - (sum_x * sum_y)) / ((n * (sum_x^2)) - (sum_x^2));

  return m;
}