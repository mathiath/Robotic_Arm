String encoder_value = "0.0,0.0,0.0,0.0,0.0,0.0"; //Encoder data
int robot_speed = 30;
int gripper_mode = 0;

String message = "0";
float Jlist[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // a list of all joint angles in degrees

void update_encoder_valus() {
  encoder_value = String(Jlist[0]) + "," + String(Jlist[1]) + "," + String(Jlist[2]) + "," + String(Jlist[3]) + "," + String(Jlist[4]) + "," + String(Jlist[5]);
}

void handleSerialInput() {
  message = Serial.readStringUntil('\n');
  char type = message.charAt(0);
  message.remove(0,1);
  message.trim();
  String text;

  switch (type) {
    case 'R': //Degrees for all the joints
      listCheck(message);
      break;

    case 'S': //Speed for all the joints
      //return listCheck(message);
      break;

    case 'L': //Robot speed
      robot_speed = message.toInt();
      Serial.println(String("Robot speed is: ") + String(robot_speed));
      break;

    case 'E':
      Serial.println("Encoder:" + encoder_value);
      break;

    case 'G': // activeing gripper function
      gripper_mode = message.toInt();
      if (gripper_mode == 1) {
        Serial.println("Gripper is turned on");
      }
      else if (gripper_mode == 0) {
        Serial.println("Gripper is turned off");
      }
      break;

    default:
      Serial.println("Input not defined");
      break;
  }
}

void listCheck(String message) {
  if (message[0] != '[' || message[message.length() - 1] != ']') {
    Serial.println("List input not defined");
  }
  else if (message.indexOf('[',1) != -1) {
    for (int i=0; i <= message.length(); i++) {
      if (message.indexOf('[',1) != -1) {
        message.remove(0, message.indexOf('[',1));
      }
      else if (message[0] != '[' || message[message.length() - 1] != ']') {
        Serial.println("List input not defined");
      }
      else {
        dataInToList(message);
        break;
      }
    }
  }
  else {
  dataInToList(message);
  }
}

void dataInToList(String dataList) {
  //start utgangs punkt
  int i = dataList.indexOf('[');
  int k = 0;
  //itererer gjenomm listen
  while(i++>-1) {
    if (i > 43) {
      break;
    }
    //leser frem til den finner ',' . i er start punktet
    int j = dataList.indexOf(',',i); 
    //setter verdien i Jlist til det som er i mellom i og j
    Jlist[k++]= dataList.substring(i,j).toFloat();
    i = dataList.indexOf(',',j);
  } 
}
