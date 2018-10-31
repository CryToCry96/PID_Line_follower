float Kp = 0, Ki = 0, Kd = 0; //Chinh thong so PID tai day, tim Kp truoc (Kp~0.01 > 1), sau do tim Kd
int toc_do = 140;//Dat toc do binh thuong
int toc_do_toi_da = toc_do;// toc do toi da
int toc_do_min = 0;
int toc_do_quay = toc_do;

float do_lech = 0, gia_tri_PID = 0;
float do_lech_cu = 0;
int i = 0;
int cam_bien[5] = {0, 0, 0, 0, 0};
int vtri_cu = 0;
int so_vong = 0;
unsigned int delayMillis = 0;

//#define DEBUG

void setup()
{
  pinMode(10, OUTPUT); //PWM Trai
  pinMode(11, OUTPUT); //PWM Phai
  pinMode(A2, OUTPUT); //Phai Pin 1
  pinMode(A3, OUTPUT); //Phai Pin 2
  pinMode(A4, OUTPUT); //Trai Pin 1
  pinMode(A5, OUTPUT); //Trai Pin 2
  pinMode(2, INPUT_PULLUP);//Cam bien tu 0-4 (tu trai qua phai 1-4)
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
#ifdef DEBUG
  Serial.begin(9600); //Bat serial
#endif
  delay(3000);
}

void loop()
{
  if (so_vong > 2 )
  {
    digitalWrite(A2, LOW); //Phai Pin 1
    digitalWrite(A3, LOW); //Phai Pin 2
    digitalWrite(A4, LOW); //trai Pin 1
    digitalWrite(A5, LOW); //trai Pin 2
  }
  else
  {
    doc_cam_bien();
  }
}

void doc_cam_bien()
{
  cam_bien[0] = !digitalRead(2);
  cam_bien[1] = !digitalRead(3);
  cam_bien[2] = !digitalRead(4);
  cam_bien[3] = !digitalRead(5);
  cam_bien[4] = !digitalRead(6);
  delayMicroseconds(2500);
  int vtri = 0;
  int gia_tri_cam_bien = (1000 * cam_bien[0] + 2000 * cam_bien[1] + 3000 * cam_bien[2] + 4000 * cam_bien[3] + 5000 * cam_bien[4]);
  if (gia_tri_cam_bien == 15000)
  {
    if (millis() - delayMillis > 3000)
    {
      so_vong++;
      delayMillis = (unsigned int)millis();
    }
  }
  if (gia_tri_cam_bien == 0)
  {
    if (vtri_cu > 3000)
    {
      vtri = 5000;
      di_chuyen(0, toc_do_quay, 0);//phai lui
      di_chuyen(1, toc_do_quay, 1);//trai tien
      return;
    }
    else {
      vtri = 1000;
      di_chuyen(1, toc_do_quay, 0);//trai lui
      di_chuyen(0, toc_do_quay, 1);//phai tien
      return;
    }
  }
  else
  {
    vtri = gia_tri_cam_bien / (cam_bien[0] + cam_bien[1] + cam_bien[2] + cam_bien[3] + cam_bien[4]);
    vtri_cu = vtri;
  }

  do_lech = vtri - 3000;
#ifdef DEBUG
  Serial.print(do_lech);
  Serial.print(":");
  Serial.println(vtri);
#endif
  _PID();
  tinh_toc_do();
}

//Tinh toan gia tri PID dua vao do lech
void _PID()
{
  i = i + do_lech;
  gia_tri_PID = (Kp * do_lech) + Ki * i + Kd * (do_lech - do_lech_cu);
  do_lech_cu = do_lech;
}

void tinh_toc_do()
{
  // Dat gia tri cho dong co dua vao PID
  int toc_do_trai = toc_do - gia_tri_PID;
  int toc_do_phai = toc_do + gia_tri_PID;

  if (toc_do_phai > toc_do_toi_da ) toc_do_phai = toc_do_toi_da; // Ngan khong cho gia tri vuot qua toc_do_toi_da
  if (toc_do_trai > toc_do_toi_da ) toc_do_trai = toc_do_toi_da;

  if (toc_do_phai < toc_do_min) toc_do_phai = 0;
  if (toc_do_trai < toc_do_min) toc_do_trai = 0;

  di_chuyen(0, toc_do_trai, 1);  //Bat motor trai
  di_chuyen(1, toc_do_phai, 1); //Bat motor phai
}

void di_chuyen(int motor, int pwm, int huong_di)
{
  if (motor == 1) { //trai
    analogWrite(10, pwm);  //Bat motor trai
    digitalWrite(A4, !huong_di); //trai Pin 1
    digitalWrite(A5, huong_di); //trai Pin 2
  }
  if (motor == 0) { //phai
    analogWrite(11, pwm);  //Bat motor phai
    digitalWrite(A2, huong_di); //Phai Pin 1
    digitalWrite(A3, !huong_di); //Phai Pin 2
  }
}
