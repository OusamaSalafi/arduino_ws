long Temp; int d; int i=0;
int T[]={23,25,27,29,31,33,35,37,39,41}; 
int E[]={22,24,26,28,30,32,34,36,38,40}; 
int D[10];


int distance(int T,int E){
  digitalWrite(T,HIGH);
  delayMicroseconds(10);
  digitalWrite(T,LOW);
  Temp=pulseIn(E,HIGH);
  d=Temp/58;
  delayMicroseconds(5); 
  return d;
}
  
void setup(){
  for(i=0;i<10;i++){
    pinMode(T[i], OUTPUT); 
    digitalWrite(T[i], LOW); 
    pinMode(E[i], INPUT); 
  }
  Serial.begin(9600); 
}

  
void loop() {
    for(i=0;i<10;i++){
      D[i]=distance(T[i],E[i]);
      Serial.print("Distance ");
      Serial.print(i+1);
      Serial.print("=  ");
      Serial.println(D[i]);
      delay(1000);
    }
}
