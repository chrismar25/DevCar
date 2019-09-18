int read_b(void){ //Read the distance b from a LIDAR sensor.
  int s1;
      sB.startContinuous(T); 
      s1 = sB.read()/10;//Read b. 
      sB.stopContinuous(); 
      if (sB.timeoutOccurred()) { Serial.print("TIMEOUT for sB: Distance out of range for timing budget"); }
      Serial.print("b:");
      Serial.print(s1);
      Serial.println("cm");
      return s1;
}
int read_c(void){ //Read distance c from the ultrasonic sensor.
  int s2;
      s2 = sC.ping_cm();//Ping and measure c. 
      delay(dt_Pings3);
      if (s2 == 0 || s2 >= MaxDist) //The distance is either above the upper distance limit or the supplied power is unstable.
        s2 = MaxDist; //An erroneous sensor reading should not cause an exit from the loop.
      Serial.print("c:");
      Serial.print(s2);
      Serial.println("cm");
      return s2;
}
int read_d(void){ //Read the distance d from a LIDAR sensor.
   int s1;
      sD.startContinuous(T); 
      s1 = sD.read()/10;//Read d. 
      sD.stopContinuous(); 
      if (sD.timeoutOccurred()) { Serial.print("TIMEOUT for sB: Distance out of range for timing budget"); }
      Serial.print("d:");
      Serial.print(s1);
      Serial.println("cm");
      return s1;
}
