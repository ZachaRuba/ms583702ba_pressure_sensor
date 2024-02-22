#include <Arduino.h>

void bigPrint(uint64_t n){
  //Print unsigned long long integers (uint_64t)
  //CC (BY-NC) 2020
  //M. Eric Carr / paleotechnologist.net
  unsigned char temp;
  String result=""; //Start with a blank string
  if(n==0){Serial.println(0);return;} //Catch the zero case
  while(n){
    temp = n % 10;
    result=String(temp)+result; //Add this digit to the left of the string
    n=(n-temp)/10;      
    }//while
  Serial.println(result);
}

void bigPrint(int64_t n){
    //Print signed long long integers (uint_64t)
    //CC (BY-NC) 2020
    //M. Eric Carr / paleotechnologist.net
    unsigned char temp;
    String result=""; //Start with a blank string
    bool is_neg = false;
    if(n==0){Serial.println(0);return;} //Catch the zero case
    if (n < 0){
        n = -n;
        is_neg = true;
    }
    while(n){
        temp = n % 10;
        result=String(temp)+result; //Add this digit to the left of the string
        n=(n-temp)/10;      
    }//while
    if (is_neg == true){
        result = "-" + result;
    }
    Serial.println(result);
}