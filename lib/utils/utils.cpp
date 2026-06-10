#include "utils.h"
#include "config.h"


void proportional(Num& plant, Num val, Num desired, Num k, Num step){
  Num error = val - desired;
  if(error < 0){
    plant += (-error * k) + step;
  }
  else {
    plant -= (error * k) + step;
  }
}

void integral(Num& plant, Num* recentVals, Num desired, Num k, Num step){
  float avgErr = 0;
  for(int i = 0; i < NUM_DATA_VALS; i++){
    avgErr += recentVals[i] - desired;
  }
  avgErr /= NUM_DATA_VALS;
  if(avgErr < 0){
    plant += (-avgErr * k) + step;
  } else{
    plant -= (avgErr * k) + step;
  }
}

void derivative(Num& plant, Num curVal, Num prevVal, Num k, Num step){
  Num der = (curVal - prevVal) / 2.0;
  if(prevVal < 0){
    plant += (-der * k) + step;
  } 
  else{
    plant -= (der * k) + step;
  }
}