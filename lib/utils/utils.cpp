#include "utils.h"
#include "config.h"
#include <cstdlib>


void proportional(uint16_t& plant, float val, float desired, float k, float step){
  float error = val - desired;
  float change = abs(error * k) + step;
  if(error < 0){
    plant += static_cast<int16_t>(change);
  }
  else {
    plant -= static_cast<int16_t>(change);
  }
}

void integral(uint16_t& plant, float* recentVals, float desired, float k, float step){
  float avgErr = 0;
  for(int i = 0; i < NUM_DATA_VALS; i++){
    avgErr += recentVals[i] - desired;
  }
  float change = abs((avgErr / NUM_DATA_VALS) * k) + step;
  if(avgErr < 0){
    plant += static_cast<int16_t>(change);
  } else{
    plant -= static_cast<int16_t>(change);
  }
}

void derivative(uint16_t& plant, float curVal, float prevVal, float desired, float k, float step){
  float der = (curVal - prevVal) / 2.0;
  float change = abs(der * k) + step;
  if(curVal < desired){
    plant += static_cast<int16_t>(change);
  } 
  else{
    plant -= static_cast<int16_t>(change);
  }
}