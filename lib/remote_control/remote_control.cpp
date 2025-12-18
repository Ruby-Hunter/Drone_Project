#include "remote_control.h"
#include "config.h"

void remote_control_init(msgData *msg){
    msg->x_change = 0;
    msg->y_change = 0;
    msg->z_change = 0;
}

bool isStable(msgData msg){
    return (!msg.x_change && !msg.y_change && !msg.z_change); // If all vals are 0, keep stable
}

bool xStable(msgData msg){
    return (!msg.x_change);
}

bool yStable(msgData msg){
    return (!msg.y_change);
}