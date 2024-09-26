
#!/bin/bash


while true; do
  

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint9'],
    position: [255, 255, 255, 255, 255, 255, 255, 255, 255, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 10

 
  
done
