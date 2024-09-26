
#!/bin/bash


while true; do
  
rostopic pub -1 /right_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint9'],
    position: [255, 255, 255, 255, 255, 255, 0, 0, 0, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /right_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint8','joint9'],
    position: [255, 255, 255, 255, 255, 255, 255, 255, 255, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /right_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint8','joint9'],
    position: [255, 255, 255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  
done
