
#!/bin/bash


while true; do
  
  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint1'],
    position: [1, 255, 255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint1'],
    position: [255, 255, 255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint2'],
    position: [255, 1, 255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint2'],
    position: [255, 255, 255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint10'],
    position: [255, 255, 255, 255, 255, 255, 128, 128, 128, 1, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint10'],
    position: [255, 255, 255, 255, 255, 255, 128, 128, 128, 255, 0, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"

  sleep 1

  
done
