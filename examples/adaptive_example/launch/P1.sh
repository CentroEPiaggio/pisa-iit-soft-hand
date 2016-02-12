./move_hand_P1_up.sh
./open_hand.sh
./move_hand_P1.sh
sleep 4
./close_hand.sh 0.9
sleep 10
rostopic echo -n 1 /clock > P1_timing.txt
./move_hand_P1_up.sh
sleep 18
rostopic echo -n 1 /clock >> P1_timing.txt