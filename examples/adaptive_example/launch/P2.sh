./move_hand_P2_up.sh
./open_hand.sh
./move_hand_P2.sh
sleep 4
./close_hand.sh 0.9
sleep 5
echo "Finished closing"
rostopic echo -n 1 /clock > P2_timing.txt
sleep 5
echo "Starting lift"
rostopic echo -n 1 /clock >> P2_timing.txt
./move_hand_P1_up.sh
read
echo "Saving time"
rostopic echo -n 1 /clock >> P2_timing.txt