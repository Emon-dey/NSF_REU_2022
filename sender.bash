export send_ip=X.X.X.X #ip of sender
export recv_ip=X.X.X.X #ip or receiver
export port=1234 #port of connection

export topic=/tb3/cmd_vel #topic to subscribe rosbag to
export rosbag_file=send #file to save the rosbag under

# recreate the rosbag using the topic
rm -rf $rosbag_name
timeout -s SIGINT 15 ros2 bag record -o $rosbag_file $topic #record rostopic for 15 seconds and save to bag file

# add -u to -v after nc to change to udp connection
gnome-terminal -- bash -c 'tar cpf - $rosbag_file | nc -v $recv_ip $port';
sudo tcpdump src $send_ip and port $port
