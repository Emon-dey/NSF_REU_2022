export recv_ip=X.X.X.X #ip of receiver
export port=1234 # port of communication

# add -u to -vln to change to udp connection
gnome -terminal -- bash -c "nc - vln $port | tar -xvf -"
sudo tcpdump -vv dst $recv_ip and port $port
