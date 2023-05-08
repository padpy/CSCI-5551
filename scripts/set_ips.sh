if [[ $1 == "master" ]]; then
    export ROS_MASTER_URI=http://$2:11311
    export ROS_IP=$2
fi

if [[ $1 == "client" ]]; then
    export ROS_MASTER_URI=http://$2:11311
    export ROS_IP=$3
fi