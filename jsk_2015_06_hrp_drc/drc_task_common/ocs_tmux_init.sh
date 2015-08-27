CMDNAME=$(basename $0)
FC_IP=localhost
OCS_IP=localhost

while getopts hf:o:ka OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") OCS_IP="$OPTARG";;
        "k") tmux kill-session -t ocs; exit;;
        "a") tmux a -t ocs; exit;;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o OCS_IP]"; exit;;
    esac
done

tmux-newwindow() {
    if [ `tmux list-windows 2>/dev/null | grep $1 | sed -e 's/ //g'  >/dev/null 2>&1` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t ocs
        tmux send-keys -t ocs:$1 "$2" C-m
    fi
}

if `tmux has-session -t ocs`; then
    echo -e "\e[1;33msession named ocs already exists.\e[m"
    exit 1
else
    echo -e "\e[1;34mcreate new session named ocs.\e[m"
    tmux new-session -d -s ocs -n tmp
fi
