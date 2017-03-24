TYPE=$1
if [ -z $TYPE ]; then
    TYPE=release
fi

CMD="@BUILD-$TYPE/bin/servoexample -c servo.conf -l info --azim_off=209566 --elev_off=38396 -f output.txt -r 100000"

echo "RUN: '$CMD'"
$CMD
