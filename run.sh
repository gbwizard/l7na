TYPE=$1
if [ -z $TYPE ]; then
    TYPE=release
fi

CMD="@BUILD-$TYPE/bin/servoexample -c servo.conf -f output.txt -r 100000"

echo "RUN: '$CMD'"
$CMD
