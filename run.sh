
#   ./run.sh                 скорость 4 м/с, наклон 20°
set -e
cd "$(dirname "$0")"

NEED=""
for pkg in build-essential libsdl2-dev libglu1-mesa-dev; do
    dpkg -s "$pkg" >/dev/null 2>&1 || NEED="$NEED $pkg"
done
if [ -n "$NEED" ]; then
    echo "Устанавливаю:$NEED"
    sudo apt-get update
    sudo apt-get install -y $NEED
fi

make

if [ "$1" = "--test" ]; then
    ./skater --test
elif [ $# -eq 2 ]; then
    ./skater "$@"
else
    ./skater 4 20
fi
