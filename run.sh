#!/usr/bin/env bash
# Запуск модели фигуриста на Ubuntu.
#   ./run.sh                 скорость 4 м/с, наклон 20°
#   ./run.sh 5 25           свои параметры: скорость, наклон
#   ./run.sh --test          проверки модели без окна
set -e
cd "$(dirname "$0")"

# 1. Библиотеки: компилятор, SDL2, OpenGL/GLU
NEED=""
for pkg in build-essential libsdl2-dev libglu1-mesa-dev; do
    dpkg -s "$pkg" >/dev/null 2>&1 || NEED="$NEED $pkg"
done
if [ -n "$NEED" ]; then
    echo "Устанавливаю:$NEED"
    sudo apt-get update
    sudo apt-get install -y $NEED
fi

# 2. Сборка
make

# 3. Запуск
if [ "$1" = "--test" ]; then
    ./skater --test
elif [ $# -eq 2 ]; then
    ./skater "$@"
else
    ./skater 4 20
fi
