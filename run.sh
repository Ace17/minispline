#!/usr/bin/env bash
set -euo pipefail
g++ -Wall -Wextra minispline.cpp -s `pkg-config sdl SDL_gfx --cflags --libs` -o minispline
./minispline
