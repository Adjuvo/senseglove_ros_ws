#!/usr/bin/env bash
printf '[' > compile_commands.json
find ./build -type f -name 'compile_commands.json' -exec sh -c "cat {} | tail -n+2 | head -n-1 && printf ','" >> compile_commands.json \;
sed -i '$s/.$//' compile_commands.json
printf '\n]\n' >> compile_commands.json