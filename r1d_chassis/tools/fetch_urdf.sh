#!/usr/bin/env bash
set +e
cd /tmp || exit 1
base=https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots
wget -qO r1_7a.urdf "$base/r1_7a_description/R1_7a.urdf" && echo "downloaded r1_7a"
wget -qO r1_a7.urdf "$base/r1_a7_description/r1_a7.urdf" && echo "downloaded r1_a7"
for f in r1_7a.urdf r1_a7.urdf; do
  echo "=== $f ==="
  wc -c "$f"
  grep -o '<robot name="[^"]*"' "$f"
  echo "joints: $(grep -c '<joint' "$f"), links: $(grep -c '<link' "$f"), meshes: $(grep -c '<mesh' "$f")"
  grep -o '<joint name="[^"]*" type="[^"]*"' "$f" | sed 's/<joint //'
  echo "--- 是否含腿/轮/升降 ---"
  grep -oE '(hip|knee|ankle|wheel|Wheel|lift|LZ_)' "$f" | sort | uniq -c
done
