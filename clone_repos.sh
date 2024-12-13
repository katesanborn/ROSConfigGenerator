#!/bin/bash
set -e
while read -r repo; do
  git clone "$repo"
done < /config-data/repos.txt
