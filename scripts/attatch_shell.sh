#! /usr/bin/env bash
docker exec -it $(docker ps | awk 'FNR==2{print $1}') bash
