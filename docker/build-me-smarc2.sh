#!/bin/bash
docker build - -t smarc2/base --build-arg UID=1000 --build-arg GID=1000 --build-arg USERNAME=ozer  < Dockerfile
