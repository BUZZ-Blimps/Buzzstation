#!/bin/bash

cd ../src
export NODE_TLS_REJECT_UNAUTHORIZED='0'
yarn start --tunnel --port 8080
