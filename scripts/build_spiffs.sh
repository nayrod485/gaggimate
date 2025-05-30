#!/usr/bin/env bash

# Clean data
rm -rf data/*
mkdir -p data/
mkdir -p data/profiles
touch data/profiles/.keep

# Build web application
cd web || exit
npm ci
npm run build

cp -R dist/* ../data/
