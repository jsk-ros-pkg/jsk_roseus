#!/bin/bash

set -e

if [ -e /tmp/mongodb_store ]; then
    rm -r /tmp/mongodb_store
fi

mkdir /tmp/mongodb_store

