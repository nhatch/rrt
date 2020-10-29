#!/bin/bash
grep -l -r "$1" . | xargs sed -i "s/$1/$2/g"

