#!/bin/bash

function load_env(){
# load environment variables
local -r env_path="${1}"

if [ -z "$(ls ${env_path})" ]; then
  return 0
fi

echo "Load env files from "${env_path}"."
local filepath
for filepath in $(echo ${env_path}/*); do
    echo "    source ${filepath}"
    source "${filepath}"
done
}