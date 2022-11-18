#! /bin/sh
if [ -z "$1" ];
then
    echo "GITHUB_TOKEN is empty"
    exit 1
fi

sed -i 's|git@github\.com:|https://'"$1"'@github\.com/|g' "$2"
