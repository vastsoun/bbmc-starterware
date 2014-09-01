#!/bin/bash

DAY=`date +%d`
MONTH=`date +%m`
YEAR=`date +%Y`

mv $1 $2/$1
mv $2 ./"$MONTH"_"$YEAR"/"$DAY"/"$2"_`date +%d-%m-%Y_%H-%M`
