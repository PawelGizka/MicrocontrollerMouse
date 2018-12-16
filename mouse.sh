#!/bin/bash

xCalibration=0

screenWidth=1920
screenHeight=1080

xPos=$((${screenWidth} / 2))
yPos=$((${screenHeight} / 2))

nullValue=2

calibrated=false

while IFS= read -r line; do
    if [[ $line == *"click"* ]]; then
        xdotool click 1
    fi
    if [[ $line == *","* ]]; then
        IFS=',' read -r -a arr <<< "$line"

        x=${arr[0]//[!0-9]/}
        y=${arr[1]//[!0-9]/}

        if [[ "$x" -ge 128 ]];then
            x=$((0 - (256 - ${x})))
        fi

        if [[ "$y" -ge 128 ]];then
            y=$((0 - (256 - ${y})))
        fi

        if [ ${calibrated} == false ]; then
            calibrated=true
            xCalibration=$x
            yCalibration=$y
        fi

        x=$((${x} - ${xCalibration}))
        y=$((${y} - ${yCalibration}))

        xAbs=${x#-}
        yAbs=${y#-}

        if [[ "$xAbs" -ge "$nullValue" ]]; then
            xPos=$((${xPos} + ${x}))
        fi

        if [[ "$yAbs" -ge "$nullValue" ]]; then
            yPos=$((${yPos} + ${y}))
        fi

        if [[ "$xPos" -gt "$screenWidth" ]]; then
            xPos=$screenWidth
        fi

        if [[ "$yPos" -gt "$screenHeight" ]]; then
            yPos=$screenHeight
        fi

        if [[ "$xPos" -lt 0 ]]; then
            xPos=0
        fi

        if [[ "$yPos" -lt 0 ]]; then
            yPos=0
        fi

#        printf '%s,%s  \n' "$x" "$xPos"
        xdotool mousemove ${xPos} ${yPos}

    fi


done