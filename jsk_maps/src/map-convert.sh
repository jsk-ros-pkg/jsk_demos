#!/bin/bash

rosrun euslisp irteusgl ./src/map-convert.l "(progn (convert-map \"${1}\" \"${2}\") (exit))"
