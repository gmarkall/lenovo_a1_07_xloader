#!/bin/bash
##

START_TIME=`date +%s`
make distclean
make omap3621_edp1_elpida_2G_config
make ift
END_TIME=`date +%s`
let "ELAPSED_TIME=$END_TIME-$START_TIME"

