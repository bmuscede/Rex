#!/bin/bash

output_results()
{
	DATE=`date`

	echo "----------------------------------------------------------" > $4
	echo -n "Run of Triage System for " >> $4
	if [ "$3" == "BEH" ]; then
		echo "Behaviour Alteration Hotspots" >> $4
	elif [ "$3" == "PUB" ]; then
		echo "Publisher Alteration Hotspots" >> $4
	else
		echo "Timer Alteration Hotspots" >> $4
	fi
	echo "" >> $4
	echo "Computed on: ${DATE}" >> $4
	echo "----------------------------------------------------------" >> $4
	echo "" >> $4
	echo "# Path Length (Highest to Lowest) #" >> $4
	cat $1 >> $4
	echo "" >> $4
	echo "----------------------------------------------------------" >> $4
	echo "" >> $4
	echo "# Hotspot Report (w/ Path Length) #" >> $4
	cat $2 >> $4
}
