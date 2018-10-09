#!/bin/bash

print_help()
{
	echo "Usage: ./Triage.sh [OPTIONS]"
	echo ""
	echo "This script triages the results of the control flow hotspots by examining"
	echo "the path length. The results are then added into a model system and can"
	echo "be queried in the future."
	echo ""
	echo "Options:"
	echo "-i <FILE>		: The input model to process."
	echo "-o <FILE>		: The output model to process." 
	echo "-m <BEH/PUB/TIME>	: The type of hotspot to process."
	echo "				This can be BEH (Behaviour Alteration)."
	echo "				This can be PUB (Publisher Alteration)."
	echo "				This can be TIME (Timer Alteration)."
}
