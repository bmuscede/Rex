#!/bin/bash

####################################
# Triage System for Hotspots 
#
# This script produces the logic for the 
# triage system for the hotspot system.
####################################

source helper_scripts/ColourFormat.sh
source helper_scripts/UsageDisplay.sh
source helper_scripts/Outputter.sh

OUTPUT_FILE="out.txt"
EXCLUDE_FILE="ExcludeElements.txt"
TYPE="PATH"

# Performs option processing.
while getopts ":i:o:hm:e:pt:" opt; do
	case $opt in
		i)
			INPUT_MODEL=$OPTARG
      			;;
		o)
			OUTPUT_FILE=$OPTARG
			;;
		h)
			print_help
			exit 0
			;;
		m)
			# Check the model type.
			MODEL_TYPE=$OPTARG
			if [ "$MODEL_TYPE" != "BEH" ] && [ "$MODEL_TYPE" != "PUB" ] && [ "$MODEL_TYPE" != "TIME" ]; then
				error_msg "ERROR: Model type must either be BEH, PUB, or TIME!"
				print_help
				exit 1
			fi
			;;
		e)
			# Check whether the file exists.
			EXCLUDE_ELEMENTS=$OPTARG
			if [ ! -f $EXCLUDE_ELEMENTS ]; then
				error_msg "ERROR: Exclude elements file does not exist. Please check your argument."
				print_help
				exit 1
			fi
			;;
		p)
			# Output the path length.
			OUTPUT_PATH_LENGTH=true
			;;
		t)
			# Set the type.
			TYPE=$OPTARG
			if [ "$TYPE" != "PATH" ] && [ "$TYPE" != "INT" ]; then
				error_msg "Error: Type must be either PATH or INT!"
				print_help
				exit 1
			fi
      			;;
		\?)
			error_msg "ERROR: Invalid option: -$OPTARG" >&2
			print_help
			exit 1
			;;
		:)
			error_msg "Option -$OPTARG requires an argument." >&2
			print_help
			exit 1
			;;
	esac
done

# Checks if mandatory options are set.
if [ -z $INPUT_MODEL ]; then
	error_msg "ERROR: Option -i needs to be set!"
	print_help
	exit 1
fi
if [ -z $MODEL_TYPE ]; then
	error_msg "ERROR: Option -m needs to be set!"
	print_help
	exit 1
fi
if [ "$OUTPUT_FILE" == "out.txt" ]; then
	warn_msg "WARN: Output location was not set. Default output location is used!"
fi
if [ "$EXCLUDE_FILE" == "ExcludeElements.txt" ]; then
	warn_msg "WARN: Using default exclude file to remove false positives from model."
fi
if [ ! -z $OUTPUT_PATH_LENGTH ]; then
	echo "NOTE: Path length details will be outputted with triage stats."
fi

# QL check.
ql fake_file 2> /dev/null
if [ $? -ne 0 ]; then
	error_msg "ERROR: QL is not found! Make sure it's installed before proceeding."
	print_help
	exit 1
fi

# Creates temporary files.
TMP_FILE_MDL=$(mktemp /tmp/ScriptRes.XXXXXX)
TMP_FILE_TRI=$(mktemp /tmp/ScriptTriage.XXXXX)
TMP_FILE_RES=$(mktemp /tmp/ScriptTriageRes.XXXXX)
TMP_MODEL=$(mktemp /tmp/ScriptModel.XXXXX)
TMP_FILE_PT=$(mktemp /tmp/ScriptResTwo.XXXXXX)

# Injects elements into the model.
if [ -f $EXCLUDE_ELEMENTS ]; then
	TMP_REMOVE=$EXCLUDE_FILE
fi

# Get the type of analysis to perform.
if [ "$TYPE" == "PATH" ]; then
	# Get the model type.
	echo -n "Getting all path lengths from hotspot..."
	if [ "$MODEL_TYPE" == "BEH" ]; then
		ql grok_scripts/BehaviourAlterationOne.ql $INPUT_MODEL $TMP_REMOVE > $TMP_FILE_MDL 
		if [ ! -z $OUTPUT_PATH_LENGTH ]; then
			ql grok_scripts/BehaviourAlterationOne.ql $INPUT_MODEL $TMP_REMOVE true > $TMP_FILE_PT 
		fi
	elif [ "$MODEL_TYPE" == "PUB" ]; then
		ql grok_scripts/PublisherAlterationOne.ql $INPUT_MODEL $TMP_REMOVE > $TMP_FILE_MDL
		if [ ! -z $OUTPUT_PATH_LENGTH ]; then
		        ql grok_scripts/PublisherAlterationOne.ql $INPUT_MODEL $TMP_REMOVE true > $TMP_FILE_PT
		fi
	elif [ "$MODEL_TYPE" == "TIME" ]; then
		ql grok_scripts/TimerAlterationOne.ql $INPUT_MODEL $TMP_REMOVE > $TMP_FILE_MDL
		if [ ! -z $OUTPUT_PATH_LENGTH ]; then
		        ql grok_scripts/TimerAlterationOne.ql $INPUT_MODEL $TMP_REMOVE true > $TMP_FILE_PT
		fi
	fi
	success_msg "Done!"

	# Next, pretty print the results.
	echo -n "Preparing triage information..."
	python3 helper_scripts/format.py $TMP_FILE_MDL ignore > $TMP_FILE_TRI
	if [ ! -z $OUTPUT_PATH_LENGTH ]; then
		python3 helper_scripts/format.py $TMP_FILE_PT n-ignore true > $TMP_FILE_RES
	else
		python3 helper_scripts/format.py $TMP_FILE_MDL n-ignore > $TMP_FILE_RES
	fi
	success_msg "Done!"

	# Last, generates the control flow results.
	echo -n "Generating hotspot results..."
	cat $INPUT_MODEL > $TMP_MODEL
	python3 helper_scripts/model_modifier.py $TMP_FILE_TRI >> $TMP_MODEL
	if [ "$MODEL_TYPE" == "BEH" ]; then
		ql grok_scripts/BehaviourAlterationTwo.ql $TMP_MODEL $TMP_REMOVE > $TMP_FILE_MDL
	elif [ "$MODEL_TYPE" == "PUB" ]; then
		ql grok_scripts/PublisherAlterationTwo.ql $TMP_MODEL $TMP_REMOVE > $TMP_FILE_MDL
	elif [ "$MODEL_TYPE" == "TIME" ]; then
		ql grok_scripts/TimerAlterationTwo.ql $TMP_MODEL $TMP_REMOVE > $TMP_FILE_MDL
	fi
	success_msg "Done!"

	# Generates the final output.
	echo -n "Outputting results into $OUTPUT_FILE..."
	output_results $TMP_FILE_RES $TMP_FILE_MDL $MODEL_TYPE $OUTPUT_FILE
	success_msg "Done!"
else
	# Get the model type.
	echo -n "Getting all path lengths from graph..."
	ql grok_scripts/InteractionPathLength.ql $INPUT_MODEL true > $TMP_FILE_MDL
	success_msg "Done!"

	# Format the results.
	echo -n "Preparing the path length results..."
	python3 helper_scripts/interaction_format.py $TMP_FILE_MDL > $TMP_FILE_TRI
	success_msg "Done!"

	# Run the main hotspot script.
	echo -n "Generating hotspot results..."
	if [ "$MODEL_TYPE" == "BEH" ]; then
		ql grok_scripts/BehaviourAlterationThree.ql $INPUT_MODEL true > $TMP_MODEL
	elif [ "$MODEL_TYPE" == "PUB" ]; then
		ql grok_scripts/PublisherAlterationThree.ql $INPUT_MODEL true > $TMP_MODEL
	elif [ "$MODEL_TYPE" == "TIME" ]; then
		ql grok_scripts/TimerAlterationThree.ql $INPUT_MODEL true > $TMP_MODEL
	fi
	python3 helper_scripts/interaction_rank.py $TMP_FILE_TRI $TMP_MODEL > $OUTPUT_FILE
	success_msg "Done!"
fi

# Cleans the script up and ends.
rm $TMP_MODEL $TMP_FILE_MDL $TMP_FILE_TRI $TMP_FILE_RES $TMP_FILE_PT
echo ""
success_msg "Results successfully written to ${OUTPUT_FILE}!"
