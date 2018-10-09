#!/bin/bash

NC='\033[0m'

error_msg()
{
	RED='\033[0;31m'
	echo -e "${RED}$1${NC}"
}

success_msg()
{
	GREEN='\033[0;32m'
	echo -e "${GREEN}$1${NC}"
}

warn_msg()
{
	YELLOW='\033[1;33m'
	echo -e "${YELLOW}$1${NC}"
}
