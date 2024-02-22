all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

build:
	@docker build --tag gmflow_vins -f ./Dockerfile ..

clean:
	@docker rmi -f gmflow_vins
