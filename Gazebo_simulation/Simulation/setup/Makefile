IMAGE_NAME="go1:1.0.0"
CONTAINER_NAME="go1"

IMAGE_NAME_H="multidrones_humble:1.0.0"
CONTAINER_NAME_H="multidrones_cont_humble_01"

# Define the folder where all sub-Makefiles are stored
SUB_MAKEFILES_DIR := sub_makefiles

# Default option if none is provided
DEFAULT_OPTION := linux

# Use the 'option' variable if provided, otherwise default to DEFAULT_OPTION
OPTION ?= $(DEFAULT_OPTION)

# Dynamically include the corresponding Makefile if it exists
ifneq ("$(wildcard $(SUB_MAKEFILES_DIR)/$(OPTION).mk)","")
    include $(SUB_MAKEFILES_DIR)/$(OPTION).mk
else
    $(error Makefile for option '$(OPTION)' not found in $(SUB_MAKEFILES_DIR))
endif


## Manage source code
SRC_DIR = ./src

create_src:
	mkdir -p ./src/ros_ws/src


# Only use to clean the source code at the beginning => never use when developing
clean_repo:
	rm -rf ./src/ros_ws/src/go1_ws



sync_src:
	docker cp $(CONTAINER_NAME):/root/ros_ws/src/. $(SRC_DIR)/ros_ws/src

sync_src_humble:
	docker cp $(CONTAINER_NAME_H):/root/ros_ws/src/. $(SRC_DIR)/ros_ws/src

update_con_src:
	docker cp $(SRC_DIR)/ros_ws/src/. $(CONTAINER_NAME):/root/ros_ws/src

update_con_src_humble:
	docker cp $(SRC_DIR)/ros_ws/src/. $(CONTAINER_NAME_H):/root/ros_ws/src


######################################################

build_img:
	docker build --network host -t $(IMAGE_NAME) .

build_img_humble:
	docker build --network host -t $(IMAGE_NAME_H) -f Dockerfile.humble .

##### Docker container #####	
# create docker container
create_con: build_img
	xhost +local:root
	@docker run -it \
	--rm \
	--name $(CONTAINER_NAME) \
	$(ENVIRONMENT) \
	$(DEV) \
	$(NET) \
	$(GPU) \
	$(VOLUME) \
	$(IMAGE_NAME) \
	bash
	
	xhost -local:root

create_con_humble: build_img_humble
	xhost +local:root
	@docker run -it \
	--rm \
	--name $(CONTAINER_NAME_H) \
	$(ENVIRONMENT) \
	$(DEV) \
	$(NET) \
	$(GPU) \
	$(VOLUME) \
	$(IMAGE_NAME_H) \
	bash
	
	xhost -local:root

start_con: 
	docker start -i $(CONTAINER_NAME) 

access_con:
	docker exec -it $(CONTAINER_NAME) bash 

access_con_humble:
	docker exec -it $(CONTAINER_NAME_H) bash 


##### Clean images/containers #####
# remove docker image
clean_img: 
	# IMAGE_ID=$(cat logs/image_id.txt) 
	# @docker rmi -f $(IMAGE_ID)
	docker rmi -f $(IMAGE_NAME)
	
clean_con: 
	@docker stop $(CONTAINER_NAME)
	@docker rm $(CONTAINER_NAME)