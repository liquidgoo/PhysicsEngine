IMAGE_NAME := sampleimage
CONTAINER_NAME := samplecont

.PHONY: build create start clean

build: 
	docker build -t $(IMAGE_NAME) .

create: build 
	docker create -ti --mount type=bind,src=.,dst=/sample --name $(CONTAINER_NAME) $(IMAGE_NAME)

start: 
	docker start -ai ${CONTAINER_NAME}

clean:
	docker container remove ${CONTAINER_NAME}


	