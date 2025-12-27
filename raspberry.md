# Installation on Raspberry Pi   
## Create files and folders   

clone branch raspberry.   

>**RUN:** docker-compose -f docker-compose.daq.yml build --no-cache   

This builds the Docker image using a multi-stage Dockerfile.   
Build dependencies are included only in the builder stage; the runtime image is minimal.   
Then:   

>**RUN:** docker-compose -f docker-compose.daq.yml up   

the dockerfile is set up for autostart. The container starts on reboot and spins IMU and GPS nodes.   