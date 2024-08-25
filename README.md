# omniverse_sdg_sizing

### 1. **Run the command below to confirm your GPU driver version is 535.129.03 or later.**

```sh
$ nvidia-smi
```  
### 2. **Add NVIDIA Container Toolkit**  
For instructions on getting started with the NVIDIA Container Toolkit, refer to the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
```sh
$ sudo systemctl restart docker
```

### 3. **Pull Repo**
```sh
$ git clone https://github.com/leekunhan/omniverse_sdg_sizing.git
```   

### 4. **Change Directory**
```sh
$ cd omniverse_sdg_sizing
```  

### 5. **Follow the steps in [Generate Your NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key). Use command line to login into NGC to download the Isaac Sim container.**  
Press `+ Generate API Key`  
```sh
$ docker login nvcr.io
 Username: $oauthtoken
 Password: 
 WARNING! Your password will be stored unencrypted in /home/username/.docker/config.json.
 Configure a credential helper to remove this warning. See
 https://docs.docker.com/engine/reference/commandline/login/#credentials-store
 Login Succeeded
```

### 6. **Pull the Isaac Sim container**
```sh
$ docker pull nvcr.io/nvidia/isaac-sim:4.1.0
```  
First pull might need to wait 5~10 minutes

### 7. **Use Container Load Isaac-sim**  
```sh
$ docker run --name isaac-sim --entrypoint bash \
-it --runtime=nvidia --gpus all \
-e "ACCEPT_EULA=Y" --rm --network=host \
-e "PRIVACY_CONSENT=Y" \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v $PWD/sdg_sizing:/isaac-sim/sdg_sizing:rw \
-v $PWD/replicator_data/:/isaac-sim/replicator_data:rw \
nvcr.io/nvidia/isaac-sim:4.1.0
```  
> Note:  
> * By using the -e `ACCEPT_EULA=Y` flag, you are accepting the NVIDIA Omniverse License Agreement of the image.
> * By using the -e `PRIVACY_CONSENT=Y` flag, you opt-in to the data collection agreement found at Omniverse Data Collection & Use FAQ. You may opt-out by not setting this flag.
> * The -e `PRIVACY_USERID= flag` can optionally be set for tagging the session logs.

### 8. **Add excuting to file in container**  
```sh
$ chmod +x -R sdg_sizing/
```

## 9. **Excute it**
Dynamic SDG  
```sh
$ ./sdg_sizing/dynamic_sizing/dynamic_data_benchmark_sdg.sh
```  
or  
Statistic SDG 
```sh
$ ./sdg_sizing/statistic_sizing/statistics_data_benchmark_sdg.sh
```  