# yolo_camera_detector
C++ YOLO v8s ONNX Runtime Inference code for camera cone detection

## Dependecies

TODO: Find non-conflicting versions of CUDA, CUDNN, CUDA drivers, ONNXruntime

**HOW TO INSTALL**

- ONNXRuntime

  - Download onnxruntime-linux-x64-gpu-1.X.tgz file (where X=1.10+) from https://github.com/microsoft/onnxruntime/releases
  - Go to the directory where you downloaded the file and unzip it with ```tar zxvf onnxruntime-linux-x64-gpu-1.X.tgz```
  - Place the unzipped folder (onnxruntime library) in */home/your_user_name/.amz/*

- CUDA version/drivers and CUDNN

  Before installing CUDA and its drivers make sure to uninstall any previous versions of them that already exist in your system (to avoid potential conflicts).
  To do so, follow the commands below:

  - If you have installed using **apt-get**, to remove them use:

    ```bash
    sudo apt-get --purge remove "*cublas*" "cuda*" "nsight*" # to uninstall cuda-toolkit
    sudo apt-get --purge remove "*nvidia*" # to unistall cuda-drivers
    ```
  - If you have installed via **source files**  (assuming the default location to be /usr/local), to remove them use:

    ```sudo rm -rf /usr/local/cuda* ```

   Now you are ready to install CUDA, its drivers and cudnn by following the step-by-step instructions of the following post (Make sure to install the correct versions for this project):

   https://medium.com/geekculture/install-cuda-and-cudnn-on-windows-linux-52d1501a8805



## Build
Before building this project make sure:

- To update the CMakeLists.txt file with your version of ONNXRuntime

  LINE 10: ```set(ONNXRUNTIME_DIR ~/.amz/onnxruntime-linux-x64-1.12.0)  # Local + CB```

- To update the path of yolov5s model in the configuration file (default.yaml)

  LINE 35: ```weights: "/.amz/best_2560s.onnx"```

To build the project run:
```colcon build --packages-select yolo_camera_detector```



## Good to Know
To convert different models in ONNX format, check the [official tutorial](https://github.com/ultralytics/yolov5/issues/251).

## References
- ONNXRuntime Inference code influenced by this repo: https://github.com/itsnine/yolov5-onnxruntime
- YOLO v8 repo: https://github.com/ultralytics/ultralytics
- ONNXRuntime Inference examples: https://github.com/microsoft/onnxruntime-inference-examples
