if [ ! -f "PyTorch_YOLOv4/data/bsw.names" ];then
    wget https://www.dropbox.com/s/y6i7ztx5eirbpcp/Micromax_torchv4_weight.zip?dl=1 -O weights.zip
    unzip weights.zip

    echo "MOVE bws.names -> PyTorch_YOLOv4/data"
    cp Micromax_torchv4_weight/bsw.names PyTorch_YOLOv4/data
    echo "MOVE bws.cfg PyTorch_YOLOv4/cfg"
    cp Micromax_torchv4_weight/bsw.cfg PyTorch_YOLOv4/cfg
    echo "MOVE best.pt PyTorch_YOLOv4/weights"
    cp Micromax_torchv4_weight/best.pt PyTorch_YOLOv4/weights/

    echo "REMOVE"
    rm weights.zip
    rm -r Micromax_torchv4_weight/
    echo "DONE."
fi


if [ ! -f "NVS/inference/weights/best.pt" ];then
    echo "Downloading NVS weights"
    wget https://www.dropbox.com/s/5yp1ft17v1155nj/weights.zip?dl=0 -O weights.zip
    unzip weights.zip

    echo "Moving best.pt to NVS/inference/weights"
    mkdir -p NVS/inference/weights
    cp weights/best.pt NVS/inference/weights/

    echo "Removing temp files"
    rm weights.zip
    rm -r weights/
    echo "Done"
fi
