wget https://www.dropbox.com/s/y6i7ztx5eirbpcp/Micromax_torchv4_weight.zip?dl=1 -O weights.zip
unzip weights.zip

echo "MOVE bws.names -> PyTorch_YOLOv4/data"
cp Micromax_torchv4_weight/bsw.names PyTorch_YOLOv4/data
echo "MOVE bws.cfg PyTorch_YOLOv4/cfg"
cp Micromax_torchv4_weight/bsw.cfg PyTorch_YOLOv4/cfg
echo "MOVE best.pt PyTorch_YOLOv4/models"
cp Micromax_torchv4_weight/best.pt PyTorch_YOLOv4/models/

echo "REMOVE"
rm weights.zip
rm -r Micromax_torchv4_weight/
echo "DONE."