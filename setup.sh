# build and setup dependencies
(cd ../.. && catkin_make && rosdep install semantic_mapper)

# make the model folder if not exists
MODEL='../../model'
if [ ! -d "$MODEL" ];
then
    mkdir $MODEL
    echo "model folder created"
else
    echo "model folder exists"
fi


# download deploy.prototxt
(cd ../../model && wget -O deploy.prototxt "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=deploy.prototxt")

# download places.caffemodel
(cd ../../model && wget -O places.caffemodel "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=places.caffemodel")

# download mean.npy
(cd ../../model && wget -O mean.npy "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=mean.npy")

# download categoryIndex_places205.csv
(cd ../../model && wget -O categoryIndex_places205.csv "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=categoryIndex_places205.csv")

# download my_cats.txt
(cd ../../model && wget -O my_cats.txt "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=my_cats.txt")

# download office.bag
(cd ../../model && wget -O office.bag "https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq/download?path=%2F&files=office.bag")

MODEL_DIRECTORY=$( cd ../../ && pwd )
echo "Path to model folder : $MODEL_DIRECTORY/"

#export environment variables
export DEPLOY_PROTOTXT="$MODEL_DIRECTORY/model/deploy.prototxt"
echo $DEPLOY_PROTOTXT
export PLACES_CAFFE_MODEL="$MODEL_DIRECTORY/model/places.caffemodel" 
echo $PLACES_CAFFE_MODEL
export MEAN_NPY="$MODEL_DIRECTORY/model/mean.npy"
echo $MEAN_NPY
export CATEGORY_INDEX="$MODEL_DIRECTORY/model/categoryIndex_places205.csv"
echo $CATEGORY_INDEX
export MY_CATS_TXT="$MODEL_DIRECTORY/model/my_cats.txt"
echo $MY_CATS_TXT 
export OFFICE_BAG="$MODEL_DIRECTORY/model/office.bag"
echo $OFFICE_BAG

#run launch file
(cd ../.. && source devel/setup.bash && roslaunch semantic_mapper run_system_amcl_office.launch)