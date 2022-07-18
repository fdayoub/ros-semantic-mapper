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