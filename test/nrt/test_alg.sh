#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

ALG_NAME=$1
SEG_TYPE=$2
OBJ_NAME=$3
OBJ_NAME_NO_ID=`echo $OBJ_NAME | cut -c5-`
DATA=./datasets/ycb-video
OUTPUT=./results/$ALG_NAME/nrt/${SEG_TYPE}/validation/${OBJ_NAME}

ALGORITHM=""
USE_OURS_OUTLIER_REJECTION=""
if [ "$ALG_NAME" == "mask-ukf" ]; then
    ALGORITHM="UKF"
    USE_OURS_OUTLIER_REJECTION="true"
elif [ "$ALG_NAME" == "icp" ]; then
    ALGORITHM="ICP"
    USE_OURS_OUTLIER_REJECTION="false"
fi

for video_id in `cat ./test/objects/${OBJ_NAME} | sed -n 2p`;
do
    mkdir -p $OUTPUT/$video_id;
    rm -f $OUTPUT/$video_id/*.txt

    ./build/bin/mask-ukf-tracker\
            --from ./config/config.cfg\
            --algorithm $ALGORITHM\
            --point_cloud_filtering::outlier_rejection $USE_OURS_OUTLIER_REJECTION\
            --segmentation::masks_set $SEG_TYPE\
            --object::object_name $OBJ_NAME\
            --object::path $DATA/$video_id\
            --log::enable_log true\
            --log::absolute_log_path $OUTPUT/$video_id\
            --autostart true;
    cp $DATA/$video_id/gt_$OBJ_NAME_NO_ID/data.log $OUTPUT/$video_id/object-tracking_ground_truth.txt;
done
