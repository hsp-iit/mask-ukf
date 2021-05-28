OBJ_NAMES=`ls ./test/objects`

for obj_name in $OBJ_NAMES;
do
    bash ./test/test_icp_single.sh $1 $obj_name
done
