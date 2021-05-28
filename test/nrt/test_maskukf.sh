OBJ_NAMES=`ls ./test/objects`

for obj_name in $OBJ_NAMES;
do
    bash ./test/nrt/test_maskukf_single.sh $1 $obj_name
done
