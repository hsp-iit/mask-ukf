#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

OBJ_NAMES=`ls ./test/objects`

for obj_name in $OBJ_NAMES;
do
    bash ./test/test_icp_single.sh $1 $obj_name
done
