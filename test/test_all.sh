#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

# Test both ours and ICP baseline with Mask R-CNN and PoseCNN segmentation
bash ./test/nrt/test_maskukf.sh mrcnn
bash ./test/nrt/test_maskukf.sh posecnn
bash ./test/nrt/test_icp.sh mrcnn
bash ./test/nrt/test_icp.sh posecnn
