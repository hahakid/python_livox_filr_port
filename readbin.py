import numpy as np
import open3d as o3d
import os
import utils


def get_item(path):
    pcd = np.fromfile(path, dtype=np.float32).reshape(-1, 6) #[:, :3]  # 没有拿反射率
    return pcd

pc = o3d.io.read_point_cloud('./git/OpenPyLivox-master/test-Cloud.pcd')
utils.pc_show([pc])
