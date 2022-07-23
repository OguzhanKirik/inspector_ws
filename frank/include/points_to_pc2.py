import sys
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import rospy


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg

def pointcloud2_to_array(cloud_msg, squeeze=True):
  ''' Converts a rospy PointCloud2 message to a numpy recordarray 
  
  Reshapes the returned array to have shape (height, width), even if the height is 1.

  The reason for using np.fromstring rather than struct.unpack is speed... especially
  for large point clouds, this will be <much> faster.
  '''
  # construct a numpy record type equivalent to the point type of this cloud
  dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

  # parse the cloud into an array
  cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

  # remove the dummy fields that were added
  cloud_arr = cloud_arr[
      [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
  
  if squeeze and cloud_msg.height == 1:
      return np.reshape(cloud_arr, (cloud_msg.width,))
  else:
      return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
   '''Pulls out x, y, and z columns from the cloud recordarray, and returns
       a 3xN matrix.
   '''
   # remove crap points
   if remove_nans:
       mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
       cloud_array = cloud_array[mask]
   
   # pull out x, y, and z values
   points = np.zeros(cloud_array.shape + (3,), dtype=dtype)
   points[...,0] = cloud_array['x']
   points[...,1] = cloud_array['y']
   points[...,2] = cloud_array['z']

   return points