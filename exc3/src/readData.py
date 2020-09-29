import rosbag_pandas as rbpd
import pandas as pd
#import pyrosbag
import rosbag

class ReadData():
    def __init__(self):
        self._data_path = 'data/data.bag'


    def read_data(self):
        df = rbpd.bag_to_dataframe(self._data_path)
        df.head()
