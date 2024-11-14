# pcl_pose の csv を x,y だけのものに変換する

import pandas as pd
import sys

X = "/pcl_pose/pose/pose/position/x"
Y = "/pcl_pose/pose/pose/position/y"

files = sys.argv[1:]

for file in files:
    df = pd.read_csv(file)
    df = df[[X, Y]]
    df.columns = ["x", "y"]
    df.to_csv(file, index=False)
