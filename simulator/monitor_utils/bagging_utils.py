import pandas as pd
import numpy as np
import os

def nptocsv(nparr, path, name, column=None, index=None, index_csv=True):
    os.makedirs(path, exist_ok=True)
    df = pd.DataFrame(nparr, columns=column, index=index)
    filename = str(path)+'/'+str(name)+'.csv'
    print(filename)
    df.to_csv(filename, index = index_csv)

def csvtonp(filename):
    df = pd.read_csv(str(filename))
    return df.to_numpy()

def test():
    # arr = csvtonp('../gym_bringup/configuration/waypoints.csv')
    arr = csvtonp('../maps/innerhalf.csv')
    print(arr.shape)
    print(type(arr))

if __name__ == '__main__':
    test()