#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import folium
from folium.features import DivIcon
import os
import pandas as pd
import numpy as np
import time

def return_local_time(unix_time):
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(unix_time))

def return_data_raw(df_lane, latitude='Latitude (deg)', longitude='Longitude (deg)'):
    df_lane = df_lane.dropna(subset=[latitude])
    df_lane = df_lane.dropna(subset=[longitude])
    latitude, longitude = df_lane[latitude].to_numpy(), df_lane[longitude].to_numpy()
    data_raw = list(zip(latitude, longitude))

    return data_raw, df_lane

def set_initial_map(location_point, zoom_start,  tilechoice='google_satellite'):
    tile_choices = ['google_satellite', 'Gaode_satellite']
    if tilechoice not in tile_choices:
        raise ValueError(f'please choose between {tile_choices}')

    if tilechoice == 'google_satellite':
        m = folium.Map(location=location_point,
                       zoom_start=zoom_start,
                       tiles='https://mt.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
                       attr='default'
                       )
    return m

def plot_line_with_folium(data_raw, m, color, weight=3, opacity=0.8):
    print(data_raw)
    folium.PolyLine(data_raw,
                    color='blue',
                    weight=weight,
                    opacity=opacity).add_to(m)

    return m

# %%
if __name__ == '__main__':
    
    output_folder = '/home/zhang20/Project/DCAITI_TUB_WS22/IMU_GPS_FUSION_2023'

    data1 = np.loadtxt("/home/zhang20/Project/DCAITI_TUB_WS22/IMU_GPS_FUSION_2023/build/bin/dataout_t.txt")
    lat = data1[:, 9]
    lon = data1[:, 10]
    size_data = len(lat)
    print(size_data)
    data = []

    for i in range(size_data-2):
        add_new = {"Latitude (deg)":lat[i],"Longitude (deg)":lon[i]}
        data.append(add_new) 

    df = pd.DataFrame(data)  

    data_raw, df_filtered = return_data_raw(df)
    m = set_initial_map(data_raw[0], zoom_start=14, tilechoice='google_satellite')
    m = plot_line_with_folium(data_raw, m, 'red', weight=5)
    
    ## 保存成html格式
    output_file = '{}_{}.html'.format(2023, 3)
    m.save(os.path.join(output_folder, output_file))

