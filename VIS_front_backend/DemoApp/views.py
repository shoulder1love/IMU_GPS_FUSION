from django.views import View
import json
import os
import sys
from django.conf import settings
from django.http import HttpResponse
from django.shortcuts import render, redirect
import calendar
import time
import base64
import uuid
import time

def successResponseCommon(data, message="success"):
    return HttpResponse(json.dumps({"code": 0, "data": data, "msg": message}, ensure_ascii=False), content_type="application/json")

def errorResponseCommon(data, message):
    return HttpResponse(json.dumps({"code": 1, "data": data, "msg": message}, ensure_ascii=False), content_type="application/json")

class IndexView(View):
    def get(self, request):
        return  render(request,"index.html")

#上传文件
class ExecShellScriptFileView(View):
    def post(self, request):
        if request.FILES:
            userFile = request.FILES.get('file')
            if userFile:
                ts = calendar.timegm(time.gmtime())
                name=str(ts)+get_a_uuid()
                filepath = os.path.join(settings.BASE_DIR, "tempfile/%s" % name)
                with open(filepath, 'wb') as f:
                    for content in userFile.chunks():
                        f.write(content)
                    f.close()
                result = ExecShell(filepath,name)
                return successResponseCommon(result,'ok')

def ExecShell(filepath,name):
    ####处理文件
    try:
        IMU_GPS_Path = os.path.join(settings.BASE_DIR, "helper/IMU_GPS_FUSION")
        ##输出文件路径
        Out_Path = os.path.join(settings.BASE_DIR, "tempfile/%s.txt" % name)
        print(IMU_GPS_Path, filepath, Out_Path)
        os.system("%s %s %s" % (IMU_GPS_Path, filepath, Out_Path))

        ##调用方法生成图片
        time.sleep(10)

        Pic_Path = os.path.join(settings.BASE_DIR, "media/%s.png" % name)
        Pic_Path2 = os.path.join(settings.BASE_DIR, "media/%s2.png" % name)
        Pic_Path3 = os.path.join(settings.BASE_DIR, "media/%s3.png" % name)
        Pic_Path4 = os.path.join(settings.BASE_DIR, "media/%s4.png" % name)
        MakePng(Out_Path, Pic_Path)
        MakePng2(Out_Path, Pic_Path2)
        GPS_lat_diff(Out_Path, Pic_Path3)
        GPS_lon_diff(Out_Path, Pic_Path4)
        Html_Path=os.path.join(settings.BASE_DIR, "templates/%s.html" % name)
        MakeMap(Out_Path,Html_Path)
        result = {"code": 0, "url": "media/%s.png" % name,"url2": "media/%s2.png" % name, "url3": "media/%s3.png" % name,"url4": "media/%s4.png" % name,"mapName":name }
        # return successResponseCommon(result,"ok")
        return result
    except Exception as e:
        print("未知错误 %s" % e)
        result = {"code": 500, "data": e, }
        # return errorResponseCommon(result,"error")
        return result

class ShowHtmlView(View):
    def get(self, request):
        name=request.GET.get("name","")
        return  render(request,"%s.html" % name)


from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
###生成图片
def MakePng(Out_Path,Pic_Path):
    data1 = np.loadtxt(Out_Path)
    num = data1.size
    datax = data1[:, 11]
    datay = data1[:, 12]
    dataz = data1[:, 13]
    data_gpsx = data1[:, 14]
    data_gpsy = data1[:, 15]
    data_gpsz = data1[:, 16]
    print(datax)
    print(datay)
    print(dataz)
    numx = datax.size
    print(numx)
    numy = datay.size
    print(numy)
    numz = dataz.size
    print(numz)
    # new a figure and set it into 3d
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    # set figure information
    ax.set_title("3D_Curve")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    # fig_gps = plt.figure()
    # ax_gps = fig.gca(projection='3d_gps')
    # ax_gps.set_title("3D_Curve")
    # ax_gps.set_xlabel("x")
    # ax_gps.set_ylabel("y")
    # ax_gps.set_zlabel("z")
    # draw the figure, the color is r = read
    # figure = ax.plot(datax, datay, dataz, c='r', label='fusion')
    figure = ax.plot(data_gpsx, data_gpsy, data_gpsz, c='b', label='original')
    # figure_gps = ax_gps.plot(data_gpsx, data_gpsy, data_gpsz, c='b')
    ax.legend()
    plt.savefig(Pic_Path)
    # plt.show()

def MakePng2(Out_Path,Pic_Path):
    data1 = np.loadtxt(Out_Path)
    num = data1.size
    datax = data1[:, 11]
    datay = data1[:, 12]
    dataz = data1[:, 13]
    data_gpsx = data1[:, 14]
    data_gpsy = data1[:, 15]
    data_gpsz = data1[:, 16]
    print(datax)
    print(datay)
    print(dataz)
    numx = datax.size
    print(numx)
    numy = datay.size
    print(numy)
    numz = dataz.size
    print(numz)
    # new a figure and set it into 3d
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    # set figure information
    ax.set_title("3D_Curve")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    # fig_gps = plt.figure()
    # ax_gps = fig.gca(projection='3d_gps')
    # ax_gps.set_title("3D_Curve")
    # ax_gps.set_xlabel("x")
    # ax_gps.set_ylabel("y")
    # ax_gps.set_zlabel("z")
    # draw the figure, the color is r = read
    figure = ax.plot(datax, datay, dataz, c='r', label='fusion')
    figure = ax.plot(data_gpsx, data_gpsy, data_gpsz, c='b', label='original')
    # figure_gps = ax_gps.plot(data_gpsx, data_gpsy, data_gpsz, c='b')
    ax.legend()
    plt.savefig(Pic_Path)
    # plt.show()

def GPS_lat_diff(Out_Path,Pic_Path):
    data1 = np.loadtxt(Out_Path)
    datay_f = data1[:, 6]
    datay_o = data1[:, 9]
    datax = list(range(1, len(datay_f)+1))
    plt.figure("GPS_lat_diff")
    plt.title('GPS_lat_diff',verticalalignment='bottom')
    plt.ylabel('Latitude')
    plt.xlabel('timeline')
    plt.scatter(datax, datay_o, s=0.1, c='blue',label='original')
    plt.scatter(datax, datay_f, s=0.1, c='red',label='fusion')
    plt.legend()
    # plt.show()
    plt.savefig(Pic_Path)

def GPS_lon_diff(Out_Path,Pic_Path):
    data1 = np.loadtxt(Out_Path)
    datay_f = data1[:, 7]
    datay_o = data1[:, 10]
    datax = list(range(1, len(datay_f)+1))
    plt.figure("GPS_lon_diff")
    plt.title('GPS_lon_diff',verticalalignment='bottom')
    plt.ylabel('Longitude')
    plt.xlabel('timeline')
    plt.scatter(datax, datay_o, s=0.1, c='blue',label='original')
    plt.scatter(datax, datay_f, s=0.1, c='red',label='fusion')
    plt.legend()
    plt.savefig(Pic_Path)

def GPS_lon_error(Out_Path,Pic_Path):
    data1 = np.loadtxt(Out_Path)
    datay_f = data1[:, 7]
    datay_o = data1[:, 10]
    datay_o = datay_f - datay_o
    datax = list(range(1, len(datay_f)+1))
    plt.figure("GPS_lon_diff")
    plt.title('GPS_lon_diff',verticalalignment='bottom')
    plt.ylabel('Longitude')
    plt.xlabel('timeline')
    # plt.scatter(datax, datay_o, s=0.1, c='blue',label='original')
    plt.scatter(datax, datay_e, s=0.1, c='green',label='GPS_lon_error')
    plt.legend()
    plt.savefig(Pic_Path)

def decode(s):
    try:
        return s.decode('utf-8')
    except UnicodeDecodeError:
        return s.decode('gbk')

def get_a_uuid():
    r_uuid = base64.urlsafe_b64encode(uuid.uuid4().bytes).decode()
    return r_uuid.replace('=', '')



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
    # print(data_raw)
    folium.PolyLine(data_raw,
                    color='blue',
                    weight=weight,
                    opacity=opacity).add_to(m)

    return m


def MakeMap(Out_Path,html_Path):
    data1 = np.loadtxt(Out_Path)
    lat = data1[:, 9]
    lon = data1[:, 10]
    size_data = len(lat)
    print(size_data)
    data = []

    for i in range(size_data - 2):
        add_new = {"Latitude (deg)": lat[i], "Longitude (deg)": lon[i]}
        data.append(add_new)

    df = pd.DataFrame(data)

    data_raw, df_filtered = return_data_raw(df)
    m = set_initial_map(data_raw[0], zoom_start=14, tilechoice='google_satellite')
    m = plot_line_with_folium(data_raw, m, 'red', weight=5)
    ## 保存成html格式
    m.save(html_Path)