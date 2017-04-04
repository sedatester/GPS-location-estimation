# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 20:11:32 2017

@author: Sumit Saxena
"""
from geographiclib.constants import Constants
from geographiclib.geodesic import Geodesic
import urllib2
import math
import re
import os
import glob
from datetime import datetime
from operator import itemgetter
import sys

def get_flight_data(time_stamp, GPS_log):
    #dir = os.path.dirname(__file__)
    #filename = os.path.join(dir, 'Project/GPS location estimation/gps_3.txt')
    f= open(GPS_log, "r")
    text = []
    for line in f:
        line = line.replace("/", " ")
        line = line.strip()
        columns = line.split()
        #columns[1] = columns[1][:11]
        processed_line = []
        for i in range(len(columns)):
            processed_line.append(columns[i])
        text.append(processed_line)
    f.close()
    ts = datetime.strptime(time_stamp, "%H:%M:%S.%f")
    # required input format: ts = datetime.strptime('11:44:27.121100', "%H:%M:%S.%f")
    diff = []
    #test = text[:][1] - ts
    for i in range(len(text)):
        tdiff = abs(datetime.strptime(text[i][1], "%H:%M:%S.%f")-ts)
        diff.append(tdiff.seconds*1000000 + tdiff.microseconds)
    #a = min(diff)
    res = min(enumerate(diff), key=itemgetter(1))[0] 
    
    return [text[0],text[res]]
    

def cal_img_footprint(altitude, xgimbal, ygimbal, xview, yview):
    x_drone_to_imgbottom = altitude*(math.tan(xgimbal-0.5*xview))
    x_drone_to_imgtop = altitude*(math.tan(xgimbal+0.5*xview))
    
    y_drone_to_imgleft = altitude*(math.tan(ygimbal-0.5*yview))
    y_drone_to_imgright = altitude*(math.tan(ygimbal+0.5*yview))
    
    img_xlength = x_drone_to_imgtop - x_drone_to_imgbottom    
    img_ylength = y_drone_to_imgright - y_drone_to_imgleft

    return (img_xlength, img_ylength)

def getEndpoint(lt1,ln1,bearing,d):
#    R = 6378.1                   #Radius of the Earth
#    brng = math.radians(bearing) #convert degrees to radians
#    d = d/1000                  #convert nautical miles to km
#    lat1 = math.radians(lt1)
#    lon1 = math.radians(ln1)
#    lat2 = lat1 + math.asin( math.sin(lat1)*math.cos(d/R) + math.cos(lat1)*math.sin(d/R)*math.cos(brng))
#    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
#    lat2 = math.degrees(lat2)
#    lat2 = math.degrees(lon2)        
#    return lat2,lon2
    geod = Geodesic(Constants.WGS84_a, Constants.WGS84_f)
    d = geod.Direct(lt1, ln1, bearing, d)
    return d['lat2'], d['lon2']

def get_ground_altitude(drone_lat, drone_lon):
    url = 'https://maps.googleapis.com/maps/api/elevation/json?locations='+str(drone_lat)+','+str(drone_lon)+'&key=AIzaSyD6IyuGp49Qqgybr99D3kCQDn7CSk_PmCw'
    #response = urllib2.urlopen('https://maps.googleapis.com/maps/api/elevation/json?locations=39.7391536,-104.9847034&key=AIzaSyD6IyuGp49Qqgybr99D3kCQDn7CSk_PmCw')
    response = urllib2.urlopen(url)
    html = response.read()
    ground_altitude = float(html[html.find('"elevation" : ') + len('"elevation" : '):html.find(',\n         "location"')])
    return ground_altitude
    

def cal_signature_dist(sig_loc_i, sig_loc_j, time_stamp, GPS_log):#drone_lat, drone_lon, cam_bearing, asl):
# a = cal_signature_dist(1098, 349, 40.47300396737519, -79.96576386201583, 0, 3000000)
#    url = 'https://maps.googleapis.com/maps/api/elevation/json?locations='+str(drone_lat)+','+str(drone_lon)+'&key=AIzaSyD6IyuGp49Qqgybr99D3kCQDn7CSk_PmCw'
#    #response = urllib2.urlopen('https://maps.googleapis.com/maps/api/elevation/json?locations=39.7391536,-104.9847034&key=AIzaSyD6IyuGp49Qqgybr99D3kCQDn7CSk_PmCw')
#    response = urllib2.urlopen(url)
#    html = response.read()
#        
#    ground_altitude = float(html[html.find('"elevation" : ') + len('"elevation" : '):html.find(',\n         "location"')])
    
    #agl = asl - ground_altitude
    
    flight_data = get_flight_data(time_stamp, GPS_log)
    
    sloc_data = flight_data[0]
    iloc_data = flight_data[1]
    
    lat_sloc = float(sloc_data[4])
    lon_sloc = float(sloc_data[5])
    yaw_sloc = float(sloc_data[13])
    
    lat_iloc = float(iloc_data[4])
    lon_iloc = float(iloc_data[5])
    alt_iloc = float(iloc_data[7])
    yaw_iloc = float(iloc_data[13])
    drone_pitch = float(iloc_data[9])
    
    cam_angle = 48
    cam_bearing = yaw_iloc #- yaw_sloc
    
    asl_sloc = get_ground_altitude(lat_sloc, lon_sloc)
    asl_iloc = get_ground_altitude(lat_iloc, lon_iloc)
    
    agl = alt_iloc - (asl_sloc - asl_iloc)
    
    x_res = 1080
    y_res = 1920
    
    
    #xsensor = 36 # width of sensor in mm 
    #ysensor = 24 # height of sensor in mm 
    #focallen = 50 # focal length of lens in mm
    #altitude = 100 # height in m
    xgimbal = math.pi*(cam_angle + drone_pitch)/180 # x-axis gimbal angle or pitch 
    ygimbal = math.pi*0/180 # y-axis gimbal angle or roll
    
    # Calculate field of view
    xview = math.pi*44/180  #2*degrees(atan(xsensor/(2*focallen)))
    yview = math.pi*57/180  #2*degrees(atan(ysensor/(2*focallen)))
    
    # xground = altitude*(tan(90-xgimbal+0.5*xview))-altitude*(tan(90-xgimbal-0.5*xview))
    x_drone_to_imgbottom = agl*(math.tan(xgimbal-0.5*xview))
    x_drone_to_imgtop = agl*(math.tan(xgimbal+0.5*xview))
    
    y_drone_to_imgleft = agl*(math.tan(ygimbal-0.5*yview))
    y_drone_to_imgright = agl*(math.tan(ygimbal+0.5*yview))
    
    img_xdist = x_drone_to_imgtop - x_drone_to_imgbottom    
    img_ydist = y_drone_to_imgright - y_drone_to_imgleft
    
    #image_footprint = cal_img_footprint(altitude, xgimbal, ygimbal, xview, yview)
    
    # displacements calculated so that we be able to calculate the change in heading
    x_disp = (img_xdist/x_res)*(x_res - sig_loc_j) + x_drone_to_imgbottom #TO MODIFY!!!!
    #y_disp = img_ydist/y_res*(y_res - sig_loc_i) - y_drone_to_imgright
    y_disp = (img_ydist/y_res)*(sig_loc_i) + y_drone_to_imgleft
    
    if (x_disp==0):
        new_bearing = cam_bearing + math.degrees(math.atan(y_disp/0.0001))
    else:
        new_bearing = cam_bearing + math.degrees(math.atan(y_disp/x_disp))
    
    d = math.sqrt(x_disp**2 + y_disp**2)
    
    end_location = getEndpoint(lat_iloc, lon_iloc, new_bearing,d)
    
    return end_location
    
#a = cal_signature_dist(10, 80, 28.455556118519617, -80.52777807304113, 317.662819, 3000)
#c = get_flight_data('11:45:16.121100')
    

def main():
    GPS_logs_folder = sys.argv[1]
    #signatures_folder = sys.argv[2]
    
    #list_of_files = glob.glob('D:/Project/GPS location estimation/*.txt')
    GPS_logs = glob.glob(GPS_logs_folder + '*.txt')
    latest_GPS_log = max(GPS_logs, key=os.path.getctime)
    #latest_GPS_log = 'D:/Project/GPS location estimation/gps_3.txt'
    
    a = cal_signature_dist(1421, 472, '11:45:31.000000', latest_GPS_log)
    #print a 


if __name__ == "__main__":main()
#a = cal_signature_dist(60, 80, 28.455556, 80.527778, 317.662819, 10)









