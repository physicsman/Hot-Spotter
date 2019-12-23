import numpy as np
import pandas as pd
from sklearn.metrics.pairwise import cosine_similarity as cs
from copy import deepcopy as copy
from datetime import datetime
# TESTING
#import matplotlib.pyplot as plt

# Estimate distance between two GPS coordinates
def ll2m(lat1,lon1,lat2,lon2):
    d = np.asarray([lat1, lon1, lat2, lon2]) # no idea why I'm doing this
    #d = d * np.pi() / 180 # convert to radians
    c = 40075000.0 # circumfrance of earth (m)
    dx = (d[3]-d[1])*np.cos((d[0]+d[2])*np.pi/360)*c/360
    dy = (d[2]-d[0])*c/360
    return dx, dy

# Return GPS coordinates given a point on the heatmap
def m2ll(lat1, lon1, dx, dy):
    c = 40075000.0 # circumfrance of earth (m)
    lat2 = dy*360/c + lat1
    lon2 = (dx*360/c) / (np.cos((lat1+lat2)*np.pi/360)) + lon1
    return lat2, lon2

# Forgot about the whole 'dict' passing refernces ):
# I intend to wrap most of 'heatmap.py' into a nice Class

# This classless thing creates the default dict to store 
# important info to maintain the heatmap and track the vehicle 
def init_cur_reading(tmp_fov=35, max_tilt=25, max_height=35):
    # tmp_fov (deg): Temperature sensor field-of-view
    # max_tilt (deg): maximum tilt angle of sensor to consider
    # max_height (m): Maximum expected distance above ground
    
    cur_reading = {}
    cur_reading['flags'] = {}
    cur_reading['arrays'] = {}
    cur_reading['heat_map'] = {}
    cur_reading['flags']['new'] = False
    cur_reading['flags']['heat_map'] = False
    tmp_fov = tmp_fov * np.pi/180 # (deg --> rad)
    max_tilt = max_tilt * np.pi/180 # (deg --> rad)
    max_height = round(max_height)
    cur_reading['flags']['tmp_fov'] = tmp_fov
    cur_reading['flags']['max_tilt'] = max_tilt
    cur_reading['flags']['max_height'] = max_height
    
    spots = [] # A list of arrays used to project onto the ground
               # Saves from regenerating them everytime we update
    #datas = [] # A list of default data pairs for each point
               # '[running_total_temp, number_of_observations]'
    for h in range(int(max_height), 0, -1):
        #h = max_height
        # circlular spot aproximation radius
        r = round(h * np.tan(max_tilt + (tmp_fov/2)))
        if r >= 1:
            xlim  = (-int(r), int(r)) # initial grid xlim (m)
            ylim = (-int(r), int(r)) # initial grid ylim (m)
            res = int(1) # Future implement fidelity option
            y_grid = np.flip(np.tile(np.asarray([range(ylim[0],ylim[1]+1)]).T,(xlim[1]-xlim[0]+1))) / res
            x_grid = np.tile(np.asarray([range(xlim[0],xlim[1]+1)]).T,(ylim[1]-ylim[0]+1)).T / res
            zeros_grid = np.zeros_like(x_grid)*0.0 # computationally usefull dummy grid
            # mask grid ('m_grid') for circlular spot aproximation
            m_grid = np.sqrt(np.square(y_grid) + np.square(x_grid))
            m_grid = np.where(m_grid > r, 0.0, 1.0)
            y_grid *= m_grid
            x_grid *= m_grid
            xyz_array = np.array([
                x_grid.view().flatten(), 
                y_grid.view().flatten(),
                zeros_grid.view().flatten()]).T.reshape((-1,3))

            xyz_array = np.unique(xyz_array, axis=0)
            #data = [ np.array([0.0, 1.0]) for n in range(xyz_array.shape[0])]
        # FUTURE: add error handling
        spots.append(xyz_array)
        #datas.append(data)
    cur_reading['arrays']['spots'] = list(reversed(spots))
    #cur_reading['arrays']['datas'] = list(reversed(datas))

    return cur_reading

def update_heatmap(cur_reading):
    try:
        # Make sure we have at least two current readings for:
        # TeMPerature, LIDAR DiSTance, POSition, QuaTeRnion(not really)
        chk_tags = ['TMP', 'DST', 'POS', 'QTR']
        for tag in chk_tags:
            if not cur_reading[tag]['time']['t0']:
                return cur_reading
    except:
        # Testing:
        # print('except')
        return cur_reading
    # TeMPerature
    tmp = copy(cur_reading['TMP']['data']['t1'][1]) # [AmbientTempC, ObjectTempC] (c)
    t_now = copy(cur_reading['TMP']['time']['t1']) # "Current" time (s)
    # Linearly extrapolate past measurments to 't_now'
    tags = ['POS', 'DST', 'QTR']
    vals = []
    for tag in tags:
        slope = copy(cur_reading[tag]['slope'])
        t_1 = copy(cur_reading[tag]['time']['t1'])
        x_1 = copy(cur_reading[tag]['data']['t1']) # [GroundDistance] (m)
        vals.append(slope*(t_now-t_1) + x_1)     
    tmp_fov = copy(cur_reading['flags']['tmp_fov'])
    max_tilt = copy(cur_reading['flags']['max_tilt'])
    max_height = copy(cur_reading['flags']['max_height'])
    
    h_lat, h_lon, h_amsl, h_rel = copy(cur_reading['flags']['home']) # "Home"
    lat, lon, amsl, rel = tuple(vals[0])
    x, y = ll2m(h_lat,h_lon,lat,lon)
    # TESTING: 
    # print(f'Lat: {lat} Lon: {lon} Rev: {m2ll(h_lat, h_lon, x, y)}')
    _x = x - int(x)
    _y = y - int(y)
    z = vals[1][0]
    if z < 1:
        return cur_reading
    iz = int(round(z))-1
    spot = copy(cur_reading['arrays']['spots'][iz])
    _spot = copy(cur_reading['arrays']['spots'][iz])
    # Check for sign error (yup, -= _x and -= _y -> +=)
    _spot[:,0] += _x
    _spot[:,1] += _y
    _spot[:,2] += z
    #data = copy(cur_reading['arrays']['datas'][iz])
    qtr = np.array([vals[2]]) # Future: use numpy expand
    #qtr = np.array([[0.0, 0.0, 1.0]]) # TESTING
    # Project temperature sensor onto the ground given sensor attitude 
    cos_sim = cs(_spot,qtr) # Cosine distance (angle) between sensor pointing and points on the ground
    idx = np.where(cos_sim > np.cos(tmp_fov/2))
    spot = spot[idx[0]]
    spot[:,0] += int(x)
    spot[:,1] += int(y)
    #data = data[idx[0]]
    #data[:,0] += tmp
    #data = [ np.array([tmp, 1.0]) for n in range(spot.shape[0])]
    tmp_data = [ tmp for n in range(spot.shape[0])]
    div_data = [ 1.0 for n in range(spot.shape[0])]
    tuples = list(zip(spot[:,0],spot[:,1]))
    index = pd.MultiIndex.from_tuples(tuples, names=['x', 'y'])
    #series = pd.Series(data, index=index)
    tmp_series = pd.Series(tmp_data, index=index)
    div_series = pd.Series(div_data, index=index)
    if cur_reading['flags']['heat_map']:
        #cur_reading['heat_map']['series'] = cur_reading['heat_map']['series'].add(series, fill_value=0.0)
        cur_reading['heat_map']['tmp_series'] = cur_reading['heat_map']['tmp_series'].add(tmp_series, fill_value=0.0)
        cur_reading['heat_map']['div_series'] = cur_reading['heat_map']['div_series'].add(div_series, fill_value=0.0)
    else:
        #cur_reading['heat_map']['series'] = series
        cur_reading['heat_map']['tmp_series'] = tmp_series
        cur_reading['heat_map']['div_series'] = div_series
        cur_reading['flags']['heat_map'] = True
    return cur_reading

# Process data sent over from 'spotter.py'
def parse_data(cur_reading, tag, time, data):

    try:
        cur_reading[tag]['time']['t0'] = cur_reading[tag]['time']['t1']
    except:
        cur_reading[tag] = {}
        cur_reading[tag]['time'] = {}
        cur_reading[tag]['time']['t0'] = False
        cur_reading[tag]['time']['t1'] = False
        cur_reading[tag]['data'] = {}
        cur_reading[tag]['data']['t0'] = False
        cur_reading[tag]['data']['t1'] = False
        # Set flags

    cur_reading[tag]['time']['t0'] = cur_reading[tag]['time']['t1']
    cur_reading[tag]['time']['t1'] = float(time)
    cur_reading[tag]['data']['t0'] = cur_reading[tag]['data']['t1']
    cur_reading[tag]['data']['t1'] = data
    # Shouldn't reach the next line if ['time']['t1'] not set
    tags = ['QTR', 'DST', 'POS', 'TMP']
    if (tag in tags) and (cur_reading[tag]['time']['t0'] != False):
        cur_reading[tag]['slope'] = \
        (cur_reading[tag]['data']['t1'] - cur_reading[tag]['data']['t0']) \
        / (cur_reading[tag]['time']['t1'] - cur_reading[tag]['time']['t0'])

    cur_reading = process_tag(cur_reading, tag)
    return cur_reading

# Make 'tag' based decisions
def process_tag(cur_reading, tag):
    #check if tag is 'NEW' flag to reset position/et al.
    # start small grid, expand as needed
    if tag == 'NEW':
        cur_reading['flags']['new'] = True
    elif tag == 'POS':
        if cur_reading['flags']['new']:
            cur_reading['flags']['home'] = tuple(cur_reading[tag]['data']['t1'])
            cur_reading['flags']['new'] = False
        
    elif tag == 'TMP':
        ### Use OIA ??? ###
        try:
            # Check if in air and check off scale reading from temp sensor
            if (cur_reading['OIA']['data']['t1']) and (sum(cur_reading['TMP']['data']['t1']) < 800):
                cur_reading = update_heatmap(cur_reading)
    
        except:
            pass
    return cur_reading

# Find the global maximum of the heatmap and return its GPS coordinates
def get_hotspot(cur_reading):
    try:
        tmp_series = copy(cur_reading['heat_map']['tmp_series'])
        div_series = copy(cur_reading['heat_map']['div_series'])
        heatmap = tmp_series.divide(div_series)
        h_lat, h_lon, h_amsl, h_rel = copy(cur_reading['flags']['home']) # "Home"
        x_max, y_max = heatmap.idxmax()
        lat, lon = m2ll(h_lat, h_lon, x_max, y_max)
    except:
        lat = False
        lon = False
    return lat, lon

# Save the heatmap
def save_data(cur_reading):
    now = datetime.now() # current date and time
    date_time = now.strftime("%y%m%d_%H%M%S")
    logfile = ''.join(['logs/log-(', date_time, ').log'])
    try:
        tmp_series = copy(cur_reading['heat_map']['tmp_series'])
        div_series = copy(cur_reading['heat_map']['div_series'])
        tmp_series.to_pickle(''.join(['data/tmp_series-(', date_time, ').pkl']))
        div_series.to_pickle(''.join(['data/div_series-(', date_time, ').pkl']))
    except:
        return False
    return True


if __name__ == "__main__":
    print('#### HELLO ####')