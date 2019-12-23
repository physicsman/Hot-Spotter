#!/usr/bin/env python3
#import nest_asyncio
import time
from datetime import datetime

import asyncio
import numpy as np
import itertools as it
from queue import Queue
import logging
from mavsdk import System, ShellMessage

import heatmap

# Estimate distance between two GPS coordinates
def ll2m(lat1,lon1,lat2,lon2):
    d = np.asarray([lat1, lon1, lat2, lon2])
    #d = d * np.pi() / 180 # convert to radians
    c = 40075000 # circumfrance of earth (m)
    dx = (d[3]-d[1])*c*np.cos((d[0]+d[2])*np.pi/360)/360
    dy = (d[2]-d[0])*c/360
    return dx, dy

# Rotate a vector given a Quaternion
def rot_v(v, qat):
    v = np.array(v)
    qat = np.array(qat)
    a, b, c, d = qat[0], qat[1], qat[2], qat[3]
    R = np.array([
        [a**2 + b**2 - c**2 - d**2, 2*b*c - 2*a*d, 2*b*d + 2*a*c],
        [2*b*c + 2*a*d, a**2 - b**2 + c**2 - d**2, 2*c*d - 2*a*b],
        [2*b*d - 2*a*c, 2*c*d + 2*a*b, a**2 - b**2 - c**2 + d**2]
    ])
    return np.matmul(R.T,v)

async def next_position(drone, q): # 'POS'

    ''' Monitors position'''
    async for p in drone.telemetry.position():
        position = [
            p.latitude_deg, # Latitude in degrees (range: -90 to +90)
            p.longitude_deg, # Longitude in degrees (range: -180 to +180)
            p.absolute_altitude_m, # Altitude AMSL (above mean sea level) in metres
            p.relative_altitude_m # Altitude relative to takeoff altitude in metres
        ]
        t = time.perf_counter()
        data = np.array(position)
        #await q.put(('POS', position, t))
        q.put(('POS', data, t))

async def observe_is_in_air(drone, q): # 'OIA'
    
    """ Monitors whether the drone is flying or not"""
    async for is_in_air in drone.telemetry.in_air():
        t = time.perf_counter()
        data = np.array([is_in_air])
        #await q.put(('OIA', is_in_air, t))
        q.put(('OIA', data, t))

async def observe_is_armed(drone, q):
    
    """ Monitors whether the drone is armed or not"""
    async for is_armed in drone.telemetry.armed():
        t = time.perf_counter()
        data = np.array([is_armed])
        #await q.put(('ARM', is_in_air, t))
        q.put(('ARM', data, t))
        
async def observe_mission_end(drone, q):
    
    """ Monitors the drone for mission completion"""
    is_mission = False
    while not is_mission:
        is_mission = await drone.mission.is_mission_finished()
        t = time.perf_counter()
        data = np.array([is_mission])
        #await q.put(('ARM', is_in_air, t))
        q.put(('MXX', data, t))
        await asyncio.sleep(1)
        
async def quaternion(drone, q): # Actually returns sensor attitude 

    ''' Monitors Quaternion'''
    async for quat in drone.telemetry.attitude_quaternion():
        vec = [
            quat.w,
            quat.x,
            quat.y,
            quat.z,
        ]
        t = time.perf_counter()
        #print('quat: ',vec)
        v = [0, 0, 1] # orentation of sensor [x, y, z]
        data = rot_v(v, vec)
        #await q.put(('QTR', vec, t))
        q.put(('QTR', data, t))
        
async def temperature_and_distance(drone, q):
    
    """ Hacky Monitor of Temperature and LIDAR Distance """
    msgT = ShellMessage(1,3000, 'hg_temp\n') # ms timeout (3000 = 3sec)
    msgD = ShellMessage(1,3000, 'listener distance_sensor \n')
    while True:
        lines = await drone.shell.send(msgT)
        for l in lines.splitlines():
            if 'JJJ' in l: # 'JJJ' is a string tag I added in the firmware
                data = np.array(l[3:].split('|'), dtype=float)  
                t = time.perf_counter()
                #await q.put(('TMP', l, t))
                q.put(('TMP', data, t))
        lines = await drone.shell.send(msgD)
        for l in lines.splitlines():
            if 'current_distance:' in l:
                data = np.array([l.split(':')[1]], dtype=float)
                t = time.perf_counter()
                #await q.put(('DST', l, t))
                q.put(('DST', data, t))

# Process info and make decisions    
async def process_and_command(drone, q, cur_reading):
    t = time.perf_counter()
    data = np.array([True])
    cur_reading = heatmap.parse_data(cur_reading, 'NEW', t, data)
    n=1
    while True:
        #print('\n')
        #tag, data, t = await q.get()
        if not q.empty():
            tag, data, t = q.get()
            now = time.perf_counter()
            n+=1
            if tag == 'SAV':
                chk = heatmap.save_data(cur_reading)
                if chk:
                    continue
                    # Testing
                    #print('Saved')
            elif tag == 'MXX':
                if data[0]:
                    continue
                    # This did not go well ...
                    '''
                    lat, lon = heatmap.get_hotspot(cur_reading)
                    #Testing
                    #print('#### Lat: ',lat,' Lon: ',lon)
                    mission_items = []
                    mission_items.append(MissionItem(lat,
                                                     lon,
                                                     10,
                                                     2,
                                                     False,
                                                     float('nan'),
                                                     float('nan'),
                                                     MissionItem.CameraAction.NONE,
                                                     10,
                                                     float('nan')))
                    await drone.mission.set_return_to_launch_after_mission(True)

                    print("-- Uploading mission")
                    await drone.mission.upload_mission(mission_items)

                    #print("-- Arming")
                    #await drone.action.arm()

                    print("-- Starting mission")
                    await drone.mission.start_mission()
                    '''
            else:
                # Pass info to heatmap routine. 
                cur_reading = heatmap.parse_data(cur_reading, tag, t, data)
                logging.info(f'{tag}:{t}:{data.tolist()}')
            # Testing
            #print(n,f' {tag} reports: {data.tolist()} ')# \n with a process delay of {now-t:0.5f} seconds.')
        else:
            await asyncio.sleep(.1)  

            
async def run():
    n = 5 # 1200
    save = True
    # Future: get this wraped up in a class
    cur_reading = heatmap.init_cur_reading()
    
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")
    #await drone.connect(system_address="serial:///dev/tty.usbserial-DN05ZPQU:57600")
    await asyncio.sleep(2)
    try:
        # Having memory issues / trying to get the drone to
        # start mission before starting tasks (I don't think it helped)
        #print("-- Arming")
        await drone.action.arm()

        #print("-- Starting mission")
        await drone.mission.start_mission()
        await asyncio.sleep(5)
        
        q = Queue() # asyncio.Queue()
        t = time.perf_counter()
        tasks = [] ### asyncio.create_task() requires Python 3.7+
        tasks.append(asyncio.create_task(next_position(drone, q))) # Position Task
        tasks.append(asyncio.create_task(observe_is_in_air(drone, q))) # Observe in Air Task
        tasks.append(asyncio.create_task(observe_is_armed(drone, q))) # Observe is Armed Task
        tasks.append(asyncio.create_task(quaternion(drone, q))) # Quaternion Task
        tasks.append(asyncio.create_task(temperature_and_distance(drone, q))) # Measure Ground Temperature/Distance Task
        ###
        tasks.append(asyncio.create_task(process_and_command(drone, q, cur_reading))) # Process and Command
        '''
        #print("-- Arming")
        await drone.action.arm()

        #print("-- Starting mission")
        await drone.mission.start_mission()
        '''
        tasks.append(asyncio.create_task(observe_mission_end(drone, q))) # Observe end of Mission Task
    except Exception as e:
        print(e)
        save = False
        n = 1
    await asyncio.sleep(n)
    if save:
        q.put(('SAV', False, False))
        await asyncio.sleep(5)
    await asyncio.get_event_loop().shutdown_asyncgens()
    for t in tasks:
        t.cancel()

if __name__ == "__main__":
    now = datetime.now() # current date and time
    date_time = now.strftime("%y%m%d_%H%M%S")
    logfile = ''.join(['logs/log-(', date_time, ').log'])
    #nest_asyncio.apply()
    logging.basicConfig(level=logging.INFO, filename=logfile)
    logging.info('NEW')
    drone = System()
    start = time.perf_counter()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
