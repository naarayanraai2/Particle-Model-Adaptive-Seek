import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__),'../'))
import time
from datetime import datetime
from multiprocessing import Pool, Process, Queue, Value
import json
import glob

import numpy as np

from simulator import add_noise
import simulator as simulator
from idealpath import IdealPath
from config import Config
# from preprocess.preprocess import extract_data_from_array

# from preprocess.draw_animation_from_csv import circular_trajectory_animation
from model import adaptiveSeek_vectorized

def save_config(source, destination_folder):

    with open(source, 'r') as f:
        config = json.load(f)

    dest_file = destination_folder + '/' + 'config.json'

    with open(dest_file, 'w') as f:
        json.dump(config, f)




def save_log(source,  destination_folder,csv = '0'):

    data = []
    with open(source) as f:
        data = json.load(f)


    dest_file = destination_folder + '/' + 'config_log.txt'

    with open(dest_file, 'a') as f:
        f.write("======      " + csv + "      ======      \n")
        f.write(str(data)+"   \n")
        f.write(" \n")


def extract_config_name(config_file):
    config_file = config_file.replace('\\', '/')
    name = config_file.split('/')[-1].split('.')[0]
    return name

def generate_circular_trajectory(radius, num_points):
    trajectory = {}
    points = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        points.append((x, y))
    trajectory["1_2"] = points
    with open("../trajectory/circular_trajectory.json", "w") as f:
        json.dump(trajectory, f)


if __name__ == '__main__':
    print(os.getcwd())
    os.chdir('C:/RC_Car_Lab/particle_model_adaptive_seek/Robotic-Car-Simulation/Robotic-Car-Simulation/scripts')

    sample_interval = 1
    radius = 3.66
    n_init_cars = 22
    num_points = n_init_cars if n_init_cars > 1 else 4
    center_x, center_y = 0.0, 0.0
    trajectory_file = '../trajectory/circular_trajectory.json'
    generate_circular_trajectory(radius,num_points)
    config_folder = '../config_files/calibration_with_collision/20241017_tom_config/*.json'

    for config_file in glob.glob(config_folder):
        folder_info = 'density_01_v_10_site2'

        config = Config(config_file)

        config_surfix = extract_config_name(config_file)
        file_info = folder_info + '_' + config_surfix

        for iteration_num in range(1):
            file_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            folder_time = datetime.now().strftime("%Y%m%d")

            raw_folder_name = f'../data/raw/{folder_time}_{folder_info}'
            raw_file_name = f'{raw_folder_name}/{file_time}_{file_info}_tau={iteration_num}.npy'

            clean_folder_name = f'../data/multiple_vehicle/{folder_time}_{folder_info}'
            clean_file_name = f'{clean_folder_name}/{file_time}_{file_info}_tau={iteration_num}.csv'

            avi_file_name  = f'{clean_folder_name}/{file_time}_tau={iteration_num}.avi'

            config_file_name = f'{clean_folder_name}/{file_time}_{file_info}_tau={iteration_num}_config.json'

            if not os.path.exists(raw_folder_name):
                print(raw_file_name)
                os.mkdir(raw_folder_name)

            if not os.path.exists(clean_folder_name):
                print(clean_file_name)
                os.mkdir(clean_folder_name)

            save_config(config_file, clean_folder_name)
            save_log(config_file,clean_folder_name,f'{file_time}_{file_info}')


            np.random.seed(config.seed)
            simulation_start_time = time.time()
            ideal_path = IdealPath(trajectory_file, config, sample_interval=sample_interval)

            pool = Pool(config.parallel_workers)
            que = Queue()

            car_list = []
            uid_max = 0

            log = []

            path_len = len(ideal_path._path['1_2'][0])

            for i in range(n_init_cars):
                point_index = int(i * path_len / n_init_cars)
                new_car_list, uid_max = simulator.add_new_car_fixed_path_on_circle(
                    ideal_path, config, car_list, uid_max,
                    entrance_id='1', exit_id='2', point_index=point_index, with_noise=False)
                car_list += new_car_list

            for simulation_step in range(config.simulation_steps):
                print("simu step: ", simulation_step)
                n_cars = len(car_list)
                opt_rst = []
                if n_cars>0:
                    for i in range(n_cars):
                        opt_rst.append(adaptiveSeek_vectorized(ideal_path, config, car_list, i, config.with_softmax))
                        # print("rst: ", car_list[i].uid, opt_rst[i])

                    # print("---------------------------------")

                log_states = add_noise(config, car_list, opt_rst, ideal_path, step=simulation_step)
                log.append(log_states)

                new_car_list = []
                for car in car_list:
                    if not car.arrived:
                        new_car_list.append(car)
                car_list = new_car_list

            np.save(raw_file_name, log)
            pool.close()
            pool.join()

            extract_data_from_array(raw_file_name, clean_file_name)
            print("Simulation Done.")
            print('total simulation time: ', time.time() - simulation_start_time)

            circular_trajectory_animation(
                config=config,
                trajectory_csv=clean_file_name,
                output_video=avi_file_name,
                radius=config.radius,
                wheel_base=config.car_wheel_base,
                road_width=config.lane_width,
                frame_size=(854, 854),
                fps=10
            )

            print(clean_file_name)



