import agent
from config import Config
from idealpath import IdealPath
import model
import simulator
import run_sim

if __name__ == "__main__":
    car_list = [1, 2, 3]
    ego_car_id = 1

    config = Config('./Particle-Model-Adaptive-Seek/adaptiveseek/alt_config.json')
    ideal_path = IdealPath('./Particle-Model-Adaptive-Seek/adaptiveseek/circular_trajectory.json', config, sample_interval=1)
    model.adaptiveSeek_vectorized(ideal_path, config, car_list, ego_car_id)