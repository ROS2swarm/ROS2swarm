import torch
import yaml
import math 
from easydict import EasyDict
from ament_index_python.packages import get_package_share_directory
from ros2swarm.utils.model import SimpleModel
from ros2swarm.utils.mobilenetv3pts import _mobilenet_v3_conf, MobileNetV3

class KinDetectionFunctions: 

    @staticmethod  
    def load_model(): 
        model = None 
        #with open(get_package_share_directory('ros2swarm') + '/kin_detection_models/kin_detection_config.yaml', 'r') as f:
        
            # simple model 
            #config = EasyDict(yaml.load(f, Loader=yaml.FullLoader))
            #model = SimpleModel(config.model)
            
        model = MobileNetV3(_mobilenet_v3_conf('mobilenetPts_v3_medium')[0])
            
        model.load_state_dict(torch.load(get_package_share_directory('ros2swarm') + '/kin_detection_models/mobile_medium_best.pth', map_location=torch.device('cpu'))['model'])
        # model.eval()
        
        return model 
   
    def propagate_input(ranges, replacement, model): 
        
        ranges = [replacement if math.isinf(el) else el for el in ranges]
        ranges = torch.tensor(ranges).view(1, 1, 360)
        
        with torch.no_grad():
            out = model(ranges)
        
        out = out.tolist()
        mask = [1.0 if out[0][1][i] > out[0][0][i] else 0.0 for i in range(0, len(out[0][0]))]
        
        return mask  
        
  
        
