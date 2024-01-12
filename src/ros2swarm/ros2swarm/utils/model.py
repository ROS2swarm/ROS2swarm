import torch
from ros2swarm.utils.mobilenetv3pts import MobileNetV3, _mobilenet_v3_conf


class CloseRing(torch.nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, x):
        npoints = x.shape[-1]
        return x[:, :, [-1, *[i for i in range(npoints)], 0]]


def conv_layer(ch_in, ch_out):
    layer = torch.nn.Sequential(
        torch.nn.BatchNorm1d(ch_in),
        CloseRing(),
        torch.nn.Conv1d(ch_in, ch_out, 3),
        torch.nn.ReLU()
    )
    return layer


class SimpleModel(torch.nn.Module):

    def __init__(self, config):
        super().__init__()

        tmp_layers = []
        for l in range(len(config.LAYERS) - 1):
            tmp_layers.append(conv_layer(config.LAYERS[l], config.LAYERS[l + 1]))

        self.layers = torch.nn.Sequential(*tmp_layers)

    def forward(self, x):
        return self.layers(x)


def build_model(config):
    if config.NAME == 'simple':
        return SimpleModel(config)
    elif config.NAME == 'mobile':
        mobile_layer_conf, _ = _mobilenet_v3_conf('mobilenetPts_v3_small')
        return MobileNetV3(mobile_layer_conf)
    else:
        print(f"No model {config.NAME} implemented")
        return None
