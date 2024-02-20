import torch

from functools import partial
from torch import nn, Tensor
from torch.nn import functional as F
from typing import Any, Callable, Dict, List, Optional, Sequence

from torchvision.models.mobilenetv2 import _make_divisible, ConvBNActivation


__all__ = ["MobileNetV3", "mobilenet_v3_large", "mobilenet_v3_small"]


model_urls = {
    "mobilenet_v3_large": "https://download.pytorch.org/models/mobilenet_v3_large-8738ca79.pth",
    "mobilenet_v3_small": "https://download.pytorch.org/models/mobilenet_v3_small-047dcff4.pth",
}

class CloseRing(torch.nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, x):
        npoints = x.shape[-1]
        return x[:, :, [-1, *[i for i in range(npoints)], 0]]


class ConvBNActivation(nn.Sequential):
    def __init__(
        self,
        in_planes: int,
        out_planes: int,
        kernel_size: int = 3,
        stride: int = 1,
        groups: int = 1,
        dilation: int = 1,
        norm_layer: Optional[Callable[..., nn.Module]] = None,
        activation_layer: Optional[Callable[..., nn.Module]] = None,
    ) -> None:
        padding = 0
        if norm_layer is None:
            norm_layer = nn.BatchNorm1d
        if activation_layer is None:
            activation_layer = nn.ReLU6
        super().__init__(
            nn.Conv1d(in_planes, out_planes, kernel_size, stride, padding, groups=groups, dilation=dilation, bias=False) if stride > 0 else
            nn.ConvTranspose1d(in_planes, out_planes, kernel_size, -stride, padding=1, output_padding=1, groups=groups, dilation=dilation, bias=False),
            norm_layer(out_planes),
            activation_layer(inplace=True)
        )
        self.out_channels = out_planes
        self.stride = stride


class SqueezeExcitation(nn.Module):
    # Implemented as described at Figure 4 of the MobileNetV3 paper
    def __init__(self, input_channels: int, squeeze_factor: int = 4):
        super().__init__()
        squeeze_channels = _make_divisible(input_channels // squeeze_factor, 8)
        self.fc1 = nn.Conv1d(input_channels, squeeze_channels, 1)
        self.relu = nn.ReLU(inplace=True)
        self.fc2 = nn.Conv1d(squeeze_channels, input_channels, 1)

    def _scale(self, input: Tensor, inplace: bool) -> Tensor:
        scale = F.adaptive_avg_pool1d(input, 1)
        scale = self.fc1(scale)
        scale = self.relu(scale)
        scale = self.fc2(scale)
        return F.hardsigmoid(scale, inplace=inplace)

    def forward(self, input: Tensor) -> Tensor:
        scale = self._scale(input, True)
        return scale * input


class InvertedResidualConfig:
    # Stores information listed at Tables 1 and 2 of the MobileNetV3 paper
    def __init__(self, input_channels: int, kernel: int, expanded_channels: int, out_channels: int, use_se: bool,
                 activation: str, stride: int, dilation: int, width_mult: float):
        self.input_channels = self.adjust_channels(input_channels, width_mult)
        self.kernel = kernel
        self.expanded_channels = self.adjust_channels(expanded_channels, width_mult)
        self.out_channels = self.adjust_channels(out_channels, width_mult)
        self.use_se = use_se
        self.use_hs = activation == "HS"
        self.stride = stride
        self.dilation = dilation

    @staticmethod
    def adjust_channels(channels: int, width_mult: float):
        return _make_divisible(channels * width_mult, 8)


class InvertedResidual(nn.Module):
    # Implemented as described at section 5 of MobileNetV3 paper
    def __init__(self, cnf: InvertedResidualConfig, norm_layer: Callable[..., nn.Module],
                 se_layer: Callable[..., nn.Module] = SqueezeExcitation):
        super().__init__()
        # if not (1 <= cnf.stride <= 2):
        #     raise ValueError('illegal stride value')

        self.use_res_connect = cnf.stride == 1 and cnf.input_channels == cnf.out_channels

        layers: List[nn.Module] = []
        activation_layer = nn.Hardswish if cnf.use_hs else nn.ReLU

        # expand
        if cnf.expanded_channels != cnf.input_channels:
            layers.append(ConvBNActivation(cnf.input_channels, cnf.expanded_channels, kernel_size=1,
                                           norm_layer=norm_layer, activation_layer=activation_layer))

        # depthwise
        if cnf.stride > 0:
            layers.append(CloseRing())
        stride = 1 if cnf.dilation > 1 else cnf.stride
        layers.append(ConvBNActivation(cnf.expanded_channels, cnf.expanded_channels, kernel_size=cnf.kernel,
                                       stride=stride, dilation=cnf.dilation, groups=cnf.expanded_channels,
                                       norm_layer=norm_layer, activation_layer=activation_layer))
        if cnf.use_se:
            layers.append(se_layer(cnf.expanded_channels))

        # project
        layers.append(ConvBNActivation(cnf.expanded_channels, cnf.out_channels, kernel_size=1, norm_layer=norm_layer,
                                       activation_layer=nn.Identity))

        self.block = nn.Sequential(*layers)
        self.out_channels = cnf.out_channels
        self._is_cn = cnf.stride > 1
        self.stride = cnf.stride

    def forward(self, input: Tensor) -> Tensor:
        # print(input.shape)
        result = self.block(input)
        if self.use_res_connect:
            result += input
        return result


class MobileNetV3(nn.Module):

    def __init__(
            self,
            inverted_residual_setting: List[InvertedResidualConfig],
            num_classes: int = 1000,
            block: Optional[Callable[..., nn.Module]] = None,
            norm_layer: Optional[Callable[..., nn.Module]] = None,
            **kwargs: Any
    ) -> None:
        """
        MobileNet V3 main class

        Args:
            inverted_residual_setting (List[InvertedResidualConfig]): Network structure
            last_channel (int): The number of channels on the penultimate layer
            num_classes (int): Number of classes
            block (Optional[Callable[..., nn.Module]]): Module specifying inverted residual building block for mobilenet
            norm_layer (Optional[Callable[..., nn.Module]]): Module specifying the normalization layer to use
        """
        super().__init__()

        if not inverted_residual_setting:
            raise ValueError("The inverted_residual_setting should not be empty")
        elif not (isinstance(inverted_residual_setting, Sequence) and
                  all([isinstance(s, InvertedResidualConfig) for s in inverted_residual_setting])):
            raise TypeError("The inverted_residual_setting should be List[InvertedResidualConfig]")

        if block is None:
            block = InvertedResidual

        if norm_layer is None:
            norm_layer = partial(nn.BatchNorm1d, eps=0.001, momentum=0.01)

        self.up_layers: torch.nn.ModuleList[nn.Module] = torch.nn.ModuleList()
        self.down_layers: torch.nn.ModuleList[nn.Module] = torch.nn.ModuleList()

        # building first layer
        firstconv_output_channels = inverted_residual_setting[0].input_channels
        self.up_layers.extend([CloseRing(), norm_layer(1)])
        self.up_layers.extend([ConvBNActivation(1, firstconv_output_channels, kernel_size=3, stride=1, norm_layer=norm_layer,
                                       activation_layer=nn.Hardswish)])

        # building inverted residual blocks
        up = True
        for cnf in inverted_residual_setting:
            if up and cnf.stride > 0:
                self.up_layers.extend([block(cnf, norm_layer)])
            else:
                up = False
                self.down_layers.extend([block(cnf, norm_layer)])

        # building last several layers
        lastconv_input_channels = inverted_residual_setting[-1].out_channels
        lastconv_output_channels = 2
        self.down_layers.append(ConvBNActivation(lastconv_input_channels, lastconv_output_channels, kernel_size=1,
                                       norm_layer=norm_layer, activation_layer=nn.Hardswish))


        for m in self.modules():
            if isinstance(m, nn.Conv1d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out')
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, (nn.BatchNorm1d, nn.GroupNorm)):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.zeros_(m.bias)

    def _forward_impl(self, x: Tensor) -> Tensor:
        # print(x.shape)
        up_feats = []
        for i, layer in enumerate(self.up_layers):
            x = layer(x)
            up_feats.append(x)
        for i, layer in enumerate(self.down_layers):
            x = layer(x)
            if layer.stride < 0:
                if x.shape[2] != up_feats[-(i + 2)].shape[2]:
                    x = x[:, :, :up_feats[-(i + 2)].shape[2]]
                x = torch.cat((x, up_feats[-(i + 2)]), 1)

        return x

    def forward(self, x: Tensor) -> Tensor:
        return self._forward_impl(x)


def _mobilenet_v3_conf(arch: str, width_mult: float = 1.0, reduced_tail: bool = False, dilated: bool = False,
                       **kwargs: Any):
    reduce_divider = 2 if reduced_tail else 1
    dilation = 2 if dilated else 1

    bneck_conf = partial(InvertedResidualConfig, width_mult=width_mult)
    adjust_channels = partial(InvertedResidualConfig.adjust_channels, width_mult=width_mult)

    if arch == "mobilenet_v3_large":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, False, "RE", 1, 1),
            bneck_conf(16, 3, 64, 24, False, "RE", 2, 1),  # C1
            bneck_conf(24, 3, 72, 24, False, "RE", 1, 1),
            bneck_conf(24, 5, 72, 40, True, "RE", 2, 1),  # C2
            bneck_conf(40, 5, 120, 40, True, "RE", 1, 1),
            bneck_conf(40, 5, 120, 40, True, "RE", 1, 1),
            bneck_conf(40, 3, 240, 80, False, "HS", 2, 1),  # C3
            bneck_conf(80, 3, 200, 80, False, "HS", 1, 1),
            bneck_conf(80, 3, 184, 80, False, "HS", 1, 1),
            bneck_conf(80, 3, 184, 80, False, "HS", 1, 1),
            bneck_conf(80, 3, 480, 112, True, "HS", 1, 1),
            bneck_conf(112, 3, 672, 112, True, "HS", 1, 1),
            bneck_conf(112, 5, 672, 160 // reduce_divider, True, "HS", 2, dilation),  # C4
            bneck_conf(160 // reduce_divider, 5, 960 // reduce_divider, 160 // reduce_divider, True, "HS", 1, dilation),
            bneck_conf(160 // reduce_divider, 5, 960 // reduce_divider, 160 // reduce_divider, True, "HS", 1, dilation),
        ]
        last_channel = adjust_channels(1280 // reduce_divider)  # C5
    elif arch == "mobilenet_v3_small":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, True, "RE", 2, 1),  # C1
            bneck_conf(16, 3, 72, 24, False, "RE", 2, 1),  # C2
            bneck_conf(24, 3, 88, 24, False, "RE", 1, 1),
            bneck_conf(24, 5, 96, 40, True, "HS", 2, 1),  # C3
            bneck_conf(40, 5, 240, 40, True, "HS", 1, 1),
            bneck_conf(40, 5, 240, 40, True, "HS", 1, 1),
            bneck_conf(40, 5, 120, 48, True, "HS", 1, 1),
            bneck_conf(48, 5, 144, 48, True, "HS", 1, 1),
            bneck_conf(48, 5, 288, 96 // reduce_divider, True, "HS", 2, dilation),  # C4
            bneck_conf(96 // reduce_divider, 5, 576 // reduce_divider, 96 // reduce_divider, True, "HS", 1, dilation),
            bneck_conf(96 // reduce_divider, 5, 576 // reduce_divider, 96 // reduce_divider, True, "HS", 1, dilation),
        ]
        last_channel = adjust_channels(1024 // reduce_divider)  # C5
    elif arch == "mobilenetPts_v3_small":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, True, "RE", 2, 1),
            bneck_conf(16, 3, 16, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 16, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    elif arch == "mobilenetPts_v3_medium":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, True, "RE", 2, 1),
            bneck_conf(16, 3, 72, 24, False, "RE", 2, 1),
            bneck_conf(24, 3, 72, 16, False, "RE", -2, 1),
            bneck_conf(32, 3, 16, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 16, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    elif arch == "mobilenetPts_v3_laarge":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, False, "RE", 2, 1),
            bneck_conf(16, 3, 72, 24, True, "RE", 2, 1),
            bneck_conf(24, 3, 88, 24, True, "RE", 2, 1),
            bneck_conf(24, 3, 128, 40, True, "RE", 2, 1),
            bneck_conf(40, 3, 128, 24, True, "RE", -2, 1),
            bneck_conf(48, 3, 88, 24, True, "RE", -2, 1),
            bneck_conf(48, 3, 72, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 32, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 32, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    elif arch == "mobilenetPts_v3_large":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, True, "RE", 2, 1),
            bneck_conf(16, 3, 72, 24, False, "RE", 2, 1),
            bneck_conf(24, 3, 88, 24, False, "RE", 2, 1),
            bneck_conf(24, 3, 88, 24, False, "RE", -2, 1),
            bneck_conf(48, 3, 72, 16, False, "RE", -2, 1),
            bneck_conf(32, 3, 32, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 32, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    elif arch == "mobilenetPts_v3_llarge":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, False, "RE", 2, 1),
            bneck_conf(16, 3, 72, 24, False, "RE", 1, 1),
            bneck_conf(24, 3, 88, 24, True, "RE", 2, 1),
            bneck_conf(24, 3, 96, 40, True, "RE", 2, 1),
            bneck_conf(40, 3, 96, 24, True, "RE", -2, 1),
            bneck_conf(48, 3, 88, 24, True, "RE", -2, 1),
            bneck_conf(48, 3, 72, 16, True, "RE", 1, 1),
            bneck_conf(16, 3, 32, 16, False, "RE", -2, 1),
            bneck_conf(32, 3, 32, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    elif arch == "mobilenetPts_v3_lmedium":
        inverted_residual_setting = [
            bneck_conf(16, 3, 16, 16, False, "RE", 2, 1),
            bneck_conf(16, 3, 72, 16, True, "RE", 1, 1),
            bneck_conf(16, 3, 88, 24, True, "RE", 2, 1),
            bneck_conf(24, 3, 88, 16, True, "RE", -2, 1),
            bneck_conf(32, 3, 72, 16, True, "RE", 1, 1),
            bneck_conf(16, 3, 72, 16, False, "RE", -2, 1),
            bneck_conf(32, 3, 16, 16, False, "RE", 1, 1),
        ]
        last_channel = 0
    else:
        raise ValueError("Unsupported model type {}".format(arch))

    return inverted_residual_setting, last_channel
