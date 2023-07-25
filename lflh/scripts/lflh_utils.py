#!/usr/bin/python3
import json
import torch
import torch.nn as nn
import torch.nn.functional as F


class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class Params:
    def __init__(self, params_fname="params.json"):
        config = json.load(open(params_fname))
        for k, v in config.items():
            self.__dict__[k] = v
        self.__dict__ = self._clean_dict(self.__dict__)

    def _clean_dict(self, _dict):
        for k, v in _dict.items():
            if v == "":  # encode empty string as None
                v = None
            if isinstance(v, dict):
                v = AttrDict(self._clean_dict(v))
            _dict[k] = v
        return _dict


class LfDModel(nn.Module):
    def __init__(self, params):
        super(LfDModel, self).__init__()
        self.model_params = params

        self.fcs = nn.ModuleList()
        prev_layer_size = self.model_params.obs_size
        for layer_size in self.model_params.layer_sizes:
            self.fcs.append(nn.Linear(prev_layer_size, layer_size))
            prev_layer_size = layer_size
        self.action_fc = nn.Linear(prev_layer_size, 2)

    def forward(self, laser, goal):
        h = torch.cat([laser, goal], dim=-1)
        for fc in self.fcs:
            h = F.leaky_relu(fc(h))
        action = self.action_fc(h)
        return action
