"""
Convert OSNet-AIN-x1.0 (person re-identification model) to CoreML format.

OSNet (Omni-Scale Network) is trained on person ReID datasets and produces
512-dimensional embeddings that are invariant to viewpoint, lighting, and pose.

Architecture: KaiyangZhou/deep-person-reid
Weights: osnet_ain_x1_0 pretrained on ImageNet
"""

import os
import torch
import torch.nn as nn
import torch.nn.functional as F
import coremltools as ct
import gdown


# ============================================================
# OSNet Architecture (from KaiyangZhou/deep-person-reid)
# ============================================================


class ConvLayer(nn.Module):
    """Convolution layer (conv + bn + relu)."""

    def __init__(self, in_channels, out_channels, kernel_size, stride=1, padding=0, groups=1, IN=False):
        super().__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size, stride=stride, padding=padding, bias=False, groups=groups)
        if IN:
            self.bn = nn.InstanceNorm2d(out_channels, affine=True)
        else:
            self.bn = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))


class Conv1x1(nn.Module):
    """1x1 convolution + bn + relu."""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, 1, stride=1, padding=0, bias=False)
        self.bn = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))


class Conv1x1Linear(nn.Module):
    """1x1 convolution + bn (no relu)."""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, 1, stride=1, padding=0, bias=False)
        self.bn = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        return self.bn(self.conv(x))


class LightConv3x3(nn.Module):
    """Lightweight 3x3 convolution (depthwise separable)."""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv1 = nn.Conv2d(in_channels, in_channels, 1, stride=1, padding=0, bias=False)
        self.conv2 = nn.Conv2d(in_channels, out_channels, 3, stride=1, padding=1, bias=False, groups=in_channels)
        self.bn = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv2(self.conv1(x))))


class ChannelGate(nn.Module):
    """Channel attention gate with instance normalization."""

    def __init__(self, in_channels, num_gates=None, return_gates=False, gate_activation='sigmoid', reduction=16, layer_norm=False):
        super().__init__()
        if num_gates is None:
            num_gates = in_channels
        self.return_gates = return_gates
        self.global_avgpool = nn.AdaptiveAvgPool2d(1)
        self.fc1 = nn.Conv2d(in_channels, in_channels // reduction, kernel_size=1, bias=True, padding=0)
        self.norm1 = None
        if layer_norm:
            self.norm1 = nn.LayerNorm((in_channels // reduction, 1, 1))
        self.relu = nn.ReLU(inplace=True)
        self.fc2 = nn.Conv2d(in_channels // reduction, num_gates, kernel_size=1, bias=True, padding=0)
        if gate_activation == 'sigmoid':
            self.gate_activation = nn.Sigmoid()
        elif gate_activation == 'relu':
            self.gate_activation = nn.ReLU(inplace=True)
        elif gate_activation == 'linear':
            self.gate_activation = None
        else:
            raise RuntimeError(f"Unknown gate activation: {gate_activation}")

    def forward(self, x):
        inp = x
        x = self.global_avgpool(x)
        x = self.fc1(x)
        if self.norm1 is not None:
            x = self.norm1(x)
        x = self.relu(x)
        x = self.fc2(x)
        if self.gate_activation is not None:
            x = self.gate_activation(x)
        if self.return_gates:
            return x
        return inp * x


class OSBlock(nn.Module):
    """Omni-scale feature learning block."""

    def __init__(self, in_channels, out_channels, IN=False, bottleneck_reduction=4, **kwargs):
        super().__init__()
        mid_channels = out_channels // bottleneck_reduction
        self.conv1 = Conv1x1(in_channels, mid_channels)
        self.conv2a = LightConv3x3(mid_channels, mid_channels)
        self.conv2b = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.conv2c = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.conv2d = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.gate = ChannelGate(mid_channels)
        self.conv3 = Conv1x1Linear(mid_channels, out_channels)
        self.downsample = None
        if in_channels != out_channels:
            self.downsample = Conv1x1Linear(in_channels, out_channels)
        self.IN = None
        if IN:
            self.IN = nn.InstanceNorm2d(out_channels, affine=True)

    def forward(self, x):
        identity = x
        x1 = self.conv1(x)
        x2a = self.conv2a(x1)
        x2b = self.conv2b(x1)
        x2c = self.conv2c(x1)
        x2d = self.conv2d(x1)
        x2 = self.gate(x2a) + self.gate(x2b) + self.gate(x2c) + self.gate(x2d)
        x3 = self.conv3(x2)
        if self.downsample is not None:
            identity = self.downsample(identity)
        out = x3 + identity
        if self.IN is not None:
            out = self.IN(out)
        return F.relu(out)


class OSBlockINin(nn.Module):
    """OSBlock with instance normalization inside (AIN variant)."""

    def __init__(self, in_channels, out_channels, IN=False, bottleneck_reduction=4, **kwargs):
        super().__init__()
        mid_channels = out_channels // bottleneck_reduction
        self.conv1 = Conv1x1(in_channels, mid_channels)
        self.conv2a = LightConv3x3(mid_channels, mid_channels)
        self.conv2b = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.conv2c = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.conv2d = nn.Sequential(
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
            LightConv3x3(mid_channels, mid_channels),
        )
        self.gate = ChannelGate(mid_channels)
        self.conv3 = Conv1x1Linear(mid_channels, out_channels)
        self.downsample = None
        if in_channels != out_channels:
            self.downsample = Conv1x1Linear(in_channels, out_channels)
        self.IN = None
        if IN:
            self.IN = nn.InstanceNorm2d(out_channels, affine=True)

    def forward(self, x):
        identity = x
        x1 = self.conv1(x)
        x2a = self.conv2a(x1)
        x2b = self.conv2b(x1)
        x2c = self.conv2c(x1)
        x2d = self.conv2d(x1)
        x2 = self.gate(x2a) + self.gate(x2b) + self.gate(x2c) + self.gate(x2d)
        x3 = self.conv3(x2)
        if self.downsample is not None:
            identity = self.downsample(identity)
        out = x3 + identity
        if self.IN is not None:
            out = self.IN(out)
        return F.relu(out)


class OSNet(nn.Module):
    """Omni-Scale Network for person re-identification.
    
    Reference: Zhou et al. "Omni-Scale Feature Learning for Person Re-Identification" (ICCV 2019)
    AIN variant: Zhou et al. "Learning Generalisable Omni-Scale Representations for Person Re-Identification" (TPAMI 2021)
    """

    def __init__(self, blocks, layers, channels, feature_dim=512, IN=False, **kwargs):
        super().__init__()
        num_blocks = len(blocks)
        assert num_blocks == len(layers)
        assert num_blocks == len(channels) - 1

        # Convolutional backbone
        self.conv1 = ConvLayer(3, channels[0], 7, stride=2, padding=3, IN=IN)
        self.maxpool = nn.MaxPool2d(3, stride=2, padding=1)
        self.conv2 = self._make_layer(blocks[0], layers[0], channels[0], channels[1], reduce_spatial_size=True, IN=IN)
        self.conv3 = self._make_layer(blocks[1], layers[1], channels[1], channels[2], reduce_spatial_size=True)
        self.conv4 = self._make_layer(blocks[2], layers[2], channels[2], channels[3], reduce_spatial_size=False)
        self.conv5 = Conv1x1(channels[3], channels[3])
        self.global_avgpool = nn.AdaptiveAvgPool2d(1)

        # FC layer for feature embedding
        self.fc = self._construct_fc_layer(feature_dim, channels[3])

        # Batch norm neck
        self.feature_dim = feature_dim

        self._init_params()

    def _make_layer(self, block, layer, in_channels, out_channels, reduce_spatial_size, IN=False):
        layers_list = []
        layers_list.append(block(in_channels, out_channels, IN=IN))
        for _ in range(1, layer):
            layers_list.append(block(out_channels, out_channels, IN=IN))
        if reduce_spatial_size:
            layers_list.append(nn.Sequential(
                Conv1x1(out_channels, out_channels),
                nn.AvgPool2d(2, stride=2),
            ))
        return nn.Sequential(*layers_list)

    def _construct_fc_layer(self, fc_dims, input_dim):
        if fc_dims is None or fc_dims < 0:
            self.feature_dim = input_dim
            return None
        if isinstance(fc_dims, int):
            fc_dims = [fc_dims]
        layers = []
        for dim in fc_dims:
            layers.append(nn.Linear(input_dim, dim))
            layers.append(nn.BatchNorm1d(dim))
            layers.append(nn.ReLU(inplace=True))
            input_dim = dim
        return nn.Sequential(*layers)

    def _init_params(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm1d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def featuremaps(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        x = self.conv5(x)
        return x

    def forward(self, x):
        x = self.featuremaps(x)
        v = self.global_avgpool(x)
        v = torch.flatten(v, 1)
        if self.fc is not None:
            v = self.fc(v)
        # L2 normalize the embedding
        v = F.normalize(v, p=2, dim=1)
        return v


def osnet_ain_x1_0(pretrained=True):
    """Build OSNet-AIN-x1.0 model."""
    model = OSNet(
        blocks=[OSBlockINin, OSBlockINin, OSBlockINin],
        layers=[2, 2, 2],
        channels=[64, 256, 384, 512],
        IN=True,
    )
    if pretrained:
        weight_path = "osnet_ain_x1_0_imagenet.pth"
        if not os.path.exists(weight_path):
            # Official pretrained weights from KaiyangZhou/deep-person-reid
            # https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO.html
            url = "https://drive.google.com/uc?id=1SigwBE6mPdqiJMqhuIY4aqC7--5CsMal"
            print(f"Downloading OSNet-AIN-x1.0 pretrained weights...")
            gdown.download(url, weight_path, quiet=False)
        
        state_dict = torch.load(weight_path, map_location="cpu", weights_only=True)
        # Remove classifier keys if present (we only need the feature extractor)
        keys_to_remove = [k for k in state_dict.keys() if k.startswith("classifier")]
        for k in keys_to_remove:
            del state_dict[k]
        model.load_state_dict(state_dict, strict=False)
        print(f"Loaded pretrained weights ({len(state_dict)} parameters)")
    
    return model


def convert_to_coreml(model, output_path="OSNetReID.mlpackage"):
    """Convert PyTorch OSNet model to CoreML format."""
    model.eval()

    # OSNet expects 256x128 (height x width) — standard ReID input size
    example_input = torch.randn(1, 3, 256, 128)

    # Trace the model
    print("Tracing model...")
    traced = torch.jit.trace(model, example_input)

    # Convert to CoreML
    print("Converting to CoreML...")
    import numpy as np

    mlmodel = ct.convert(
        traced,
        inputs=[
            ct.ImageType(
                name="image",
                shape=(1, 3, 256, 128),
                scale=1.0 / 255.0,
                bias=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
                color_layout=ct.colorlayout.RGB,
            )
        ],
        outputs=[
            ct.TensorType(name="embedding", dtype=np.float32)
        ],
        minimum_deployment_target=ct.target.iOS16,
        convert_to="mlprogram",
    )

    # Set model metadata
    mlmodel.author = "KaiyangZhou (deep-person-reid)"
    mlmodel.short_description = (
        "OSNet-AIN-x1.0 person re-identification model. "
        "Produces 512-dim L2-normalized embeddings from 256x128 person crops. "
        "Compare embeddings via cosine similarity for person matching."
    )
    mlmodel.input_description["image"] = "Person crop image (256x128 RGB)"
    mlmodel.output_description["embedding"] = "512-dimensional L2-normalized embedding vector"

    # Save
    mlmodel.save(output_path)
    size_mb = sum(
        os.path.getsize(os.path.join(dp, f))
        for dp, dn, filenames in os.walk(output_path)
        for f in filenames
    ) / (1024 * 1024)
    print(f"Saved CoreML model to {output_path} ({size_mb:.1f} MB)")

    return mlmodel


def verify_model(mlmodel_path="OSNetReID.mlpackage"):
    """Verify the converted model works correctly."""
    import numpy as np

    print("\nVerifying CoreML model...")
    model = ct.models.MLModel(mlmodel_path)

    # Create a dummy image (RGB, 128x256 = WxH)
    from PIL import Image
    dummy = Image.fromarray(np.random.randint(0, 255, (256, 128, 3), dtype=np.uint8))

    result = model.predict({"image": dummy})
    embedding = result["embedding"].flatten()

    print(f"  Output shape: {embedding.shape}")
    print(f"  L2 norm: {np.linalg.norm(embedding):.4f} (should be ~1.0)")
    print(f"  Min: {embedding.min():.4f}, Max: {embedding.max():.4f}")
    assert embedding.shape == (512,), f"Expected (512,), got {embedding.shape}"
    assert abs(np.linalg.norm(embedding) - 1.0) < 0.1, "Embedding should be L2-normalized"
    print("  ✓ Verification passed!")


if __name__ == "__main__":
    # Build model with pretrained weights
    model = osnet_ain_x1_0(pretrained=True)

    # Convert to CoreML
    output_path = os.path.join(os.path.dirname(__file__), "..", "RoboCar", "Models", "OSNetReID.mlpackage")
    output_path = os.path.normpath(output_path)
    mlmodel = convert_to_coreml(model, output_path)

    # Verify
    verify_model(output_path)

    print(f"\nDone! Add {output_path} to your Xcode project.")
