"""
Hai mô hình cùng phương pháp FOMO (Faster Objects, More Objects - Edge Impulse):
  ảnh 96x96 xám -> backbone CNN cắt ở stride 8 -> bản đồ nhiệt tâm đối tượng 12x12 (1 logit/ô)

1) FomoMbv2   : tái tạo FOMO chuẩn của Edge Impulse (MobileNetV2 alpha 0.35, cắt tại block_6_expand,
                đầu 1x1 32 kênh -> lớp). Dùng làm mốc độ chính xác cho ESP32-S3 (triển khai thật qua
                Edge Impulse Studio + EON + ESP-NN, xem README).
2) FomoMicro  : FOMO thu nhỏ chỉ gồm conv 3x3/1x1 + ReLU (không depthwise, không residual) để
                (a) chạy bằng C thuần trên ESP32-S3 và (b) chạy trên bộ tăng tốc RTL của Tang Nano 20K
                với bộ nhân AdaReMAC. Toàn bộ trọng số ~24k tham số INT8, nằm gọn trong BSRAM.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F


# ----------------------------------------------------------------------------------------
# FOMO-Micro (đích: ESP32-S3 C thuần + FPGA Tang Nano 20K)
# ----------------------------------------------------------------------------------------
MICRO_LAYERS = [
    # (cin, cout, k, stride)  — mọi conv đều padding "same" (k//2), ReLU sau mỗi lớp trừ lớp cuối
    (1, 8, 3, 2),    # 96 -> 48
    (8, 16, 3, 2),   # 48 -> 24
    (16, 32, 3, 2),  # 24 -> 12
    (32, 32, 3, 1),  # 12
    (32, 32, 3, 1),  # 12
    (32, 1, 1, 1),   # 12x12 logit "person"
]


class FomoMicro(nn.Module):
    def __init__(self, layers=MICRO_LAYERS):
        super().__init__()
        self.cfg = layers
        mods = []
        for i, (ci, co, k, s) in enumerate(layers):
            last = i == len(layers) - 1
            mods.append(nn.Conv2d(ci, co, k, s, k // 2, bias=last))
            if not last:
                mods.append(nn.BatchNorm2d(co))
                mods.append(nn.ReLU(inplace=True))
        self.net = nn.Sequential(*mods)

    def forward(self, x):          # x: [B,1,96,96] in 0..1
        return self.net(x)[:, 0]   # [B,12,12] logit


# ----------------------------------------------------------------------------------------
# FOMO MobileNetV2-0.35 (tái tạo cấu trúc Edge Impulse)
# ----------------------------------------------------------------------------------------
def _div(v, d=8):
    nv = max(d, int(v + d / 2) // d * d)
    if nv < 0.9 * v:
        nv += d
    return nv


class _IR(nn.Module):
    def __init__(self, ci, co, s, t, cut_after_expand=False):
        super().__init__()
        hid = ci * t
        self.use_res = s == 1 and ci == co
        self.cut = cut_after_expand
        layers = []
        if t != 1:
            layers += [nn.Conv2d(ci, hid, 1, bias=False), nn.BatchNorm2d(hid), nn.ReLU6(True)]
        self.expand = nn.Sequential(*layers) if layers else nn.Identity()
        self.rest = nn.Sequential(
            nn.Conv2d(hid, hid, 3, s, 1, groups=hid, bias=False), nn.BatchNorm2d(hid), nn.ReLU6(True),
            nn.Conv2d(hid, co, 1, bias=False), nn.BatchNorm2d(co))

    def forward(self, x):
        e = self.expand(x)
        if self.cut:
            return e
        y = self.rest(e)
        return x + y if self.use_res else y


class FomoMbv2(nn.Module):
    """MobileNetV2 alpha=0.35, cắt ở đầu ra expand của block 6 (stride 8) giống Edge Impulse FOMO."""

    def __init__(self, alpha=0.35, in_ch=1, head_ch=32):
        super().__init__()
        c = lambda v: _div(v * alpha)
        first = _div(32 * alpha)
        blocks = [nn.Sequential(nn.Conv2d(in_ch, first, 3, 2, 1, bias=False), nn.BatchNorm2d(first), nn.ReLU6(True))]
        # (t, c, n, s) của MobileNetV2, chỉ tới block 6 (expand)
        settings = [(1, 16, 1, 1), (6, 24, 2, 2), (6, 32, 3, 2)]
        ci = first
        for t, cc, n, s in settings:
            co = c(cc)
            for i in range(n):
                blocks.append(_IR(ci, co, s if i == 0 else 1, t))
                ci = co
        # block_6: chỉ lấy phần expand (1x1, x6) + ReLU6
        blocks.append(_IR(ci, c(64), 2, 6, cut_after_expand=True))
        self.backbone = nn.Sequential(*blocks)
        feat = ci * 6
        self.head = nn.Sequential(nn.Conv2d(feat, head_ch, 1), nn.ReLU(True), nn.Conv2d(head_ch, 1, 1))

    def forward(self, x):
        return self.head(self.backbone(x))[:, 0]


class FomoTeacher(nn.Module):
    """FOMO với backbone MobileNetV2-1.0 ĐÃ HUẤN LUYỆN TRƯỚC trên ImageNet (torchvision), cắt ở stride 8.
    Quá nặng cho vi điều khiển (~21 M MAC) — chỉ dùng làm mô hình thầy (knowledge distillation)
    và làm mốc trần độ chính xác."""

    def __init__(self, pretrained=True):
        super().__init__()
        import torchvision
        w = "IMAGENET1K_V1" if pretrained else None
        self.backbone = torchvision.models.mobilenet_v2(weights=w).features[:7]
        self.head = nn.Sequential(nn.Conv2d(32, 32, 1), nn.ReLU(True), nn.Conv2d(32, 1, 1))

    def forward(self, x):
        x = (x - 0.45) / 0.225
        return self.head(self.backbone(x.expand(-1, 3, -1, -1)))[:, 0]


def count_macs(model, size=96):
    macs = 0

    def hook(m, i, o):
        nonlocal macs
        if isinstance(m, nn.Conv2d):
            macs += o.numel() // o.shape[0] * (m.in_channels // m.groups) * m.kernel_size[0] * m.kernel_size[1]

    hs = [m.register_forward_hook(hook) for m in model.modules() if isinstance(m, nn.Conv2d)]
    model.eval()
    with torch.no_grad():
        model(torch.zeros(1, 1, size, size))
    for h in hs:
        h.remove()
    params = sum(p.numel() for p in model.parameters())
    return macs, params


if __name__ == "__main__":
    for M in (FomoMicro, FomoMbv2):
        m = M()
        macs, p = count_macs(m)
        print(f"{M.__name__}: {macs/1e6:.2f} M MAC/khung, {p/1e3:.1f} k tham số, out {tuple(m(torch.zeros(2,1,96,96)).shape)}")
