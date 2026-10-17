"""Bước 1: huấn luyện CNN nhỏ FP32 trên MNIST bằng PyTorch, lưu trọng số ra model_fp32.npz.

Chạy:  python train_mnist.py            (tự dùng GPU nếu có)
Ảnh đưa vào mạng ở thang [0,1], KHÔNG trừ mean/chia std -> lượng tử hóa về sau đơn giản.
"""
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from mnist_data import load_mnist

torch.manual_seed(0)
np.random.seed(0)
EPOCHS, BATCH = 6, 128
dev = "cuda" if torch.cuda.is_available() else "cpu"


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 8, 3)       # 28 -> 26
        self.conv2 = nn.Conv2d(8, 16, 3)      # 13 -> 11
        self.fc = nn.Linear(16 * 5 * 5, 10)   # 11 -> pool -> 5

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv1(x)), 2)
        x = F.max_pool2d(F.relu(self.conv2(x)), 2)
        return self.fc(x.flatten(1))


def to_t(x):
    return torch.tensor(x, dtype=torch.float32, device=dev).unsqueeze(1) / 255.0


xtr, ytr, xte, yte = load_mnist()
Xtr, Ytr = to_t(xtr), torch.tensor(ytr, dtype=torch.long, device=dev)
Xte, Yte = to_t(xte), torch.tensor(yte, dtype=torch.long, device=dev)

net = Net().to(dev)
opt = torch.optim.Adam(net.parameters(), lr=2e-3)
sched = torch.optim.lr_scheduler.StepLR(opt, step_size=2, gamma=0.5)

for ep in range(EPOCHS):
    net.train()
    perm = torch.randperm(len(Xtr), device=dev)
    for i in range(0, len(perm), BATCH):
        b = perm[i:i + BATCH]
        opt.zero_grad()
        F.cross_entropy(net(Xtr[b]), Ytr[b]).backward()
        opt.step()
    sched.step()
    net.eval()
    with torch.no_grad():
        acc = (net(Xte).argmax(1) == Yte).float().mean().item()
    print(f"epoch {ep + 1}/{EPOCHS}  test acc = {acc * 100:.2f}%")

p = {k: v.detach().cpu().numpy().astype(np.float32) for k, v in net.state_dict().items()}
np.savez("model_fp32.npz",
         conv1_w=p["conv1.weight"], conv1_b=p["conv1.bias"],
         conv2_w=p["conv2.weight"], conv2_b=p["conv2.bias"],
         fc_w=p["fc.weight"], fc_b=p["fc.bias"])
print("Đã lưu model_fp32.npz")
