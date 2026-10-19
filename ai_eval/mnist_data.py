"""Tải và đọc MNIST (định dạng idx) bằng NumPy thuần, không cần torchvision."""
import gzip
import os
import urllib.request

import numpy as np

MIRRORS = [
    "https://raw.githubusercontent.com/golbin/TensorFlow-MNIST/master/mnist/data/",
    "https://ossci-datasets.s3.amazonaws.com/mnist/",
    "https://storage.googleapis.com/cvdf-datasets/mnist/",
]
FILES = ["train-images-idx3-ubyte.gz", "train-labels-idx1-ubyte.gz",
         "t10k-images-idx3-ubyte.gz", "t10k-labels-idx1-ubyte.gz"]
CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def _fetch(name):
    path = os.path.join(CACHE, name)
    if os.path.exists(path):
        return path
    os.makedirs(CACHE, exist_ok=True)
    last = None
    for base in MIRRORS:
        try:
            urllib.request.urlretrieve(base + name, path)
            return path
        except Exception as e:  # thử mirror kế tiếp
            last = e
    raise RuntimeError(f"Không tải được {name} từ mirror nào: {last}")


def _read(name, header, shape=None):
    with gzip.open(_fetch(name), "rb") as f:
        a = np.frombuffer(f.read(), dtype=np.uint8, offset=header)
    return a.reshape(shape) if shape else a


def load_mnist():
    """Trả về (x_train, y_train, x_test, y_test); ảnh uint8 [N,28,28], nhãn uint8 [N]."""
    return (_read(FILES[0], 16, (-1, 28, 28)), _read(FILES[1], 8),
            _read(FILES[2], 16, (-1, 28, 28)), _read(FILES[3], 8))


if __name__ == "__main__":
    xtr, ytr, xte, yte = load_mnist()
    print(xtr.shape, ytr.shape, xte.shape, yte.shape, xtr.max(), ytr[:10])
