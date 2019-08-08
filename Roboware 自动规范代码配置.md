# Roboware 自动规范代码配置

## 1.支持Python调试功能，需要安装pylint
```sh
sudo apt-get install python-pip
sudo python -m pip install pylint
```

## 2.为了获得更好的代码补全体验，需要安装Clang
```sh
sudo apt-get install clang-format-3.8
```

## 3.创建一个clang-format链接
```sh
sudo ln -s /usr/bin/clang-format-3.8 /usr/bin/clang-format
```

## Reference
[1] [Roboware的安装与使用](https://blog.csdn.net/qq_16397695/article/details/69524880)

[2] [‘clang-format'在Roboware中显示不可用](https://blog.csdn.net/okasy/article/details/79558388)

