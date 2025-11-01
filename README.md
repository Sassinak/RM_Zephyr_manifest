# Engineer-manifest文件结构
## .venv
- python创建的一个虚拟环境，安装的west工具等都在虚拟环境内
***
## submanifest
- 引入的子工具，在 `west.yml`中有定义
***

## zephyr
- 指示Kconfig和cmake在本清单中的启动位置
***
## west.yml
- 说明项目的**远程仓库**，**名称**，**需要加入的模块**等构建信息
