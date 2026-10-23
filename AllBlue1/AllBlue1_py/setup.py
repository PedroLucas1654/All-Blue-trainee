from setuptools import find_packages, setup

package_name = "AllBlue1_py"  # igual ao <name> do package.xml

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Pedro Lucas",
    maintainer_email="plac1603@gmail.com",
    description="Nó ROS 2 para o GPS GY-NEO6MV2 (NEO-6M) via Serial",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ros2 run AllBlue1_py gps_node
            "gps_node = AllBlue1_py.main:main",
        ],
    },
)
