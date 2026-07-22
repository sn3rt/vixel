from setuptools import find_packages, setup


package_name = "vixel_network"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/systemd", ["systemd/vixel-network-setup.service.in"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="sn3rt",
    maintainer_email="sn3rt@users.noreply.github.com",
    description="Safe managed-network setup for Vixel sensor links.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vixel-network-setup = vixel_network.network_setup:main",
        ],
    },
)
